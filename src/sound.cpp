// SquachWatch-CYD — the speaker. See sound.h.
//
// Freenove, from their schematics and Sketch_07.1_Music: an ES8311 codec at
// I2C 0x18 on the touch bus, fed over I2S, driving a small mono amplifier
// whose SHUTDOWN pin is GPIO1 on both boards (AUDIO_EN on the schematic;
// their sketch drives it LOW to play). The I2S pins differ:
//
//   2.8" (FNK0104B): MCK 4, BCK 5, WS 7, DOUT 8, DIN 6
//   3.5" (FNK0104N): MCK 17, BCK 18, WS 21, DOUT 15, DIN 16
//
// None of them is used by anything else on either board (the 3.5"'s 15-18
// and 21 are free; the 2.8"'s 4-8 are free), checked against PINOUT.md.
//
// The codec is set up once, at boot, over Wire -- the loop task owns Wire
// (the touch controller is polled on it), so the player never touches I2C.
// Loudness is done in the samples; silence is the amplifier's enable pin,
// which also means an idle board spends nothing on sound: the amp is in
// shutdown (under a microamp) and the I2S clocks are stopped.
//
// Playback is a small FreeRTOS task fed by a queue. On the codec boards it
// renders the whole sound into PSRAM, then streams it 256 frames at a time
// into the I2S driver's DMA buffers and blocks on the write, so a slow frame
// on the drawing side never puts a gap in a sound. Four 256-frame buffers is
// 64 ms of DMA, 4 KB of internal RAM. The 2.8" CYD has no PSRAM and its
// framebuffer already owns the largest internal block, so that board renders
// each chunk as it plays, into the onboard amp on DAC2 (GPIO26).
#include "sound.h"
#if SQW_SPEAKER
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#if defined(CYD)
#include <driver/dac.h>
#endif
#include "settings.h"
#include "clock.h"

namespace Sound {

#if defined(FREENOVE_S3_35)
static const int PIN_MCK = 17, PIN_BCK = 18, PIN_WS = 21, PIN_DOUT = 15;
static const int PIN_AMP = 1;
#elif defined(TDECK_PLUS)
// The T-Deck Plus: a MAX98357A class-D amp straight on I2S (LilyGo's
// BOARD_I2S_BCK 7 / WS 5 / DOUT 6). No codec to set up, no master clock, and
// no enable pin of its own -- it sleeps when the clocks stop.
static const int PIN_MCK = -1, PIN_BCK = 7, PIN_WS = 5, PIN_DOUT = 6;
static const int PIN_AMP = -1;
#define SOUND_NO_CODEC 1
#else
static const int PIN_MCK = 4,  PIN_BCK = 5,  PIN_WS = 7,  PIN_DOUT = 8;
static const int PIN_AMP = 1;
#endif
#ifndef SOUND_NO_CODEC
#define SOUND_NO_CODEC 0
#endif
static const uint8_t    CODEC   = 0x18;
static const uint32_t   RATE    = 16000;      // plenty for tones under 4 kHz
static const i2s_port_t PORT    = I2S_NUM_0;
static const int        CHUNK   = 256;        // frames per write, 16 ms

static bool          s_ok      = false;
static bool          s_ampLow  = true;        // the amp plays with the pin LOW
static volatile bool s_hush    = false;
static QueueHandle_t s_q       = nullptr;
static TaskHandle_t  s_task    = nullptr;
#if defined(CYD)
static portMUX_TYPE  s_mux     = portMUX_INITIALIZER_UNLOCKED;
#endif

// ---- the sounds ------------------------------------------------------------
// A sound is a short list of parts, played one after another unless a part is
// marked WITH, which starts it together with the part before (a clank under a
// spring); `off` nudges a part's start by that many ms, `gap` is silence after
// it. Every part has an attack (a, ms), a level held until h percent of the
// way (or a ringing fade, for a bell), a level lv in percent, and COS for a
// rounded fade instead of a straight one. The kinds:
//
//   TONE   a wave gliding through the pitches p[] (Hz, 0 = unused), with an
//          optional wobble: k0 depth, k1 rate (Hz), k2 the ms it fades over
//   NOISE  the first Sound Lab's low-passed noise burst
//   VOICE  a throat buzzing through the pitches p[], shaped by the mouth
//          vowels v[]: k0 mouth size (under 1 = a bigger creature, 0 = 1),
//          k1 breath, k2 an "h" at the start (ms), k3 vibrato depth
//   BELL   two-operator FM, carrier p[0]: k0 the modulator's ratio, k1 its
//          brightness, k2 the ring (ms), k3 how fast the brightness goes, as a
//          share of the ring (0 = half)
//   AIR    noise through a band-pass gliding through the centres p[], k0 = Q
//
// Sound Lab 2 (2026-10-14) rendered these same recipes in the browser, sample
// for sample, and the picks below are the owner's. The ones kept from the
// first Sound Lab (2026-10-11) are written with BEEP and sound as before.
enum : uint8_t { TONE = 0, NOISE, VOICE, BELL, AIR };
enum : uint8_t { SQ = 0, TRI, SIN, SAW };
enum : uint8_t { WITH = 1, COS = 2 };
enum : uint8_t { V_END = 0, OO, OH, AH, UH, EH, EE, IH, MM };
struct Part {
    uint8_t  kind, wave, flags;
    int16_t  off;
    uint16_t ms, gap;
    uint8_t  a, h, lv;
    uint16_t p[4];
    uint8_t  v[4];
    float    k[4];
};
struct Sound { const Part* p; uint8_t n; uint16_t echoMs; float fb, mix, gain; };

#define BEEP(f, to, ms, w, gap) { TONE, w, 0, 0, ms, gap, 4, 60, 100, { f, to } }
static const Part NEW_DEVICE[] = { BEEP(660, 0, 45, TRI, 25), BEEP(880, 0, 45, TRI, 25), BEEP(1100, 0, 60, TRI, 0) };
static const Part THROW[]      = { BEEP(300, 1500, 260, SIN, 0) };
static const Part LAND[]       = { { NOISE, 0, 0, 0, 30, 0, 4, 60, 100 }, BEEP(180, 70, 60, SQ, 0) };
static const Part TOAST_POP[]  = { BEEP(200, 1800, 80, SQ, 0) };
#undef BEEP
// The Bigfoot whoop, twice, echoing through the trees.
static const Part WATCH_HIT[]  = {
    { VOICE, 0, COS, 0, 520, 120, 25, 70, 100, { 240, 300, 500 }, { OO, OO, OO, OH }, { 0, 0.12f } },
    { VOICE, 0, COS, 0, 560, 0,   25, 70, 100, { 240, 300, 520 }, { OO, OO, OO, OH }, { 0, 0.12f } } };
static const Part HEADS_UP[]   = {
    { VOICE, 0, COS, 0, 560, 0,   25, 70, 100, { 240, 300, 500 }, { OO, OO, OO, OH }, { 0, 0.12f } } };
// A toy winding down: a wobble that slows and sags.
static const Part LOW_BATT[]   = { { TONE, TRI, COS, 0, 650, 0, 5, 60, 100, { 520, 110 }, {}, { 0.12f, 9, 500 } } };
// Two glass chimes, overlapping.
static const Part MESSAGE[]    = {
    { BELL, 0, 0,    0,   450, 0, 2, 0, 100, { 1568 }, {}, { 3.5f, 1, 260 } },
    { BELL, 0, WITH, 110, 520, 0, 2, 0, 100, { 2093 }, {}, { 3.5f, 1, 320 } } };
// Two wooden knocks on the cave door.
static const Part VISITOR[]    = {
    { AIR,  0,   0,    0, 35, 0,   1, 10, 120, { 950 }, {}, { 2.5f } },
    { TONE, SIN, WITH, 0, 50, 110, 1, 10, 70,  { 230, 150 } },
    { AIR,  0,   0,    0, 35, 0,   1, 10, 120, { 900 }, {}, { 2.5f } },
    { TONE, SIN, 0,    0, 55, 0,   1, 10, 70,  { 220, 140 } } };
// A doorstop spring: one flick, and it wobbles down.
static const Part CLIPPY[]     = { { TONE, SIN, 0, 0, 380, 0, 2, 15, 100, { 420 }, {}, { 0.35f, 17, 150 } } };
// A round slide with a short echo, up for the cable and down without it.
static const Part PLUG_IN[]    = { { TONE, TRI, COS, 0, 260, 0, 10, 50, 100, { 220, 880 } } };
static const Part UNPLUG[]     = { { TONE, TRI, COS, 0, 300, 0, 10, 50, 100, { 880, 220 } } };
// T0@$TY lands: a tin box on the counter, and the lever rattles.
static const Part LAND_TOASTER[] = {
    { AIR,  0,   0,    0,  25,  0, 1, 10, 80,  { 2400 }, {}, { 3 } },
    { BELL, 0,   WITH, 0,  360, 0, 2, 0,  100, { 620 },  {}, { 2.76f, 2.5f, 180 } },
    { TONE, SIN, 0,    40, 220, 0, 2, 20, 35,  { 300 },  {}, { 0.25f, 22, 120 } } };
// The yeti lands: big and grumpy, "OOF." out loud.
static const Part LAND_YETI[]  = {
    { TONE,  SIN, 0,          0, 100, 0, 1, 15, 90,  { 100, 36 } },
    { VOICE, 0,   WITH | COS, 0, 300, 0, 8, 50, 100, { 110, 98, 80 }, { OH, OO }, { 0.85f, 0.25f } } };

#define SND(a) a, (uint8_t)(sizeof(a) / sizeof(a[0]))
static const Sound SOUNDS[(int)Id::COUNT] = {
    { SND(NEW_DEVICE) }, { SND(WATCH_HIT), 190, 0.38f, 0.45f, 0.75f }, { SND(LOW_BATT) }, { SND(MESSAGE) },
    { SND(VISITOR), 70, 0.2f, 0.25f }, { SND(THROW) }, { SND(LAND) }, { SND(CLIPPY) }, { SND(TOAST_POP) },
    { SND(PLUG_IN), 90, 0.3f, 0.35f }, { SND(UNPLUG), 90, 0.3f, 0.35f },
    { SND(LAND_TOASTER) }, { SND(LAND_YETI) }, { SND(HEADS_UP), 190, 0.38f, 0.45f, 0.75f },
};
#undef SND

// ---- the codec -------------------------------------------------------------
static bool wr(uint8_t r, uint8_t v) {
    Wire.beginTransmission(CODEC); Wire.write(r); Wire.write(v);
    return Wire.endTransmission() == 0;
}
static uint8_t rd(uint8_t r) {
    Wire.beginTransmission(CODEC); Wire.write(r);
    if (Wire.endTransmission(false) != 0) return 0;
    if (Wire.requestFrom(CODEC, (uint8_t)1) != 1) return 0;
    return (uint8_t)Wire.read();
}
// Espressif's es8311 driver boiled down to one case: slave on the bus, MCLK
// from its pin at 256 x 16 kHz, 16-bit I2S in, the DAC on, the ADC left
// asleep. The divider registers (02-08) only matter to the codec as a bus
// master, but they are set to the table's 4.096 MHz / 16 kHz row anyway, as
// the driver does. Volume register 32 sits at the driver's 85%.
static bool codecInit() {
    if (!wr(0x00, 0x1F)) return false;      // reset everything
    delay(20);
    wr(0x00, 0x00);
    wr(0x00, 0x80);                         // power on
    wr(0x01, 0x3F);                         // MCLK from the MCLK pin, every clock on
    wr(0x02, (uint8_t)(rd(0x02) & 0x07));   // pre-divider 1, multiplier 1x
    wr(0x03, 0x10);                         // single speed, ADC OSR
    wr(0x04, 0x10);                         // DAC OSR
    wr(0x05, 0x00);                         // ADC and DAC dividers 1
    wr(0x06, (uint8_t)((rd(0x06) & 0xC0) | 0x03));   // BCLK not inverted, divider 4
    wr(0x07, (uint8_t)(rd(0x07) & 0xC0));   // LRCK divider high
    wr(0x08, 0xFF);                         // LRCK divider low
    wr(0x00, (uint8_t)(rd(0x00) & 0xBF));   // serial port slave
    wr(0x09, 0x0C);                         // SDP in: 16-bit I2S
    wr(0x0A, 0x0C);                         // SDP out: 16-bit I2S
    wr(0x0D, 0x01);                         // analog circuitry up
    wr(0x0E, 0x02);                         // PGA and ADC modulator
    wr(0x12, 0x00);                         // DAC up
    wr(0x13, 0x10);                         // output to the HP drive (the amp's input)
    wr(0x1C, 0x6A);                         // ADC equalizer bypass
    wr(0x37, 0x08);                         // DAC equalizer bypass
    wr(0x32, 0xD8);                         // DAC volume, 85%
    return true;
}

// ---- the amplifier ---------------------------------------------------------
static void amp(bool on) {
#if defined(CYD)
    // DAC channel 2 is GPIO26, the amp input. Channel 1 is GPIO25, the touch
    // clock: i2s_set_dac_mode() enables it too, so it is never called.
    if (on) { dac_i2s_enable(); dac_output_enable(DAC_CHANNEL_2); }
    else    { dac_output_disable(DAC_CHANNEL_2); dac_i2s_disable(); }
#else
    if (PIN_AMP >= 0) digitalWrite(PIN_AMP, (on == s_ampLow) ? LOW : HIGH);
#endif
}

// ---- rendering -------------------------------------------------------------
// The whole sound is rendered into a float buffer in PSRAM first (every board
// with a speaker has it; the longest sound, the whoop with its echo, is about
// 150 KB), parts added on top of each other, then the echo and a soft limiter,
// then it streams out 256 frames at a time, the same sample in both slots
// (the codec takes the left). Rendering the longest one takes a few ms.
static const float TAU_F = 6.2831853f;

// Mouth shapes: the three loudest resonances of a throat, in Hz, and their widths.
static const uint16_t VOWEL[][3] = {
    { 0, 0, 0 }, { 300, 870, 2240 }, { 570, 840, 2410 }, { 730, 1090, 2440 }, { 640, 1190, 2390 },
    { 530, 1840, 2480 }, { 270, 2290, 3010 }, { 390, 1990, 2550 }, { 260, 1050, 2200 } };
static const float BANDW[3] = { 80, 110, 160 };
static const float FORM[3]  = { 1.0f, 0.55f, 0.28f };

// A seeded noise source, so a sound comes out the same every time: -1..1.
struct Rng { uint32_t s; float next() { s = s * 1664525u + 1013904223u; return (float)(s >> 8) / 8388608.0f - 1.0f; } };

// A band-pass with a peak of exactly 1 (the Audio EQ Cookbook's), for the
// mouth and for filtered air. c = b0, b2, a1, a2; z = x1, x2, y1, y2.
static void bpSet(float* c, float f, float bw) {
    if (f > RATE * 0.45f) f = RATE * 0.45f;
    const float w = TAU_F * f / (float)RATE, q = fmaxf(0.5f, f / bw), al = sinf(w) / (2 * q), a0 = 1 + al;
    c[0] = al / a0; c[1] = -al / a0; c[2] = -2 * cosf(w) / a0; c[3] = (1 - al) / a0;
}
static float bpRun(const float* c, float* z, float x) {
    const float y = c[0] * x + c[1] * z[1] - c[2] * z[2] - c[3] * z[3];
    z[1] = z[0]; z[0] = x; z[3] = z[2]; z[2] = y;
    return y;
}

// How many points a list holds: pitches end at the first 0, vowels at V_END.
static int pts(const uint16_t* p) { int n = 0; while (n < 4 && p[n]) n++; return n ? n : 1; }
static int pts(const uint8_t* v)  { int n = 0; while (n < 4 && v[n]) n++; return n ? n : 1; }
// Glide along the points, u = 0..1 through the part, in equal musical steps.
static float along(const uint16_t* p, float u) {
    const int n = pts(p);
    if (n == 1) return (float)p[0];
    const float x = u * (n - 1);
    const int i = x >= n - 1 ? n - 2 : (int)x;
    return (float)p[i] * powf((float)p[i + 1] / (float)p[i], x - i);
}

static int attackFrames(const Part& st) { const int a = st.a * (int)RATE / 1000; return a > 0 ? a : 1; }
// Attack, then level until h percent of the way, then down to nothing.
static float envLin(const Part& st, int n, int N) {
    const int a = attackFrames(st), h = N * st.h / 100;
    if (n < a) return (float)n / a;
    if (n < h) return 1.0f;
    const float r = (float)(N - n) / (float)(N - h);
    return (st.flags & COS) ? 0.5f - 0.5f * cosf(3.1415927f * r) : r;
}
// Attack, then a ring that dies away, with a short fade at the very end.
static float envRing(const Part& st, int n, int N, float ringS) {
    const int a = attackFrames(st), tail = RATE * 12 / 1000;
    if (n < a) return (float)n / a;
    float e = expf(-(float)(n - a) / (float)RATE / ringS);
    if (n > N - tail) e *= (float)(N - n) / tail;
    return e;
}

// Carried across chunks so a sound can be rendered as it plays. A full-buffer
// render starts these at zero (and the shared noise seed) and runs n = 0..N.
struct ToneRun { float ph; uint32_t lcg; float lp; };
struct VoiceRun { float ph, prev, jit; Rng r; float c[3][4]; float z[3][4]; };
struct AirRun { Rng r; float c[4]; float z[4]; };

static void renderTone(const Part& st, float* out, int n0, int count, int N, ToneRun& run) {
    const float lv = st.lv / 100.0f;
    const int end = n0 + count;
    for (int n = n0; n < end; n++) {
        const float t = (float)n / RATE;
        float f = along(st.p, (float)n / N);
        if (st.k[0] > 0) f *= 1 + st.k[0] * sinf(TAU_F * st.k[1] * t) * (st.k[2] > 0 ? expf(-t * 1000 / st.k[2]) : 1);
        float v;
        if (st.kind == NOISE) {
            run.lcg = run.lcg * 1664525u + 1013904223u;
            run.lp += 0.25f * (((int32_t)(run.lcg >> 8) / 8388608.0f) - 1.0f - run.lp);   // one-pole low-pass
            v = run.lp * 0.9f;
        } else switch (st.wave) {
            case SQ:  v = run.ph < 0.5f ? 0.35f : -0.35f; break;
            case TRI: v = (run.ph < 0.5f ? (run.ph * 4.0f - 1.0f) : (3.0f - run.ph * 4.0f)) * 0.7f; break;
            case SIN: v = sinf(run.ph * TAU_F) * 0.7f; break;
            default:  v = (run.ph * 2.0f - 1.0f) * 0.4f; break;
        }
        out[n - n0] += v * envLin(st, n, N) * lv;
        run.ph += f / RATE;
        if (run.ph >= 1.0f) run.ph -= 1.0f;
    }
}

// A throat: a Rosenberg glottal pulse at the pitch, its slope (what the mouth
// hears) through three resonances that glide between the vowels, plus breath.
static void renderVoice(const Part& st, float* out, int n0, int count, int N, VoiceRun& run) {
    const float sz = st.k[0] > 0 ? st.k[0] : 1.0f, br = st.k[1], vd = st.k[3], lv = st.lv / 100.0f;
    const int h0 = (int)(st.k[2] * RATE / 1000);
    const int nv = pts(st.v);
    const int end = n0 + count;
    for (int n = n0; n < end; n++) {
        const float u = (float)n / N, t = (float)n / RATE;
        run.jit += (run.r.next() * 0.5f - run.jit) * 0.02f;
        const float f0 = along(st.p, u) * (1 + 0.012f * sinf(TAU_F * 5.2f * t) + 0.01f * run.jit + vd * sinf(TAU_F * 6 * t));
        run.ph += f0 / RATE;
        if (run.ph >= 1.0f) run.ph -= 1.0f;
        const float pu = run.ph < 0.4f ? 0.5f - 0.5f * cosf(3.1415927f * run.ph / 0.4f)
                       : run.ph < 0.56f ? cosf(1.5707963f * (run.ph - 0.4f) / 0.16f) : 0.0f;
        float e = (pu - run.prev) * RATE / (f0 * 7.85f);
        run.prev = pu;
        const float hs = n < h0 ? 1.0f - (float)n / h0 : 0.0f;    // the h of "hey"
        e = e * (1 - hs * 0.85f) + run.r.next() * (br * 0.35f + hs * 0.9f);
        if ((n & 15) == 0) {
            const float x = u * (nv - 1);
            const int i = nv < 2 ? 0 : (x >= nv - 1 ? nv - 2 : (int)x);
            const float fr = x - i;
            const uint16_t* A = VOWEL[st.v[i]];
            const uint16_t* B = VOWEL[st.v[nv < 2 ? i : i + 1]];
            for (int k = 0; k < 3; k++) bpSet(run.c[k], (A[k] + (B[k] - A[k]) * fr) * sz, BANDW[k]);
        }
        float y = 0;
        for (int k = 0; k < 3; k++) y += bpRun(run.c[k], run.z[k], e) * FORM[k];
        out[n - n0] += y * 1.7f * envLin(st, n, N) * lv;
    }
}

static void renderBell(const Part& st, float* out, int n0, int count, int N) {
    const float fc = st.p[0], fm = fc * st.k[0], I = st.k[1], ring = st.k[2] / 1000.0f;
    const float ie = ring * (st.k[3] > 0 ? st.k[3] : 0.5f), lv = st.lv / 100.0f;
    const int end = n0 + count;
    for (int n = n0; n < end; n++) {
        const float t = (float)n / RATE;
        const float v = sinf(TAU_F * fc * t + I * expf(-t / ie) * sinf(TAU_F * fm * t));
        out[n - n0] += v * 0.55f * envRing(st, n, N, ring) * lv;
    }
}

static void renderAir(const Part& st, float* out, int n0, int count, int N, AirRun& run) {
    const float q = st.k[0] > 0 ? st.k[0] : 1.0f, lv = st.lv / 100.0f;
    const int end = n0 + count;
    for (int n = n0; n < end; n++) {
        if ((n & 15) == 0) { const float fc = along(st.p, (float)n / N); bpSet(run.c, fc, fc / q); }
        out[n - n0] += bpRun(run.c, run.z, run.r.next()) * 1.4f * envLin(st, n, N) * lv;
    }
}

static void writeFrames(int16_t* buf, int frames) {
    size_t done = 0;
    i2s_write(PORT, buf, (size_t)frames * 4, &done, portMAX_DELAY);
}

static void silence(int16_t* buf, int ms) {
    memset(buf, 0, CHUNK * 4);
    for (int left = ms * (int)RATE / 1000; left > 0; left -= CHUNK)
        writeFrames(buf, left < CHUNK ? left : CHUNK);
}

static void* bigAlloc(size_t bytes) {
    void* p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    return p ? p : malloc(bytes);
}

#if defined(CYD)
// One part's place in the mix, and the oscillator state that has to survive
// from one 16 ms chunk to the next. Held only while a sound plays: a mounted
// card leaves one block of about 10.7 KB, wardriving pauses under 8 KB, and
// these buffers plus a task stack are most of that gap.
struct PartRun {
    int at, N;
    bool live;
    ToneRun tone;
    VoiceRun voice;
    AirRun air;
};
struct DacWs {
    PartRun  run[8];
    float    chunk[CHUNK];
    uint16_t dac[CHUNK * 2];
};

static void addPart(const Part& st, PartRun& run, float* chunk, int base, int n) {
    const int from = base > run.at ? base : run.at;
    const int to = base + n < run.at + run.N ? base + n : run.at + run.N;
    if (from >= to) return;
    float* out = chunk + (from - base);
    const int n0 = from - run.at, count = to - from;
    switch (st.kind) {
        case VOICE: renderVoice(st, out, n0, count, run.N, run.voice); break;
        case BELL:  renderBell(st, out, n0, count, run.N); break;
        case AIR:   renderAir(st, out, n0, count, run.N, run.air); break;
        default:    renderTone(st, out, n0, count, run.N, run.tone); break;
    }
}

// Stereo frames, the same sample in both slots. Only the left slot reaches
// the amp (DAC2, GPIO26). Midscale is silence: the DAC reads the high byte.
static void writeDac(uint16_t* dac, int frames) {
    size_t done = 0;
    i2s_write(PORT, dac, (size_t)frames * 4, &done, portMAX_DELAY);
}
static void dacSilence(uint16_t* dac, int ms) {
    for (int i = 0; i < CHUNK * 2; i++) dac[i] = 0x8000;
    for (int left = ms * (int)RATE / 1000; left > 0; left -= CHUNK)
        writeDac(dac, left < CHUNK ? left : CHUNK);
}

// Installed for one sound and removed after it. Left up, its four 1 KB DMA
// buffers come out of the 10.7 KB block a mounted card leaves, and wardriving
// then sits on LOW MEMORY: it pauses once the largest block falls under 8 KB.
static bool dacInstall() {
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN);
    cfg.sample_rate          = RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_MSB;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 4;
    cfg.dma_buf_len          = CHUNK;
    cfg.use_apll             = false;
    cfg.tx_desc_auto_clear   = true;
    cfg.fixed_mclk           = 0;
    cfg.mclk_multiple        = I2S_MCLK_MULTIPLE_256;
    cfg.bits_per_chan        = I2S_BITS_PER_CHAN_16BIT;
    return i2s_driver_install(PORT, &cfg, 0, nullptr) == ESP_OK;
}

static void playNowDac(const Sound& snd, const int* start, uint8_t parts, int total) {
    DacWs* ws = (DacWs*)malloc(sizeof(DacWs));
    if (!ws) { Serial.println("[sound] no memory for the sound"); return; }
    if (!dacInstall()) {
        Serial.println("[sound] I2S DAC refused");
        free(ws);
        return;
    }
    for (uint8_t i = 0; i < parts; i++) {
        const Part& st = snd.p[i];
        PartRun& r = ws->run[i];
        r.at = start[i] * (int)RATE / 1000;
        r.N = st.ms * (int)RATE / 1000;
        r.live = r.at >= 0 && r.N > 0 && r.at + r.N <= total;
        r.tone = ToneRun{ 0, 0x5EED1234u, 0 };
        r.voice = VoiceRun{};
        r.voice.r.s = 777;
        r.air = AirRun{};
        r.air.r.s = 4242;
    }
    const int D = snd.echoMs ? snd.echoMs * (int)RATE / 1000 : 0;
    float* echo = D ? (float*)malloc((size_t)D * sizeof(float)) : nullptr;
    if (echo) memset(echo, 0, (size_t)D * sizeof(float));
    int ei = 0;

    const float pct = (float)Settings::soundVolume() / 100.0f;
    const float vol = pct * pct * 32000.0f, g = snd.gain > 0 ? snd.gain : 1.0f;

    i2s_start(PORT);
    dacSilence(ws->dac, 24);  // clocks and midscale before the pad connects: no pop
    amp(true);
    dacSilence(ws->dac, 16);
    for (int base = 0; base < total; ) {
        const int n = (total - base) < CHUNK ? (total - base) : CHUNK;
        memset(ws->chunk, 0, (size_t)n * sizeof(float));
        for (uint8_t i = 0; i < parts; i++) if (ws->run[i].live) addPart(snd.p[i], ws->run[i], ws->chunk, base, n);
        for (int i = 0; i < n; i++) {
            float sample = ws->chunk[i];
            if (echo) {
                const float z = echo[ei];
                echo[ei] = sample + z * snd.fb;
                sample += z * snd.mix;
                if (++ei == D) ei = 0;
            }
            const float v = sample * g, a = fabsf(v);
            const float o = a < 0.7f ? v : copysignf(0.7f + 0.3f * tanhf((a - 0.7f) / 0.3f), v);
            int s = (int)(o * vol);
            if (s > 32767) s = 32767;
            if (s < -32768) s = -32768;
            const uint16_t u = (uint16_t)(s + 32768);
            ws->dac[i * 2] = u; ws->dac[i * 2 + 1] = u;
        }
        writeDac(ws->dac, n);
        base += n;
    }
    free(echo);
    dacSilence(ws->dac, 16);
    amp(false);
    dacSilence(ws->dac, 24);
    i2s_zero_dma_buffer(PORT);
    i2s_stop(PORT);
    i2s_driver_uninstall(PORT);
    free(ws);
}
#endif

static void playNow(Id id) {
    const Sound& snd = SOUNDS[(int)id];
    // Lay the parts out in time, the way the Lab did.
    const uint8_t parts = snd.n < 8 ? snd.n : 8;
    int start[8], t = 0, last = 0;
    for (uint8_t i = 0; i < parts; i++) {
        const Part& st = snd.p[i];
        start[i] = ((st.flags & WITH) ? last : t) + st.off;
        last = start[i];
        if (start[i] + st.ms + st.gap > t) t = start[i] + st.ms + st.gap;
    }
    const int total = (t + snd.echoMs * 6 + 30) * (int)RATE / 1000;
#if defined(CYD)
    playNowDac(snd, start, parts, total);
    return;
#endif
    float* mix = (float*)bigAlloc((size_t)total * sizeof(float));
    if (!mix) { Serial.println("[sound] no memory for the sound"); return; }
    memset(mix, 0, (size_t)total * sizeof(float));

    uint32_t lcg = 0x5EED1234u;
    float lp = 0;
    for (uint8_t i = 0; i < parts; i++) {
        const Part& st = snd.p[i];
        const int at = start[i] * (int)RATE / 1000, N = st.ms * (int)RATE / 1000;
        if (at < 0 || at + N > total) continue;
        switch (st.kind) {
            case VOICE: {
                VoiceRun vr{};
                vr.r.s = 777;
                renderVoice(st, mix + at, 0, N, N, vr);
                break;
            }
            case BELL:  renderBell(st, mix + at, 0, N, N);  break;
            case AIR: {
                AirRun ar{};
                ar.r.s = 4242;
                renderAir(st, mix + at, 0, N, N, ar);
                break;
            }
            default: {
                ToneRun tr{ 0, lcg, lp };
                renderTone(st, mix + at, 0, N, N, tr);
                lcg = tr.lcg;
                lp = tr.lp;
                break;
            }
        }
    }
    if (snd.echoMs) {
        const int D = snd.echoMs * (int)RATE / 1000;
        float* line = (float*)bigAlloc((size_t)D * sizeof(float));
        if (line) {
            memset(line, 0, (size_t)D * sizeof(float));
            for (int n = 0, i = 0; n < total; n++) {
                const float z = line[i];
                line[i] = mix[n] + z * snd.fb;
                mix[n] += z * snd.mix;
                if (++i == D) i = 0;
            }
            free(line);
        }
    }

    // The percent squared: the ear hears loudness on a curve, and squaring
    // the slider gives the bottom half of it real steps instead of a jump.
    const float pct = (float)Settings::soundVolume() / 100.0f;
    const float vol = pct * pct * 32000.0f, g = snd.gain > 0 ? snd.gain : 1.0f;
    int16_t buf[CHUNK * 2];

    i2s_start(PORT);
    silence(buf, 24);         // clocks and zeros before the amp wakes: no pop
    amp(true);
    silence(buf, 16);
    for (int at = 0; at < total; ) {
        const int n = (total - at) < CHUNK ? (total - at) : CHUNK;
        for (int i = 0; i < n; i++, at++) {
            // A soft limiter: untouched under 0.7, then rounding off toward 1.
            const float v = mix[at] * g, a = fabsf(v);
            const float o = a < 0.7f ? v : copysignf(0.7f + 0.3f * tanhf((a - 0.7f) / 0.3f), v);
            const int16_t s = (int16_t)(o * vol);
            buf[i * 2] = s; buf[i * 2 + 1] = s;
        }
        writeFrames(buf, n);
    }
    free(mix);

    silence(buf, 16);
    amp(false);
    silence(buf, 24);         // let the DMA drain before the clocks stop
    i2s_zero_dma_buffer(PORT);
    i2s_stop(PORT);
}

static void soundTask(void*) {
    uint8_t id;
    for (;;) {
#if defined(CYD)
        // Idle a tenth of a second, then leave. The stack is the block
        // wardriving is measured against, so it cannot sit there between sounds.
        if (xQueueReceive(s_q, &id, pdMS_TO_TICKS(100)) != pdTRUE) {
            bool leave = false;
            portENTER_CRITICAL(&s_mux);
            if (s_task == xTaskGetCurrentTaskHandle()) {
                s_task = nullptr;
                leave = true;
            }
            portEXIT_CRITICAL(&s_mux);
            if (leave) vTaskDelete(nullptr);
            continue;
        }
#else
        if (xQueueReceive(s_q, &id, portMAX_DELAY) != pdTRUE) continue;
#endif
        if (id < (uint8_t)Id::COUNT) playNow((Id)id);
    }
}

#if defined(CYD)
// The queue and the task exist only around a sound. Claimed with a stand-in
// handle so a second play during the create does not start a second task.
static void ensurePlayer() {
    portENTER_CRITICAL(&s_mux);
    const bool start = s_task == nullptr;
    if (start) s_task = (TaskHandle_t)1;
    portEXIT_CRITICAL(&s_mux);
    if (!start) return;
    if (!s_q) s_q = xQueueCreate(4, sizeof(uint8_t));
    TaskHandle_t created = nullptr;
    const BaseType_t ok = s_q
        ? xTaskCreatePinnedToCore(soundTask, "sound", 4096, nullptr, 2, &created, 1)
        : pdFAIL;
    portENTER_CRITICAL(&s_mux);
    s_task = (ok == pdPASS) ? created : nullptr;
    portEXIT_CRITICAL(&s_mux);
    if (ok != pdPASS) Serial.println("[sound] no memory for the player");
}
#endif

// ---- the API ---------------------------------------------------------------
void begin() {
#if defined(CYD)
    // Built-in DAC, not a codec. Do not call i2s_set_pin(NULL) or
    // i2s_set_dac_mode(): both turn on DAC1, and that pad is GPIO25, the
    // touch clock. The amp is DAC2, GPIO26. Nothing is allocated here: the
    // driver, the mix buffer and the player task are created for one sound
    // and freed after it, because a mounted card leaves one block of about
    // 10.7 KB and wardriving pauses once that block falls under 8 KB.
    s_ok = true;
    Serial.println("[sound] CYD amp on DAC2 GPIO26, brought up only while a sound plays");
#else
    if (PIN_AMP >= 0) pinMode(PIN_AMP, OUTPUT);
    amp(false);
#if !SOUND_NO_CODEC
    Wire.beginTransmission(CODEC);
    if (Wire.endTransmission() != 0) { Serial.println("[sound] ES8311 did not answer; no speaker"); return; }
#endif

    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate          = RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 4;
    cfg.dma_buf_len          = CHUNK;
    cfg.use_apll             = false;
    cfg.tx_desc_auto_clear   = true;
    cfg.fixed_mclk           = 0;
    cfg.mclk_multiple        = I2S_MCLK_MULTIPLE_256;
    cfg.bits_per_chan        = I2S_BITS_PER_CHAN_16BIT;
    if (i2s_driver_install(PORT, &cfg, 0, nullptr) != ESP_OK) { Serial.println("[sound] I2S driver refused"); return; }
    i2s_pin_config_t pins = {};
    pins.mck_io_num   = PIN_MCK >= 0 ? PIN_MCK : I2S_PIN_NO_CHANGE;
    pins.bck_io_num   = PIN_BCK;
    pins.ws_io_num    = PIN_WS;
    pins.data_out_num = PIN_DOUT;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;
    i2s_set_pin(PORT, &pins);
    i2s_zero_dma_buffer(PORT);
    // The codec's power-up wants its clocks running; the driver starts them
    // on install. Set the codec up, then park the clocks until a sound.
#if SOUND_NO_CODEC
    i2s_stop(PORT);
    s_q = xQueueCreate(4, sizeof(uint8_t));
    xTaskCreatePinnedToCore(soundTask, "sound", 4096, nullptr, 2, &s_task, 1);
    s_ok = s_q && s_task;
    Serial.printf("[sound] I2S amp ready: BCK %d WS %d DOUT %d%s\n", PIN_BCK, PIN_WS, PIN_DOUT, s_ok ? "" : " -- task failed");
#else
    const bool ok = codecInit();
    const uint8_t id1 = rd(0xFD), id2 = rd(0xFE);
    i2s_stop(PORT);
    if (!ok) { Serial.println("[sound] ES8311 setup failed"); return; }
    s_q = xQueueCreate(4, sizeof(uint8_t));
    xTaskCreatePinnedToCore(soundTask, "sound", 4096, nullptr, 2, &s_task, 1);
    s_ok = s_q && s_task;
    Serial.printf("[sound] ES8311 id %02X%02X ready: MCK %d BCK %d WS %d DOUT %d, amp on GPIO%d%s\n",
                  id1, id2, PIN_MCK, PIN_BCK, PIN_WS, PIN_DOUT, PIN_AMP, s_ok ? "" : " -- task failed");
#endif
#endif
}

void play(Id id, bool sample) {
    if (!s_ok || id >= Id::COUNT) return;
    if (!sample) {
        const uint8_t mode = Settings::soundMode();          // 0 OFF, 1 ALERTS, 2 ALL
        const bool alert = id == Id::NEW_DEVICE || id == Id::WATCH_HIT || id == Id::LOW_BATTERY
                        || id == Id::HEADS_UP;
        if (mode == 0 || (!alert && mode < 2)) return;
        if (s_hush) return;
        if (Clock::night() && id != Id::WATCH_HIT) return;
    }
    const uint8_t b = (uint8_t)id;
#if defined(CYD)
    ensurePlayer();
    if (!s_task || !s_q) return;
#endif
    xQueueSend(s_q, &b, 0);        // a full queue drops the sound, not the frame
}

void hush(bool on) { s_hush = on; }

void quiet() {
    if (!s_ok) return;
    if (s_q) xQueueReset(s_q);
    amp(false);
}

bool present() { return s_ok; }

void ampLow(bool activeLow) { s_ampLow = activeLow; amp(false); }

}  // namespace Sound
#endif  // SQW_SPEAKER
