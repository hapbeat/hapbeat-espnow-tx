// ---------------------------------------------------------------------------
// audio_source.cpp — ESP-NOW live audio source (see header). #ifdef AUDIO_SOURCE.
//
// Captures PA line-in via an I2S codec (M5 Module-Audio / ES8388), decimates
// 48 kHz → 16 kHz, IMA-ADPCM-encodes, and broadcasts 0xAA stream packets
// over ESP-NOW. See contracts/specs/espnow-stream.md.
//
// Shortest-path design (mirrors the reference wireless-sender-firmware
// audioStreamSender, which is the latency-optimized origin):
//   * Capture runs in a dedicated, core-pinned FreeRTOS task (not the Arduino
//     loop) so scheduling jitter doesn't perturb timing.
//   * Raw driver/i2s.h read in tiny 64-frame chunks (low read-granularity
//     latency) but backed by a deep DMA ring (16 x 32 frames ~= 10.67 ms) so a
//     multi-ms Opus encode stall can't overflow RX DMA and drop samples.
//   * Capture at 48 kHz and software-decimate 3:1 to 16 kHz — the ES8388's
//     higher-rate decimation filter has lower group delay than at 16 kHz.
//   * Line input = LINPUT1/RINPUT1 (the TRS line-in jack). LINPUT2 is the mic
//     path and leaves a channel silent for line use.
//   * ESP-NOW radio is configured for 6 Mbps in espnow_sender.cpp so the
//     1000 pkt/s stream fits with a shallow in-flight depth (low latency).
//
// On classic M5Stack Core/Basic the same path uses a manual ES8388 register
// init; on CoreS3 (-D BOARD_CORES3) the official M5Unified + M5Module-Audio
// driver inits the codec only (board-correct I2C, STM32 input enable), while
// the I2S is set up here so we control the DMA buffer size. CoreS3 keeps the
// module's A/B switch on "A" (classic wiring) — see audioSourceSetup().
//
// Robustness (DEC-033): piggyback (§3.2) + bounded in-flight; encoder state
// always advances even on a dropped packet so the next packet stays valid.
// ---------------------------------------------------------------------------

#include "audio_source.h"

#ifdef AUDIO_SOURCE

#include "config.h"        // BROADCAST_MAC
#include "display.h"
#include "ima_adpcm.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <driver/i2s.h>
#include <esp_timer.h>            // esp_timer_get_time() for encode-time telemetry
#include <esp_system.h>           // esp_reset_reason() + esp_get_free_heap_size()
#include <freertos/queue.h>       // I2S event queue (RX overflow telemetry)
#include <cstring>

#ifdef BOARD_CORES3
#include <M5Unified.h>
#include "M5Module_Audio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "opus.h"            // Opus encode (DEC-042) — CoreS3 only
#include <esp_heap_caps.h>   // MALLOC_CAP_INTERNAL for the encoder state
#else
#include <Wire.h>
#endif

// ---- audio params (must match the receiver) --------------------------------
static const uint32_t CAPTURE_RATE       = 48000;
static const uint32_t STREAM_RATE        = 16000;
static const int      DECIMATE           = CAPTURE_RATE / STREAM_RATE;  // 3
// Frames per packet. The audible glitch every 100-300 ms was 2.4 GHz
// interference on channel 1 (beacon-correlated bursts), NOT receiver-queue
// overflow. Fix = move both nodes to channel 11 (set_espnow_channel).
// On channel 11, LARGER packets recover better: interference bursts are
// time-based (a few ms), so a 64-frame (4 ms) packet captures a burst as ONE
// lost packet that the depth-1 piggyback recovers (measured 95% recovery,
// 0.17% net, worst gap 8 ms). At 16 frames (1 ms) the same burst spans many
// packets (measured maxgap 18 → 18 ms, only 53% recovered). So keep 64 (the
// receiver's MAX_FRAMES). Residual = the rare 2-packet burst; deepen the
// piggyback to depth 2 if it needs to be tighter. (Receiver reads num_frames.)
static const int      FRAMES_PKT         = 64;
static const int      CAP_FRAMES         = FRAMES_PKT * DECIMATE;       // 48
static const uint8_t  STREAM_TYPE        = 0xAA;

// ---- Streaming modes (mode_id = 0xAA packet header byte [1]) ----------------
// The sender picks the mode; the receiver reads mode_id and auto-adapts its
// decode + jitter-buffer depth. Table MUST match device-firmware
// espnow_stream.cpp RX_MODE. See docs/instructions-espnow-modes-button-ui-*.
//   0 SOLID    ADPCM 16k stereo 64-frame(4ms)  pb1  compat/floor, high-reliability (classic+CoreS3)
//   1 FAST     OPUS   8k mono    40-smp(5ms)    pb3  low-latency
//   2 BALANCED OPUS   8k mono    40-smp(5ms)    pb3  middle (deeper RX buffer)
//   3 SMOOTH   OPUS   8k mono    80-smp(10ms)   pb2  robust (deepest + 2× pb)
//   4 STEREO   OPUS   8k stereo  80-smp(10ms)   pb2  dual-channel haptics, 8 kHz
//   5 HIFI     OPUS  16k stereo 160-smp(10ms)   pb2  dual-channel, full 16 kHz band
//   6 LITE     ADPCM  8k mono    40-smp(5ms)    pb3  zero-encode mono (CoreS3 8k path)
//   7 TURBO    ADPCM  8k mono    40-smp(5ms)    pb3  same wire as LITE — 4ms RX buffer (min latency, drops-tolerant)
//   8 FINE     OPUS  16k mono   160-smp(10ms)   pb2  mono full-band (16 kHz mono middle ground between SMOOTH and HIFI)
//   9 SOLID48  OPUS  48k stereo 480-smp(10ms)   pb2  highest quality — full-band stereo, single-source use
// Per-mode worst-case packet size vs the 250 B ESP-NOW limit (ESP_NOW_MAX_DATA_LEN):
//   Opus body = type1+mode1+seq1+pbcnt1 + len1 + flen ; each pb entry = seq1+len1+flen.
//     FAST/BAL  flen=30, pb3: 4 + 1 + 30 + 3*(2+30) = 131 B  <=250 ✓
//     SMOOTH    flen=75, pb2: 4 + 1 + 75 + 2*(2+75) = 234 B  <=250 ✓
//     STEREO    flen=80, pb2: 4 + 1 + 80 + 2*(2+80) = 249 B  <=250 ✓ (at the limit)
//     HIFI      flen=75, pb2: 4 + 1 + 75 + 2*(2+75) = 234 B  <=250 ✓
//     FINE      flen=60, pb2: 4 + 1 + 60 + 2*(2+60) = 189 B  <=250 ✓ (16k mono @48kbps CBR)
//     SOLID48   flen=80, pb2: 4 + 1 + 80 + 2*(2+80) = 249 B  <=250 ✓ (at the limit; 48k stereo @64kbps CBR)
//   ADPCM body = type1+mode1+seq1+pbcnt1 + nf1 + state + data ; each pb entry = seq1+state+data.
//     SOLID (mode0) 64-frame stereo: state6 + data64, pb1 (unchanged, ~148 B).
//     LITE (mode6)/TURBO (mode7) 8k mono: state3 + data20, pb3: 4 + 1 + 3 + 20 + 3*(1+3+20) = 100 B  <=250 ✓
// Opus modes are CoreS3-only (classic sender has no encoder → ADPCM only). Mode 6
// (LITE / ADPCM 8k mono) and mode 7 (TURBO, same wire as LITE) are also CoreS3-only
// — their decimate+encode path lives in audioRxTask; the classic loop always ships
// mode 0 (16 kHz stereo ADPCM). Mode 9 (SOLID48) bypasses the FIR/decimation path
// entirely and streams the raw 48 kHz capture directly into Opus (see audioRxTask).
enum StreamMode : uint8_t {
    MODE_ADPCM = 0, MODE_L = 1, MODE_M = 2, MODE_Q = 3,
    MODE_STEREO = 4, MODE_HIFI = 5, MODE_MONO8K = 6, MODE_TURBO = 7,
    MODE_FINE = 8, MODE_SOLID48 = 9, MODE_COUNT = 10
};
struct ModeDef {
    uint8_t  codec;       // 0 = ADPCM, 1 = OPUS
    uint16_t rate;        // capture/encode rate (Hz): 8000 or 16000
    uint8_t  channels;    // 1 = mono, 2 = stereo
    uint16_t opus_frame;  // samples PER CHANNEL per Opus frame (at `rate`)
    uint8_t  piggyback;   // redundant prior frames per packet
    uint16_t bitrate;     // Opus target bitrate (bps; uint16 → keep ≤65535)
    uint8_t  complexity;  // Opus complexity 0-10 (postfilter/tf gates at ≥2/≥5)
};
static const ModeDef MODE_DEFS[MODE_COUNT] = {
    { 0, 16000, 2,   0, 1,     0, 0 },   // 0 SOLID    ADPCM 16k stereo         pb1 (unchanged, high-reliability)
    { 1,  8000, 1,  40, 3, 48000, 2 },   // 1 FAST     Opus  8k mono    5 ms  pb3 (131 B; c2: low-latency, cheap)
    { 1,  8000, 1,  40, 3, 48000, 5 },   // 2 BALANCED Opus  8k mono    5 ms  pb3 (131 B)
    { 1,  8000, 1,  80, 2, 60000, 5 },   // 3 SMOOTH   Opus  8k mono   10 ms  pb2 (234 B; pb3 would exceed 250)
    { 1,  8000, 2,  80, 2, 64000, 5 },   // 4 STEREO   Opus  8k stereo 10 ms  pb2 (249 B, at the 250 limit)
    { 1, 16000, 2, 160, 2, 60000, 5 },   // 5 HIFI     Opus 16k stereo 10 ms  pb2 (234 B; c5 postfilter — fixes c2 amplitude wobble; verify enc<70% on HW)
    { 0,  8000, 1,  40, 3,     0, 0 },   // 6 LITE     ADPCM 8k mono   5 ms  pb3 (100 B; opus_frame=40 = mono samples/packet; codec 0 → bitrate/complexity unused)
    { 0,  8000, 1,  40, 3,     0, 0 },   // 7 TURBO    ADPCM 8k mono   5 ms  pb3 — identical wire to LITE (mode 6); only the RX jitter buffer differs (4ms vs 16ms)
    { 1, 16000, 1, 160, 2, 48000, 5 },   // 8 FINE     Opus 16k mono   10 ms  pb2 (189 B; mono full-band, between SMOOTH and HIFI)
    { 1, 48000, 2, 480, 2, 64000, 1 },   // 9 SOLID48  Opus 48k stereo 10ms  pb2 (~249B @64k CBR — full-band stereo). complexity=1 is a REAL-TIME-SAFE default: 48k stereo has 3× HIFI's samples/frame, so c5 (~210% of the 10ms encode budget, est.) would starve the I2S RX. Measure the [AUDIO-SRC] enc-max us on HW; if <~7ms with headroom, raise toward 5 for quality. See DEC-046 G-A1.
};
// User-facing names (SOLID=ADPCM high-reliability baseline; FAST/BALANCED/SMOOTH=
// Opus mono latency ladder; FINE=Opus 16k mono full-band; STEREO/HIFI=dual-channel
// Opus at 8/16 kHz; LITE=zero-encode ADPCM mono; TURBO=LITE wire + shallow RX
// buffer for minimum latency).
static const char*   MODE_NAME[MODE_COUNT] = { "SOLID", "FAST", "BALANCED",
                                               "SMOOTH", "STEREO", "HIFI", "LITE", "TURBO",
                                               "FINE", "SOLID48" };
static const char*   MODE_DESC[MODE_COUNT] = { "ADPCM 30ms", "Opus 10ms",
                                               "Opus 15ms", "Opus 28ms",
                                               "Opus 18ms", "Opus 28ms", "ADPCM 8ms",
                                               "ADPCM 4ms buf", "Opus 16k mono",
                                               "Opus 48k stereo" };
// Audio format per mode (rate + channels), shown on the sender HOME screen.
static const char*   MODE_FMT[MODE_COUNT]  = { "16kHz Stereo", "8kHz Mono",
                                               "8kHz Mono", "8kHz Mono",
                                               "8kHz Stereo", "16kHz Stereo",
                                               "8kHz Mono ADPCM",
                                               "8kHz Mono ADPCM",
                                               "16kHz Mono",
                                               "48kHz Stereo" };
static const int      OPUS_MAX_LEN         = 160;    // max opus frame bytes (headroom)
static const int      OPUS_MAX_SMP         = 480;    // max samples/ch/frame (10 ms @48k, SOLID48)
// s_opusAccum is [OPUS_MAX_SMP*2] (per-channel × up to 2 ch). Guard that the
// largest Opus frame in MODE_DEFS (SOLID48 = 480 samples/ch @ 48 kHz) still
// fits — if a future mode raises opus_frame, bump OPUS_MAX_SMP or this fails
// to build instead of silently overflowing the encoder input.
static_assert(OPUS_MAX_SMP >= 480, "OPUS_MAX_SMP must hold the largest MODE_DEFS opus_frame");

// Live-selectable current mode (NVS "tx"/"mode", default 0 = ADPCM known-good).
static volatile uint8_t s_mode = MODE_ADPCM;

// ---- LONGRANGE profile + presets (DEC-043 P5 §7.3 / LR-family-ui) ----------
// RANGE=LONG (NVS tx/range_long) switches the radio to 802.11 LR / LORA_500K and
// pins the stream to the "LR profile": mode_id 3 (SMOOTH wire = Opus 8k mono
// 10 ms). The exact bitrate/pb/buffer come from one of 6 presets (grid =
// quality×stability). The wire stays id3, so receivers/repeaters need NO change:
// bitrate is Opus-self-describing, pb travels in the wire pb_count, and the
// per-preset receiver buffer depth is distributed by a 0xAC param-1 beacon at boot.
static bool          s_range_long = false;
static const uint8_t LR_MODE      = MODE_Q;   // id 3 = SMOOTH wire host

struct LrPreset { uint16_t bitrate; uint8_t pb; uint8_t buffer_ms; bool two_src; const char* name; };
static const LrPreset LR_PRESET[6] = {
    { 24000, 1, 16, true,  "ECO" },       // r0c0  ~17% duty, 2 src
    { 24000, 2, 40, true,  "ECO-SAFE" },  // r0c1  ~22% duty, 2 src (default — far-field)
    { 32000, 1, 16, true,  "STD" },       // r1c0  ~20% duty, 2 src
    { 32000, 2, 40, false, "STD-SAFE" },  // r1c1  ~27% duty, 1 src
    { 48000, 1, 24, false, "HQ" },        // r2c0  ~26% duty, 1 src
    { 48000, 2, 40, false, "HQ-SAFE" },   // r2c1  ~37% duty, 1 src
};
static const uint8_t LR_PRESET_COUNT   = 6;
static const uint8_t LR_PRESET_DEFAULT = 1;   // ECO-SAFE
static uint8_t s_lr_preset = LR_PRESET_DEFAULT;   // NVS tx/lr_preset

// ---- Fleet-tune (0xAC beacon, §3.5) ---------------------------------------
// Operator-set runtime overrides broadcast to every receiver. Set via serial
// (loopTask); the beacon is sent from audioSourceLoop (also loopTask), burst ×3
// on change then resent every 5 s so late-booting receivers catch up.
static uint8_t s_fleet_val[6] = {0, 0, 0, 0, 0, 0};             // index = param_id (1-5)
static bool    s_fleet_set[6] = {false, false, false, false, false, false};
static bool    s_fleet_dirty  = false;
// Per-param epoch (P1.1, §3.5 epoch sync). Bumped on every LOCAL change (operator
// touch/serial) so peer transmitters can tell a newer local value from a stale
// echo. Persisted to NVS alongside the value+set-mask (P1.4) so a reboot resumes
// broadcasting the same value at the same epoch — a peer's beacon that only saw
// the pre-reboot epoch must not look "newer" and wrongly overwrite us back.
static uint8_t s_fleet_epoch[6] = {0, 0, 0, 0, 0, 0};
// This transmitter's own MAC (populated in audioSourceSetup via WiFi.macAddress).
// Used only for the same-epoch/different-value tie-break in audioOnRecvCb (P1.2).
static uint8_t s_self_mac[6] = {0, 0, 0, 0, 0, 0};
// Receiver volume-max (§3.5 param 5) — the 6 UI levels shown on the RX VOL screen.
static const uint8_t VOLMAX_PCTS[6] = {10, 20, 40, 60, 80, 100};

// Persist one fleet param's value + epoch to NVS ("tx" namespace, keys "fpN"/
// "feN") and mark it set in the "fp_set" bitmask (bit (param-1)) — restored at
// boot in audioSourceSetup so a reboot doesn't blank the broadcast (P1.1/P1.4).
// CALLER MUST be the loop task (fleetTick / audioSourceSetFleetParam), never the
// ESP-NOW recv callback — see s_fleet_persist_pending below.
static void persistFleetParam(uint8_t param) {
    char kv[8], ke[8];
    snprintf(kv, sizeof(kv), "fp%u", param);
    snprintf(ke, sizeof(ke), "fe%u", param);
    Preferences p; p.begin("tx", false);
    p.putUChar(kv, s_fleet_val[param]);
    p.putUChar(ke, s_fleet_epoch[param]);
    p.putUChar("fp_set", (uint8_t)(p.getUChar("fp_set", 0) | (1 << (param - 1))));
    p.end();
}

// Params adopted from a peer TX (audioOnRecvCb) whose NVS write + log line are
// still pending. Bit (param-1). audioOnRecvCb runs on the WiFi task and MUST
// NOT call Preferences/Serial there — both can block long enough to stall
// ESP-NOW RX (the same reasoning volumeServiceTick documents on the receiver
// side, and the reason repeater.cpp's recv callback only touches a lock-free
// ring buffer). fleetTick (loop task) drains this and does the actual I/O
// (self-review F2 fix).
static volatile uint8_t s_fleet_persist_pending = 0;

// TX-TX fleet-param sync (P1.2, §3.5 epoch sync). Runs in the ESP-NOW recv
// callback (WiFi task) — mirrors repeater.cpp's onReceiveCb pattern, but this
// build (AUDIO_SOURCE) previously registered no recv cb at all. Only 0xAC with
// an epoch byte [4] is actionable: two active transmitters broadcasting
// DIFFERENT fleet values (e.g. volume_max) make receivers' values step on every
// source handoff (ops runbook pre-deploy check #1) — this converges every
// transmitter on the same value+epoch within one beacon round-trip (≤5s, faster
// with the ×3 burst).
//   - RELAYED (bit7 set) is skipped: sync is origin-to-origin only, so a
//     repeater's delayed copy of our own beacon can't be mistaken for a peer
//     and cause oscillation (P1.3).
//   - No epoch (len<5, legacy/no-op sender) is skipped: nothing to compare.
//   - Newer epoch (wrap-safe int8 delta) → adopt.
//   - Same epoch, different value (only possible if two TXs changed the same
//     param independently at the same local epoch) → the numerically LARGER
//     source MAC wins (deterministic on both sides: the loser's next receive of
//     its own now-stale broadcast compares equal MAC-loss the other way and
//     adopts) (P1.2 tie-break).
//   - Same epoch, same value → already in sync, no-op (prevents a resend loop).
static void audioOnRecvCb(const uint8_t* mac, const uint8_t* data, int len) {
    if (len < 5 || data[0] != 0xAC) return;      // need the epoch byte to compare
    if (data[1] & 0x80) return;                  // RELAYED → not origin-to-origin
    uint8_t param = data[2];
    if (param < 1 || param > 5) return;
    uint8_t value = data[3];
    uint8_t epoch = data[4];
    if (s_fleet_set[param]) {
        int8_t delta = (int8_t)(epoch - s_fleet_epoch[param]);
        if (delta < 0) return;                                  // remote is stale
        if (delta == 0) {
            if (value == s_fleet_val[param]) return;              // already in sync
            if (memcmp(mac, s_self_mac, 6) <= 0) return;           // tie-break: bigger MAC wins
        }
    }
    s_fleet_val[param]   = value;
    s_fleet_epoch[param] = epoch;
    s_fleet_set[param]   = true;
    s_fleet_dirty        = true;      // adopted value gets its own ×3 burst promptly
    // NVS write + log line deferred to fleetTick (loop task) — see
    // s_fleet_persist_pending's docstring (self-review F2 fix). Setting one bit
    // in a volatile uint8 is the only work this callback does beyond RAM state.
    s_fleet_persist_pending |= (uint8_t)(1 << (param - 1));
}

// In-flight depth (send-queue buffer). With 6 Mbps a depth of ~3 already
// sustains the 1000 pkt/s stream, but a few % still drop on jitter. For
// loss-isolation we run a deliberately deep buffer so sender drops -> ~0
// (accepting more worst-case queue latency, shown on the LCD). Override per
// build with -D STREAM_MAX_INFLIGHT=N.
#ifndef STREAM_MAX_INFLIGHT
#define STREAM_MAX_INFLIGHT 16
#endif

// Piggyback size: prev_seq(1) + prev_state(6) + prev_data(FRAMES_PKT) = 7+N
static const int PIGGYBACK_HDR = 7;

// ---- display / heartbeat interval ------------------------------------------
static const uint32_t DISPLAY_INTERVAL_MS = 250;  // ~4 fps

#ifdef BOARD_CORES3
// ---- M5 Module-Audio (ES8388) driver — used for CODEC init only ------------
static M5ModuleAudio s_audio;
static int s_i2s_din = -1;   // resolved at setup (M-Bus pin26)
#else
// ---- ES8388 (M5 Module-Audio) — I2C + I2S pins (Config A: Core/Basic) ------
static const uint8_t  ES8388_ADDR = 0x10;
static const int      I2C_SDA = 21, I2C_SCL = 22;
static const int      I2S_MCLK = 0, I2S_BCLK = 13, I2S_LRCK = 12, I2S_DIN = 34;
#endif

// ---- runtime config (NVS) -------------------------------------------------
static int     s_input_level = 50;   // 0..100, 50 = unity
static uint8_t s_channel     = 1;    // ESP-NOW channel (for display)

// ---- ADPCM encoder state (continuous across packets) ----------------------
static AdpcmState s_sl, s_sr;
static uint8_t    s_seq = 0;

// ---- in-flight control (shared with send callback in WiFi task) -----------
static volatile int      s_inflight      = 0;
static volatile uint32_t s_txSendFail    = 0;
static          uint32_t s_txDroppedBusy = 0;
static          uint32_t s_txPktSent     = 0;

// ---- P0 telemetry: Opus encode time + I2S RX overflow (see quality-fix instr) --
static volatile uint64_t s_encUsSum = 0;   // sum of per-frame opus_encode() µs
static volatile uint32_t s_encUsMax = 0;   // worst per-frame opus_encode() µs
static volatile uint32_t s_encCnt   = 0;   // frames encoded this heartbeat window
static QueueHandle_t     s_i2sEvtQ  = nullptr;  // I2S_NUM_0 driver event queue
static volatile uint32_t s_i2sOvf   = 0;   // cumulative RX_Q_OVF + DMA_ERROR events

// ---- raw-input signal stats (raw L & R, pre-decimation) for the heartbeat --
static          int32_t  s_lmin = 32767, s_lmax = -32768;
static          int64_t  s_lsum = 0;
static          int32_t  s_rmin = 32767, s_rmax = -32768;
static          int64_t  s_rsum = 0;
static          uint32_t s_scnt = 0;
static          uint32_t s_capFail = 0;
static          bool     s_codecOk = true;   // CoreS3: ES8388/STM32 begin() result

// ---- HOME input-level meter: running peak of |sample| (full-scale 0..32768) --
// Written per captured sample in rawStat (any capture path); read + reset in
// uiUpdateHome so the meter reflects the last ~0.4 s. volatile: rawStat runs in
// the CoreS3 capture task (core 1) while uiUpdateHome runs in the Arduino loop.
static volatile uint16_t s_uiLpk = 0, s_uiRpk = 0;      // trimmed AC peak (bar level)
static volatile uint16_t s_rawLpk = 0, s_rawRpk = 0;    // RAW ADC peak (clip detect)

// ---- piggyback state (saved from the last successfully enqueued packet) ----
static bool       s_has_prev      = false;
static uint8_t    s_prev_seq;
static AdpcmState s_prev_sl, s_prev_sr;   // encoder state BEFORE that block
static uint8_t    s_prev_data[FRAMES_PKT];

// ---------------------------------------------------------------------------
// Send callback — registered in audioSourceSetup(), replaces EspNowSender's.
// ---------------------------------------------------------------------------
static void audioOnSendCb(const uint8_t* /*mac*/, esp_now_send_status_t status) {
    if (s_inflight > 0) s_inflight--;
    if (status != ESP_NOW_SEND_SUCCESS) s_txSendFail++;
}

// ---------------------------------------------------------------------------
// Encode one 16 kHz stereo block (FRAMES_PKT frames, interleaved L,R) and
// broadcast it as a 0xAA STREAM packet with optional piggyback. Shared by the
// classic loop path and the CoreS3 RX task.
// ---------------------------------------------------------------------------
// Mode-aware 0xAA header: [0]=type [1]=mode_id [2]=seq [3]=pb_count.
// ADPCM body (mode 0): [4]=num_frames(1B) [5..10]=state(6) [11..]=data(N)
//                      then pb_count× [prev_seq(1)][state(6)][data(N)].
static void streamEncodeAndSend(const int16_t* pcm) {
    // Snapshot encoder state BEFORE this block (goes into the header so the
    // receiver can re-sync per packet).
    AdpcmState pre_l = s_sl, pre_r = s_sr;

    // Always encode (advances state) even if we then drop, so the next packet's
    // header reflects the correct predictor state.
    uint8_t data[FRAMES_PKT];
    adpcmEncodeBlockStereo(pcm, data, FRAMES_PKT, &s_sl, &s_sr);

    // In-flight cap: drop if too many unacknowledged sends; bump seq so the
    // receiver sees a gap (handled as loss) rather than corruption.
    if (s_inflight >= STREAM_MAX_INFLIGHT) {
        s_txDroppedBusy++;
        s_seq++;
        return;
    }

    uint8_t pkt[4 + 1 + FRAMES_PKT + PIGGYBACK_HDR + FRAMES_PKT];
    size_t  pkt_len = 0;
    pkt[pkt_len++] = STREAM_TYPE;
    pkt[pkt_len++] = MODE_ADPCM;
    pkt[pkt_len++] = s_seq;
    pkt[pkt_len++] = s_has_prev ? 1 : 0;          // pb_count
    pkt[pkt_len++] = (uint8_t)FRAMES_PKT;         // num_frames (1 byte, ≤255)
    pkt[pkt_len++] = (uint8_t)(pre_l.predictor & 0xFF);
    pkt[pkt_len++] = (uint8_t)((pre_l.predictor >> 8) & 0xFF);
    pkt[pkt_len++] = pre_l.step_index;
    pkt[pkt_len++] = (uint8_t)(pre_r.predictor & 0xFF);
    pkt[pkt_len++] = (uint8_t)((pre_r.predictor >> 8) & 0xFF);
    pkt[pkt_len++] = pre_r.step_index;
    memcpy(pkt + pkt_len, data, FRAMES_PKT);
    pkt_len += FRAMES_PKT;

    if (s_has_prev) {
        pkt[pkt_len++] = s_prev_seq;
        pkt[pkt_len++] = (uint8_t)(s_prev_sl.predictor & 0xFF);
        pkt[pkt_len++] = (uint8_t)((s_prev_sl.predictor >> 8) & 0xFF);
        pkt[pkt_len++] = s_prev_sl.step_index;
        pkt[pkt_len++] = (uint8_t)(s_prev_sr.predictor & 0xFF);
        pkt[pkt_len++] = (uint8_t)((s_prev_sr.predictor >> 8) & 0xFF);
        pkt[pkt_len++] = s_prev_sr.step_index;
        memcpy(pkt + pkt_len, s_prev_data, FRAMES_PKT);
        pkt_len += FRAMES_PKT;
    }

    s_inflight++;
    esp_err_t send_err = esp_now_send(BROADCAST_MAC, pkt, pkt_len);
    if (send_err != ESP_OK) {
        s_inflight--;
        s_txSendFail++;
    } else {
        s_txPktSent++;
        s_has_prev   = true;
        s_prev_seq   = s_seq;
        s_prev_sl    = pre_l;
        s_prev_sr    = pre_r;
        memcpy(s_prev_data, data, FRAMES_PKT);
    }
    s_seq++;
}

// Accumulate raw-input stats (raw L & R samples) for the heartbeat diagnostic.
static inline void rawStat(int16_t l, int16_t r) {
    if (l < s_lmin) s_lmin = l;
    if (l > s_lmax) s_lmax = l;
    s_lsum += l;
    if (r < s_rmin) s_rmin = r;
    if (r > s_rmax) s_rmax = r;
    s_rsum += r;
    s_scnt++;
    // Meter CLIP detection reads the RAW ADC peak here (pre-trim): the ADC
    // saturates on the raw sample, so clipping must be judged BEFORE the L/R
    // balance trim (which scales R down and would otherwise hide R's clipping —
    // R can clip the ADC while its trimmed bar still reads ~45%).
    uint16_t al = (uint16_t)(l < 0 ? -(int32_t)l : (int32_t)l);
    uint16_t ar = (uint16_t)(r < 0 ? -(int32_t)r : (int32_t)r);
    if (al > s_rawLpk) s_rawLpk = al;
    if (ar > s_rawRpk) s_rawRpk = ar;
    // The level BARS read the DC-blocked, L/R-balanced signal in audioRxTask
    // (s_uiLpk/s_uiRpk) — the raw DC offset would mislead the bar level.
}

// Periodic serial heartbeat (~2 s) for headless verification.
static void heartbeatTick() {
    static uint32_t s_last_hb = 0;
    uint32_t now = millis();
    if (now - s_last_hb < 2000) return;
    s_last_hb = now;
    long lmean = s_scnt ? (long)(s_lsum / (int64_t)s_scnt) : 0;
    long rmean = s_scnt ? (long)(s_rsum / (int64_t)s_scnt) : 0;
    uint32_t enc_avg = s_encCnt ? (uint32_t)(s_encUsSum / s_encCnt) : 0;
    Serial.printf("[AUDIO-SRC] hb codec=%d pkt=%u capfail=%u drop=%u fail=%u "
                  "enc=%u/%uus ovf=%u heap=%u/%u inflight=%d "
                  "L[%ld..%ld m%ld] R[%ld..%ld m%ld]\n",
                  s_codecOk ? 1 : 0,
                  s_txPktSent, s_capFail, s_txDroppedBusy, (uint32_t)s_txSendFail,
                  enc_avg, (uint32_t)s_encUsMax, (uint32_t)s_i2sOvf,
                  (unsigned)esp_get_free_heap_size(),
                  (unsigned)esp_get_minimum_free_heap_size(), s_inflight,
                  (long)s_lmin, (long)s_lmax, lmean,
                  (long)s_rmin, (long)s_rmax, rmean);
    s_lmin = 32767; s_lmax = -32768; s_lsum = 0;
    s_rmin = 32767; s_rmax = -32768; s_rsum = 0;
    s_scnt = 0;
    s_encUsSum = 0; s_encUsMax = 0; s_encCnt = 0;
}

#ifndef BOARD_CORES3
// ---- ES8388 helpers (classic Core/Basic only) -----------------------------
static void es8388Write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ES8388_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Minimal ES8388 line-in → ADC → I2S record config (LINPUT1/RINPUT1).
static void es8388Init() {
    es8388Write(0x00, 0x80); delay(10);   // reset
    es8388Write(0x00, 0x00);
    es8388Write(0x01, 0x50);              // power management
    es8388Write(0x02, 0x00);              // power up
    es8388Write(0x03, 0x00);              // ADC power on
    es8388Write(0x04, 0xFC);              // DAC power down (record-only)
    es8388Write(0x08, 0x00);              // I2S slave (ESP32 is master)
    es8388Write(0x09, 0x88);              // ADC: line-in LINPUT1/RINPUT1, 0 dB
    es8388Write(0x0A, 0x00);              // input select
    es8388Write(0x0B, 0x02);
    es8388Write(0x0C, 0x0C);              // ADC I2S: 16-bit, standard I2S
    es8388Write(0x0D, 0x02);              // ADC MCLK/LRCK ratio
    es8388Write(0x10, 0x00);              // ADC L volume 0 dB
    es8388Write(0x11, 0x00);              // ADC R volume 0 dB
}
#endif // !BOARD_CORES3

// ---- I2S RX init (raw driver, small DMA buffers) — Core/Basic AND CoreS3 ----
// RX-only master, mirroring the reference wireless-sender-firmware. The I2S
// master generates MCLK on `mck` regardless of TX, so the ES8388 is clocked
// (verified: switch A → MCLK on GPIO0). Small DMA buffers keep capture latency
// low; empirically this RX-only/small-buffer path also keeps the 6 Mbps
// ESP-NOW rate effective (the M5 driver's TX|RX + large-buffer path did not).
static void i2sRxInit(int mck, int bck, int ws, int din) {
    i2s_config_t cfg = {};
    cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    cfg.sample_rate          = CAPTURE_RATE;
    cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count        = 16;   // deep DMA (16×32/48000 = 10.67 ms) rides out encode stalls
    cfg.dma_buf_len          = 32;   // 0.67 ms read chunk @ 48 kHz (16 bufs = 10.67 ms total)
#ifdef BOARD_CORES3
    // ESP32-S3: APLL-derived MCLK can fail to clock the ES8388 (DIN reads 0).
    // The M5Module-Audio driver uses the non-APLL path on S3 and clocks fine.
    cfg.use_apll             = false;
#else
    cfg.use_apll             = true;   // classic ESP32: APLL low-jitter MCLK
#endif
    cfg.tx_desc_auto_clear   = true;
    cfg.fixed_mclk           = 0;

#if !defined(BOARD_CORES3) && defined(CONFIG_IDF_TARGET_ESP32)
    // Classic ESP32 needs GPIO0 routed to CLK_OUT1 for MCLK.
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
#endif

    i2s_pin_config_t pins = {};
    pins.mck_io_num   = mck;
    pins.bck_io_num   = bck;
    pins.ws_io_num    = ws;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num  = din;

    // Install WITH an event queue (depth 32) so RX overflow (RX_Q_OVF) and DMA
    // errors become observable — otherwise silent DMA drops look like clean audio.
    i2s_driver_install(I2S_NUM_0, &cfg, 32, &s_i2sEvtQ);
    i2s_set_pin(I2S_NUM_0, &pins);
    i2s_set_clk(I2S_NUM_0, CAPTURE_RATE, I2S_BITS_PER_SAMPLE_16BIT,
                I2S_CHANNEL_STEREO);
}

// Drain the I2S event queue NON-BLOCKING and tally overflow/DMA-error events.
// MUST be called every capture iteration: at len=32 the driver posts ~1500
// RX_DONE events/s, so a per-heartbeat drain would let the queue fill and lose
// the very overflow events we are counting. Shared by the CoreS3 task and the
// classic loop.
static inline void drainI2sEvents() {
    if (!s_i2sEvtQ) return;
    i2s_event_t ev;
    while (xQueueReceive(s_i2sEvtQ, &ev, 0) == pdTRUE) {
        if (ev.type == I2S_EVENT_RX_Q_OVF || ev.type == I2S_EVENT_DMA_ERROR)
            s_i2sOvf++;
    }
}

#ifdef BOARD_CORES3
// ---- Anti-alias decimation FIR (48 kHz -> 16 kHz, /3) -----------------------
// The previous 3-tap box average is a poor LPF (~-13 dB stopband), so content
// in 8-24 kHz folds back into 0-8 kHz as a high-frequency "sizzle". This is a
// proper windowed-sinc low-pass (Hann), run as a decimating FIR.
static const int FIR_TAPS = 21;
static float   s_firCoef[FIR_TAPS];
static int16_t s_firHL[FIR_TAPS];
static int16_t s_firHR[FIR_TAPS];

static void firInit() {
    const float PI_F = 3.14159265358979f;
    const float fc = 6500.0f, fs = 48000.0f;   // cutoff well below the 8 kHz out-Nyquist
    float sum = 0.0f;
    for (int n = 0; n < FIR_TAPS; n++) {
        float m = (float)n - (FIR_TAPS - 1) * 0.5f;
        float s = (m == 0.0f) ? (2.0f * fc / fs)
                              : sinf(2.0f * PI_F * fc / fs * m) / (PI_F * m);
        float w = 0.5f - 0.5f * cosf(2.0f * PI_F * (float)n / (FIR_TAPS - 1)); // Hann
        s_firCoef[n] = s * w;
        sum += s_firCoef[n];
    }
    for (int n = 0; n < FIR_TAPS; n++) s_firCoef[n] /= sum;   // unity DC gain
    for (int n = 0; n < FIR_TAPS; n++) { s_firHL[n] = 0; s_firHR[n] = 0; }
}

static inline void firPush(int16_t l, int16_t r) {
    for (int i = 0; i < FIR_TAPS - 1; i++) { s_firHL[i] = s_firHL[i+1]; s_firHR[i] = s_firHR[i+1]; }
    s_firHL[FIR_TAPS-1] = l; s_firHR[FIR_TAPS-1] = r;
}
static inline void firGet(int32_t* ol, int32_t* outr) {
    float al = 0.0f, ar = 0.0f;
    for (int i = 0; i < FIR_TAPS; i++) { al += s_firCoef[i] * s_firHL[i]; ar += s_firCoef[i] * s_firHR[i]; }
    *ol = (int32_t)al; *outr = (int32_t)ar;
}

// input_level (0-100) -> analog input PGA gain. 50 = 0 dB (preserves a healthy
// source level / no clipping); >50 adds up to +24 dB for quiet sources.
static es_mic_gain_t pgaFromLevel(int lvl) {
    int idx = (lvl - 50) * 8 / 50;
    if (idx < 0) idx = 0;
    if (idx > 8) idx = 8;
    return (es_mic_gain_t)idx;
}

// ---- Opus encode (modes 1-3; CoreS3 only) ---------------------------------
// libopus (NONTHREADSAFE_PSEUDOSTACK) keeps its temp buffers in ONE global
// scratch, lazily malloc'd as opus_alloc_scratch(GLOBAL_STACK_SIZE=60000) on the
// first encode. On the CoreS3 the internal heap is fragmented (M5GFX framebuffer
// + Wi-Fi/ESP-NOW) so that 60 KB contiguous malloc FAILS → global_stack=NULL →
// StoreProhibited on the first opus_encode. We pre-seed global_stack ourselves
// (internal SRAM for speed; PSRAM fallback) before the first encode.
extern "C" char* global_stack;
static OpusEncoder* s_enc      = nullptr;
static uint8_t      s_enc_mode = 0xFF;          // mode s_enc is configured for
static int16_t      s_opusAccum[OPUS_MAX_SMP * 2];  // frame accumulator (×2 = stereo)
static int          s_opusAccumN = 0;               // count of int16 written (interleaved)
static int          s_decim2     = 0;           // 16k→8k 2:1 decimation phase
static int32_t      s_decim2prev = 0;           // 2-tap average state (mono)
static int32_t      s_decLprev   = 0;           // 2-tap average state (stereo L)
static int32_t      s_decRprev   = 0;           // 2-tap average state (stereo R)
// Opus frame history for piggyback: shift-ring of 3 (hist[0]=most recent prev,
// hist[1]=older, hist[2]=oldest). Depth 3 lets FAST/BALANCED emit up to pb3.
static uint8_t      s_opHist[3][OPUS_MAX_LEN];
static uint8_t      s_opHistLen[3] = {0, 0, 0};
static uint8_t      s_opHistSeq[3] = {0, 0, 0};
static int          s_opHistN      = 0;         // valid history entries (0..3)

// (Re)configure the Opus encoder for `mode` (1-3). Encoder state lives in
// INTERNAL SRAM (PSRAM wait states would ~2× the encode time — see gate①).
static void opusEncoderConfig(uint8_t mode) {
    if (mode < MODE_L || mode >= MODE_COUNT) return;
    // Pre-seed libopus's NONTHREADSAFE_PSEUDOSTACK scratch (see extern above).
    // pschatzmann's ALLOC_STACK keeps a non-NULL global_stack as-is, so THIS
    // buffer's size — not the library's GLOBAL_STACK_SIZE (60000) — is the real
    // ceiling. 8 kHz mono needed <64 KB; 16 kHz stereo (HIFI) uses more CELT
    // scratch; 48 kHz stereo (SOLID48) is the new worst case, so seed 128 KB
    // internal (SRAM for fast encode; ~200 KB largest free block available)
    // with a PSRAM fallback. An overflow here would be a silent heap
    // corruption, so err toward headroom.
    if (!global_stack) {
        global_stack = (char*)heap_caps_malloc(128 * 1024,
                             MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!global_stack)
            global_stack = (char*)heap_caps_malloc(160 * 1024, MALLOC_CAP_SPIRAM);
        Serial.printf("[AUDIO-SRC] opus scratch @ %p (largest internal free %u)\n",
                      (void*)global_stack,
                      (unsigned)heap_caps_get_largest_free_block(
                          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    uint16_t rate = MODE_DEFS[mode].rate;
    uint8_t  ch   = MODE_DEFS[mode].channels;
    if (!s_enc) {
        // Created ONCE per boot, sized + init'd for THIS mode's rate/channels.
        // Safe as init-once because every mode change reboots (audioSourceSetMode
        // → ESP.restart), so the encoder is always fresh for the persisted mode.
        // (A live RESET_STATE + re-ctl mid-stream was crashing the encoder, and
        // rate/channel changes need a differently-sized encoder anyway.)
        int sz = opus_encoder_get_size(ch);
        s_enc = (OpusEncoder*)heap_caps_malloc((size_t)sz,
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_enc) { Serial.println("[AUDIO-SRC] opus enc alloc FAILED"); return; }
        opus_encoder_init(s_enc, rate, ch, OPUS_APPLICATION_RESTRICTED_LOWDELAY);
        // RANGE=LONG on the id3 host uses the preset's low bitrate (§7.3), not
        // MODE_DEFS[3]'s 60k — a low bitrate is what makes LR 2-source-viable.
        uint32_t br = (s_range_long && mode == LR_MODE) ? LR_PRESET[s_lr_preset].bitrate
                                                        : MODE_DEFS[mode].bitrate;
        opus_encoder_ctl(s_enc, OPUS_SET_BITRATE(br));
        opus_encoder_ctl(s_enc, OPUS_SET_COMPLEXITY(MODE_DEFS[mode].complexity));
        opus_encoder_ctl(s_enc, OPUS_SET_VBR(0));
    }
    s_enc_mode   = mode;   // switches opusPushSample's frame size / channels
    s_opusAccumN = 0;      // drop the partial frame from the old size
    s_opHistN    = 0;      // old-size piggyback frames are invalid now
    s_decim2     = 0;
    Serial.printf("[AUDIO-SRC] opus enc mode=%s (%u Hz %s, %u smp/ch, %u bps)\n",
                  MODE_NAME[mode], rate, ch == 2 ? "stereo" : "mono",
                  MODE_DEFS[mode].opus_frame, MODE_DEFS[mode].bitrate);
}

// Assemble + broadcast one Opus 0xAA packet (mode 1-3) with piggyback.
// Body: [4]=opus_len(1) [5..]=frame ; then pb× [prev_seq(1)][len(1)][data].
static void sendOpusPacket(const uint8_t* frame, int flen) {
    uint8_t mode = s_mode;
    // RANGE=LONG uses the LR preset's piggyback (§7.3) to trim the airtime budget.
    int depth = (s_range_long && mode == LR_MODE) ? LR_PRESET[s_lr_preset].pb
                                                  : MODE_DEFS[mode].piggyback;
    int pbn   = s_opHistN < depth ? s_opHistN : depth;

    if (s_inflight >= STREAM_MAX_INFLIGHT) {
        s_txDroppedBusy++;
    } else {
        // Static worst-case bound: header(4) + opus_len(1) + frame(<=OPUS_MAX_LEN)
        // + 3 piggyback entries × [prev_seq(1)+len(1)+data(<=OPUS_MAX_LEN)]. Max
        // piggyback in MODE_DEFS is now 3, so factor 3: 4+1+160 + 3*(2+160) = 651 B.
        // The real runtime packet is far smaller (FAST/BAL 131, STEREO 249) and the
        // >250 guard below rejects anything over ESP-NOW's 250 B limit.
        uint8_t pkt[4 + 1 + OPUS_MAX_LEN + 3 * (2 + OPUS_MAX_LEN)];
        size_t p = 0;
        pkt[p++] = STREAM_TYPE;
        pkt[p++] = mode;
        pkt[p++] = s_seq;
        pkt[p++] = (uint8_t)pbn;
        pkt[p++] = (uint8_t)flen;
        memcpy(pkt + p, frame, flen); p += flen;
        // piggyback: previous pbn frames, OLDEST first (seq order for decode).
        for (int h = pbn - 1; h >= 0; h--) {
            pkt[p++] = s_opHistSeq[h];
            pkt[p++] = s_opHistLen[h];
            memcpy(pkt + p, s_opHist[h], s_opHistLen[h]); p += s_opHistLen[h];
        }
        // ESP-NOW hard limit is 250 B (ESP_NOW_MAX_DATA_LEN). A bitrate/piggyback
        // combo that overflows would fail the send anyway — skip + warn so it's
        // visible in the log instead of a silent stream stall.
        if (p > 250) {
            Serial.printf("[AUDIO-SRC] opus packet %u B > 250, skipping (mode=%u)\n",
                          (unsigned)p, mode);
        } else {
            s_inflight++;
            esp_err_t e = esp_now_send(BROADCAST_MAC, pkt, p);
            if (e != ESP_OK) { s_inflight--; s_txSendFail++; }
            else s_txPktSent++;
        }
    }
    // Record current frame into history (shift ring of 3) + advance seq — done
    // even on a busy-drop so the NEXT packet's piggyback can still recover this
    // frame. Shift: hist[2] <- hist[1] <- hist[0] <- current.
    memcpy(s_opHist[2], s_opHist[1], s_opHistLen[1]);
    s_opHistLen[2] = s_opHistLen[1]; s_opHistSeq[2] = s_opHistSeq[1];
    memcpy(s_opHist[1], s_opHist[0], s_opHistLen[0]);
    s_opHistLen[1] = s_opHistLen[0]; s_opHistSeq[1] = s_opHistSeq[0];
    memcpy(s_opHist[0], frame, (size_t)flen);
    s_opHistLen[0] = (uint8_t)flen;  s_opHistSeq[0] = s_seq;
    if (s_opHistN < 3) s_opHistN++;
    s_seq++;
}

// Accumulate one sample (mono: use `l`, r ignored; stereo: interleave l,r at the
// mode's rate). Encode + send when a full per-channel frame is ready.
static inline void opusPushSample(int16_t l, int16_t r) {
    if (!s_enc || s_enc_mode < MODE_L) return;
    uint8_t ch = MODE_DEFS[s_enc_mode].channels;
    s_opusAccum[s_opusAccumN++] = l;
    if (ch == 2) s_opusAccum[s_opusAccumN++] = r;
    int need = MODE_DEFS[s_enc_mode].opus_frame;   // samples PER CHANNEL
    if (s_opusAccumN >= need * ch) {
        uint8_t buf[OPUS_MAX_LEN];
        int64_t t0 = esp_timer_get_time();
        int n = opus_encode(s_enc, s_opusAccum, need, buf, sizeof(buf));
        uint32_t dt = (uint32_t)(esp_timer_get_time() - t0);
        s_encUsSum += dt;
        if (dt > s_encUsMax) s_encUsMax = dt;
        s_encCnt++;
        s_opusAccumN = 0;
        if (n > 0) sendOpusPacket(buf, n);
    }
}

// ---- Mono ADPCM encode (mode 6 LITE; CoreS3 8k-mono path) ------------------
// A zero-encode-CPU sibling to the Opus mono modes: the SAME 8 kHz-mono capture
// path (downmix + 2:1 decimate), IMA-ADPCM encoded (2 samples/byte) with a
// depth-3 piggyback (like FAST) for burst-loss resilience. The lowest-latency
// mode (no Opus encode/algorithmic delay — measured ~13 ms below FAST). Kept
// SEPARATE from the stereo mode-0 state (s_sl/s_sr/s_prev_*) so a live mode
// switch can't cross-corrupt either encoder or its piggyback history.
static const int  MONO8K_FRAMES = 40;                     // 8k mono samples/packet (5 ms)
static const int  MONO8K_DBYTES = (MONO8K_FRAMES + 1) / 2;// 20 ADPCM bytes (2 smp/byte)
static AdpcmState s_sm;                                   // mono encoder state (continuous)
// Mono piggyback: shift-ring of 3 (hist[0]=most recent prev … hist[2]=oldest).
// Each entry = prev_seq(1) + state(pred_lo,pred_hi,step = 3) + data(MONO8K_DBYTES).
static int        s_monoHistN = 0;                        // valid history entries (0..3)
static uint8_t    s_monoHistSeq[3];
static AdpcmState s_monoHistSm[3];                        // encoder state BEFORE each block
static uint8_t    s_monoHistData[3][MONO8K_DBYTES];

// Encode one 40-sample 8 kHz mono block and broadcast a mode-6 (LITE) or mode-7
// (TURBO) 0xAA packet with up to depth-3 piggyback (MODE_DEFS[mode].piggyback).
// LITE and TURBO share this exact encode/wire path — `mode` only selects which
// mode_id ships in the header (the RX-side jitter buffer depth is what differs).
// Mirrors streamEncodeAndSend()'s in-flight/seq/prev-frame bookkeeping but for
// the 3-byte mono state (pred int16 LE + step).
// Body: [4]=num_frames(40) [5..7]=state(pred_lo,pred_hi,step) [8..27]=data(20);
//       then pb× [prev_seq(1)][pred_lo][pred_hi][step][data(20)] (oldest first).
// Max 100 B (pb=3): 4 + 1 + 3 + 20 + 3*(1+3+20).
static void streamEncodeAndSendMono(const int16_t* mono, uint8_t mode) {
    // Snapshot encoder state BEFORE this block (goes into the header so the
    // receiver can re-sync per packet).
    AdpcmState pre_m = s_sm;

    // Always encode (advances state) even if we then drop, so the next packet's
    // header reflects the correct predictor state.
    uint8_t data[MONO8K_DBYTES];
    adpcmEncodeBlockMono(mono, data, MONO8K_FRAMES, &s_sm);

    // In-flight cap: drop if too many unacknowledged sends; bump seq so the
    // receiver sees a gap (handled as loss) rather than corruption.
    if (s_inflight >= STREAM_MAX_INFLIGHT) {
        s_txDroppedBusy++;
        s_seq++;
        return;
    }

    // pb entries this packet = min(configured depth, valid history). LITE and
    // TURBO share MODE_DEFS.piggyback (both 3) — indexing by `mode` keeps this
    // correct if that ever diverges.
    int depth = MODE_DEFS[mode].piggyback;
    int pbn   = s_monoHistN < depth ? s_monoHistN : depth;

    uint8_t pkt[4 + 1 + 3 + MONO8K_DBYTES + 3 * (1 + 3 + MONO8K_DBYTES)];
    size_t  pkt_len = 0;
    pkt[pkt_len++] = STREAM_TYPE;
    pkt[pkt_len++] = mode;
    pkt[pkt_len++] = s_seq;
    pkt[pkt_len++] = (uint8_t)pbn;                    // pb_count
    pkt[pkt_len++] = (uint8_t)MONO8K_FRAMES;          // num_frames (mono samples)
    pkt[pkt_len++] = (uint8_t)(pre_m.predictor & 0xFF);
    pkt[pkt_len++] = (uint8_t)((pre_m.predictor >> 8) & 0xFF);
    pkt[pkt_len++] = pre_m.step_index;
    memcpy(pkt + pkt_len, data, MONO8K_DBYTES);
    pkt_len += MONO8K_DBYTES;

    // piggyback: previous pbn frames, OLDEST first (seq order for decode).
    for (int h = pbn - 1; h >= 0; h--) {
        pkt[pkt_len++] = s_monoHistSeq[h];
        pkt[pkt_len++] = (uint8_t)(s_monoHistSm[h].predictor & 0xFF);
        pkt[pkt_len++] = (uint8_t)((s_monoHistSm[h].predictor >> 8) & 0xFF);
        pkt[pkt_len++] = s_monoHistSm[h].step_index;
        memcpy(pkt + pkt_len, s_monoHistData[h], MONO8K_DBYTES);
        pkt_len += MONO8K_DBYTES;
    }

    s_inflight++;
    esp_err_t send_err = esp_now_send(BROADCAST_MAC, pkt, pkt_len);
    if (send_err != ESP_OK) {
        s_inflight--;
        s_txSendFail++;
    } else {
        s_txPktSent++;
        // Record this frame into history (shift ring of 3, newest at [0]).
        s_monoHistSeq[2] = s_monoHistSeq[1]; s_monoHistSm[2] = s_monoHistSm[1];
        memcpy(s_monoHistData[2], s_monoHistData[1], MONO8K_DBYTES);
        s_monoHistSeq[1] = s_monoHistSeq[0]; s_monoHistSm[1] = s_monoHistSm[0];
        memcpy(s_monoHistData[1], s_monoHistData[0], MONO8K_DBYTES);
        s_monoHistSeq[0] = s_seq; s_monoHistSm[0] = pre_m;
        memcpy(s_monoHistData[0], data, MONO8K_DBYTES);
        if (s_monoHistN < 3) s_monoHistN++;
    }
    s_seq++;
}

// ---- CoreS3 capture task: raw i2s_read (small chunks) → FIR decimate → send -
static void audioRxTask(void* /*arg*/) {
    static int16_t accum[FRAMES_PKT * 2];   // 16 kHz stereo, one ADPCM packet (mode 0)
    int accumIdx = 0;
    static int16_t monoAccum[MONO8K_FRAMES]; // 8 kHz mono, one ADPCM packet (mode 6)
    int monoAccumN = 0;
    int monoDecPhase = 0;                    // 16k→8k 2:1 decimation phase (mode 6)
    int32_t monoDecPrev = 0;                 // 2-tap average state (mode 6, distinct from s_decim2)
    uint32_t dcount = 0;
    uint8_t taskMode = 0xFF;                 // detect live mode switches
    static int16_t rx[FRAMES_PKT * 2];      // read chunk (64 stereo frames)
    for (;;) {
        drainI2sEvents();   // tally RX overflow / DMA errors every iteration
        size_t br = 0;
        esp_err_t e = i2s_read(I2S_NUM_0, rx, sizeof(rx), &br, portMAX_DELAY);
        if (e != ESP_OK || br < 4) { s_capFail++; vTaskDelay(1); continue; }
        size_t n = br / sizeof(int16_t);
        for (size_t i = 0; i + 1 < n; i += 2) {
            // L/R swapped vs the raw I2S slot order (receiver was reversed).
            int16_t L = rx[i + 1];
            int16_t R = rx[i];
            rawStat(L, R);
            // Mode 9 (SOLID48): full 48 kHz stereo direct into Opus, bypassing the
            // FIR decimation entirely. Checked here (before firPush/dcount) since
            // the mode-switch/encoder-config block below only runs at the
            // decimated-sample rate and would never fire for a mode that skips
            // that gate on every sample — so this mode configures its own encoder
            // once, on the first sample after boot. NOTE: the DC-blocker, R-trim,
            // and UI level-meter live INSIDE the decimation gate below, so this
            // mode streams raw (no DC-block/R-trim/meter) — acceptable for now.
            if (s_mode == MODE_SOLID48) {
                if (taskMode != MODE_SOLID48) { opusEncoderConfig(MODE_SOLID48); taskMode = MODE_SOLID48; }
                opusPushSample(L, R);
                continue;
            }
            firPush(L, R);
            if (++dcount >= (uint32_t)DECIMATE) {
                dcount = 0;
                int32_t l, r;
                firGet(&l, &r);          // gain is analog (PGA); FIR is unity DC
                // DC blocker (1-pole HPF ~10 Hz). With ALC off the ES8388's DC
                // servo is gone, exposing an ADC DC offset — remove it so no DC
                // reaches the motor. Transparent above ~30 Hz.
                static float dcxl = 0, dcyl = 0, dcxr = 0, dcyr = 0;
                const float DCA = 0.996f;
                dcyl = (float)l - dcxl + DCA * dcyl; dcxl = (float)l; l = (int32_t)dcyl;
                dcyr = (float)r - dcxr + DCA * dcyr; dcxr = (float)r; r = (int32_t)dcyr;
                // Per-channel L/R balance: the M5 Module-Audio line-in captures R
                // ~+3.3 dB hotter than L (analog ES8388/board mismatch; measured
                // R/L=1.47 with a balanced source). The M5 library exposes no per-
                // channel gain and the ES8388 ADC-volume register did not attenuate
                // it, so trim R here on the DC-blocked signal. Balances both the
                // encoded stream and the meter. Tune R_TRIM_PCT (100 = no trim).
                static const int R_TRIM_PCT = 68;        // R * 0.68 ≈ -3.3 dB
                r = (int32_t)r * R_TRIM_PCT / 100;
                if (l > 32767) l = 32767; else if (l < -32768) l = -32768;
                if (r > 32767) r = 32767; else if (r < -32768) r = -32768;
                // HOME level meter: peak of the DC-blocked, L/R-balanced AC signal.
                { uint16_t al = (uint16_t)(l < 0 ? -l : l);
                  uint16_t ar = (uint16_t)(r < 0 ? -r : r);
                  if (al > s_uiLpk) s_uiLpk = al;
                  if (ar > s_uiRpk) s_uiRpk = ar; }

                // Route by the (live-switchable) stream mode.
                uint8_t mode = s_mode;
                if (mode != taskMode) {
                    // Reset per-mode accumulation so a live switch never carries a
                    // partial block or stale piggyback into the new mode.
                    accumIdx = 0;                       // stereo ADPCM (mode 0) block
                    monoAccumN = 0;                     // mono ADPCM  (mode 6) block
                    monoDecPhase = 0;                   // mono 16k→8k decimation phase
                    if (MODE_DEFS[mode].codec == 1) {
                        opusEncoderConfig(mode);        // Opus modes (1-5, 8): (re)configure encoder
                    } else if (mode == MODE_MONO8K || mode == MODE_TURBO) {
                        adpcmStateInit(&s_sm);          // fresh mono encoder state
                        s_monoHistN = 0;                // drop stale mono piggyback
                    }
                    taskMode = mode;
                }
                if (mode == MODE_ADPCM) {
                    accum[accumIdx * 2]     = (int16_t)l;
                    accum[accumIdx * 2 + 1] = (int16_t)r;
                    if (++accumIdx >= FRAMES_PKT) { streamEncodeAndSend(accum); accumIdx = 0; }
                } else if (mode == MODE_MONO8K || mode == MODE_TURBO) {
                    // 8 kHz mono ADPCM (LITE / TURBO — identical encode+wire, only
                    // the receiver's jitter buffer depth differs): downmix + 2:1
                    // decimate 16k stereo → 8k mono (same as the 8k-mono Opus path),
                    // then pack 40 samples (5 ms) into one mode-6/7 ADPCM packet.
                    int32_t mono16 = ((int32_t)l + r) / 2;
                    if (monoDecPhase == 0) { monoDecPrev = mono16; monoDecPhase = 1; }
                    else {
                        monoDecPhase = 0;
                        monoAccum[monoAccumN++] = (int16_t)((monoDecPrev + mono16) / 2);
                        if (monoAccumN >= MONO8K_FRAMES) { streamEncodeAndSendMono(monoAccum, mode); monoAccumN = 0; }
                    }
                } else if (MODE_DEFS[mode].rate >= 16000) {
                    // 16 kHz Opus: no decimation. Stereo keeps L/R; mono downmixes.
                    if (MODE_DEFS[mode].channels == 2)
                        opusPushSample((int16_t)l, (int16_t)r);
                    else
                        opusPushSample((int16_t)(((int32_t)l + r) / 2), 0);
                } else if (MODE_DEFS[mode].channels == 2) {
                    // 8 kHz stereo Opus: 2:1 decimate L and R separately (2-tap avg).
                    if (s_decim2 == 0) { s_decLprev = l; s_decRprev = r; s_decim2 = 1; }
                    else { s_decim2 = 0;
                           opusPushSample((int16_t)((s_decLprev + l) / 2),
                                          (int16_t)((s_decRprev + r) / 2)); }
                } else {
                    // 8 kHz mono Opus: downmix + 2:1 decimate 16k stereo → 8k mono.
                    int32_t mono16 = ((int32_t)l + r) / 2;
                    if (s_decim2 == 0) { s_decim2prev = mono16; s_decim2 = 1; }
                    else { s_decim2 = 0; opusPushSample((int16_t)((s_decim2prev + mono16) / 2), 0); }
                }
            }
        }
    }
}
#endif // BOARD_CORES3

// ---------------------------------------------------------------------------
void audioSourceSetup() {
    // Performance-first: source/repeater are mains-powered, so never downclock.
    setCpuFrequencyMhz(240);

    // P0c: log why we (re)booted — distinguishes brownout (v3 LDO dropout) from
    // a panic/TWDT when chasing the "one-off reset + quality decay" symptom.
    Serial.printf("[AUDIO-SRC] reset_reason=%d\n", (int)esp_reset_reason());

    {
        Preferences p;
        p.begin("espnow", true);
        s_channel = p.getUChar("channel", ESPNOW_CHANNEL);   // default matches the radio (config.h)
        p.end();
        p.begin("tx", true);
        s_input_level = p.getInt("input_level", 50);
        s_mode        = p.getUChar("mode", MODE_ADPCM);
        if (s_mode >= MODE_COUNT) s_mode = MODE_ADPCM;
        s_range_long  = p.getBool("range_long", false);
        s_lr_preset   = p.getUChar("lr_preset", LR_PRESET_DEFAULT);
        if (s_lr_preset >= LR_PRESET_COUNT) s_lr_preset = LR_PRESET_DEFAULT;
        // Restore fleet-tune params (P1.4): a reboot must resume broadcasting the
        // same value at the same epoch, or a peer TX that only saw the pre-reboot
        // epoch would see us as "still at the old value" and could win a stale
        // tie-break against a legitimate newer local change made just before boot.
        uint8_t fp_set = p.getUChar("fp_set", 0);
        for (uint8_t pid = 1; pid <= 5; pid++) {
            if (!(fp_set & (1 << (pid - 1)))) continue;
            char kv[8], ke[8];
            snprintf(kv, sizeof(kv), "fp%u", pid);
            snprintf(ke, sizeof(ke), "fe%u", pid);
            s_fleet_val[pid]   = p.getUChar(kv, 0);
            s_fleet_epoch[pid] = p.getUChar(ke, 0);
            s_fleet_set[pid]   = true;
            s_fleet_dirty      = true;   // resume the ×3 burst + 5s resend on boot
        }
        p.end();
    }
    WiFi.macAddress(s_self_mac);   // for the TX-TX sync tie-break (P1.2)
#ifdef BOARD_CORES3
    // RANGE=LONG pins the LR profile (id3 Opus wire; CoreS3-only). Force the mode
    // here so the persisted audio mode is ignored while LONG (restored on NORMAL).
    if (s_range_long) s_mode = LR_MODE;
#else
    s_range_long = false;   // classic sender has no Opus → no LR profile
#endif

    // EspNowSender::init() (main.cpp) already did WiFi STA + radio tuning
    // (6 Mbps, PS off) + esp_now_init. Replace its send callback with ours so
    // we can track in-flight depth.
    esp_now_register_send_cb(audioOnSendCb);
    // TX-TX fleet-param sync (P1.2): previously AUDIO_SOURCE registered no recv
    // cb at all (a sender never needed to receive). audioOnRecvCb only acts on
    // 0xAC fleet-tune beacons from a PEER transmitter (own broadcasts are not
    // looped back to the sender by ESP-NOW).
    esp_now_register_recv_cb(audioOnRecvCb);

#ifdef BOARD_CORES3
    if (s_range_long) {
        // RANGE=LONG (§7.3): 802.11 LR / LORA_500K, overriding EspNowSender::init()'s
        // bgn / 6M. The LR profile is Opus (id3) → CoreS3-only.
        esp_wifi_set_protocol(WIFI_IF_STA,
            WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
        esp_err_t re = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_LORA_500K);
        const LrPreset& pr = LR_PRESET[s_lr_preset];
        Serial.printf("[AUDIO-SRC] RANGE=LONG preset=%s (id%u %u bps pb%u buf%ums) LR rc=%d\n",
                      pr.name, LR_MODE, pr.bitrate, pr.pb, pr.buffer_ms, (int)re);
        // Distribute this preset's receiver jitter-buffer depth to the whole fleet
        // via a 0xAC param-1 beacon (auto ×3 + 5 s resend). Receivers need no reflash.
        audioSourceSetFleetParam(1, pr.buffer_ms);
    }
#endif

    adpcmStateInit(&s_sl);
    adpcmStateInit(&s_sr);

#ifdef BOARD_CORES3
    // Keep the Module Audio A/B switch on "A" (classic wiring). A/B only swaps
    // which M-Bus pin carries SCLK vs MCLK; the ESP32-S3 GPIO matrix lets us
    // drive them on the A-position pins, so the same module also works on a
    // Basic/Core2 without flipping the switch. Use the M5 driver for CODEC init
    // only (board-correct I2C + STM32 input enable); we own the I2S below so we
    // can use small DMA buffers. M5.begin() (main.cpp) powered the M-Bus.
    int sda = M5.getPin(m5::pin_name_t::in_i2c_sda);
    int scl = M5.getPin(m5::pin_name_t::in_i2c_scl);
    int bck = M5.getPin(m5::pin_name_t::mbus_pin22);  // SCLK  (switch A)
    int mck = M5.getPin(m5::pin_name_t::mbus_pin24);  // MCLK  (switch A)
    int ws  = M5.getPin(m5::pin_name_t::mbus_pin21);  // LRCK   (A/B same)
    s_i2s_din = M5.getPin(m5::pin_name_t::mbus_pin26); // ADC DIN (A/B same)
    Serial.printf("[AUDIO-SRC] CoreS3 pins SDA=%d SCL=%d BCK=%d MCK=%d WS=%d DIN=%d\n",
                  sda, scl, bck, mck, ws, s_i2s_din);

    // CODEC-only begin (5-arg: I2C + ES8388 init + STM32 check; does NOT install
    // I2S — i2sRxInit() below owns the I2S so we can use small DMA buffers).
    // Settle after M5.begin() (AXP2101) and retry; the ES8388 can miss I2C right
    // after power-up.
    delay(100);
    s_codecOk = false;
    for (int attempt = 0; attempt < 5 && !s_codecOk; ++attempt) {
        s_codecOk = s_audio.begin(Wire, (uint8_t)sda, (uint8_t)scl, 0x33, 400000);
        if (!s_codecOk) {
            Serial.printf("[AUDIO-SRC] ES8388 begin failed, retry %d/5\n", attempt + 1);
            delay(120);
        }
    }
    if (!s_codecOk) {
        Serial.println("[AUDIO-SRC] Module Audio (ES8388/STM32) NOT found "
                       "(check switch=A, module seated)");
        displayError("MODULE AUDIO NOT FOUND");
    } else {
        s_audio.setMICStatus(AUDIO_MIC_OPEN);                 // STM32: open input
        s_audio.setMicInputLine(ADC_INPUT_LINPUT1_RINPUT1);   // TRS line-in (both ch)
        s_audio.setMicGain(pgaFromLevel(s_input_level));      // analog PGA from input_level
        s_audio.setMicAdcVolume(100);                         // digital ADC vol = 0 dB (max)
        s_audio.setBitsSample(ES_MODULE_ADC, BIT_LENGTH_16BITS);
        s_audio.setSampleRate(SAMPLE_RATE_48K);               // codec ADC @ 48 kHz
        // Disable ALC — at the CORRECT register. The M5 driver enables stereo ALC
        // at max gain (ADCCONTROL10 = reg 0x12 = 0xEA). Line-in does NOT want auto-
        // leveling: on a quiet/line input the ALC ramps the PGA to max gain, which
        // AMPLIFIES the ADC's DC offset + noise (measured raw idle ~ -19000/-24800 =
        // huge DC, with R>L because ALC is per-channel) and pumps on sustained tones.
        // Set ALCSEL (bits[7:6]) = 00 to turn ALC OFF so the fixed PGA (setMicGain)
        // applies. IMPORTANT: the ALC is at 0x12 (ADCCONTROL10); 0x0E (ADCCONTROL6)
        // is the ADC HIGH-PASS FILTER. The previous code wrote 0x0E=0x00 believing
        // it was the ALC register — that never disabled the ALC (leaving the DC
        // offset / R clipping) and poked the HPF instead. Confirmed via es8388.hpp
        // (#define ES8388_ADCCONTROL10 0x12) + ES8388 datasheet.
        Wire.beginTransmission(0x10);   // ES8388 I2C address
        Wire.write(0x12); Wire.write(0x00);   // ADCCONTROL10: ALCSEL=00 (ALC off)
        Wire.endTransmission();
        // NOTE: the L/R balance trim is applied DIGITALLY in audioRxTask (R_TRIM_PCT),
        // not via an ES8388 register. The ADC digital-volume register (ADCCONTROL9 =
        // 0x11) was measured to NOT attenuate the R capture (write took, ratio
        // unchanged), so a firmware-domain trim on the DC-blocked signal is used
        // instead — reliable, and it also balances the on-screen level meter.
        Serial.printf("[AUDIO-SRC] ES8388 ready (CoreS3, switch A), PGA idx=%d, ALC off (0x12=0)\n",
                      (int)pgaFromLevel(s_input_level));
    }

    firInit();
    adpcmStateInit(&s_sm);   // mono ADPCM encoder state (mode 6 LITE); s_sl/s_sr done above
    // Raw RX-only I2S with small DMA buffers (switch-A SCLK/MCLK pins).
    i2sRxInit(mck, bck, ws, s_i2s_din);

    // Dedicated capture task, pinned to core 1 (WiFi/ESP-NOW runs on core 0).
    xTaskCreatePinnedToCore(audioRxTask, "audioRx", 8192, nullptr, 18, nullptr, 1);
#else
    Wire.begin(I2C_SDA, I2C_SCL);
    es8388Init();
    i2sRxInit(I2S_MCLK, I2S_BCLK, I2S_LRCK, I2S_DIN);
#endif

    Serial.printf("[AUDIO-SRC] ch=%u capture=%u Hz -> stream=%u Hz, "
                  "input_level=%d, max_inflight=%d\n",
                  s_channel, CAPTURE_RATE, STREAM_RATE,
                  s_input_level, STREAM_MAX_INFLIGHT);
}

#ifdef BOARD_CORES3
// ---- Hierarchical touch UI: HOME → MODE selector / CHANNEL selector --------
// HOME shows the current mode + channel + live stats and two buttons. Tapping a
// button drills into a full-screen selector; tapping the header returns. Mode /
// channel changes reboot into the new setting (see audioSourceSetMode).
enum { UI_HOME = 0, UI_FAMILY = 1, UI_MODE = 2, UI_CH = 3, UI_VOLMAX = 4 };
static uint8_t s_ui      = UI_HOME;
static bool    s_uiDirty = true;              // force a full repaint on screen change
static const uint16_t UI_SEL_BG = 0x0208;     // dark-cyan fill behind a selected item

static void uiBtn(int x, int y, int w, int h, const char* s, uint16_t bd, uint16_t tx, bool sel) {
    auto& d = M5.Display;
    if (sel) d.fillRoundRect(x, y, w, h, 8, UI_SEL_BG);
    d.drawRoundRect(x, y, w, h, 8, sel ? TFT_CYAN : bd);
    d.setTextSize(2);
    d.setTextColor(tx, sel ? UI_SEL_BG : TFT_BLACK);
    d.setTextDatum(textdatum_t::middle_center);
    d.drawString(s, x + w / 2, y + h / 2);
    d.setTextDatum(textdatum_t::top_left);
}

// ---- Two-level mode picker: family (ADPCM / OPUS) -> mode within it --------
// The wire mode_id is unchanged; this only groups the on-screen selector so the
// 3x2 grid never needs a 4th row (ADPCM has 3 modes, OPUS has 6 — both fit the 3×2 grid).
// SOLID48 (mode 9) is its own family (FAM_AUDIO) rather than a 7th OPUS tile:
// OPUS already fills all 6 cells of the 3×2 MODE-select grid (FAST/BALANCED/
// SMOOTH/FINE/STEREO/HIFI), so a 7th entry would render off-screen (row 3 at
// y=234 > the 240 px panel height). A 1-entry family fits the grid trivially.
enum { FAM_ADPCM = 0, FAM_OPUS = 1, FAM_LR = 2, FAM_AUDIO = 3, FAM_COUNT = 4 };
static const char* const FAM_NAME[FAM_COUNT] = { "ADPCM", "OPUS", "LONG RANGE", "HIFI+" };
// Hints describe what the family optimises for — NOT latency/quality, which are
// per-tile axes within a family (user 2026-07-10): ADPCM has SOLID=30 ms while
// Opus FAST=12 ms, and the top sensory quality is SOLID/ADPCM. ADPCM = simple,
// zero encode-CPU, proven (but higher airtime + silence-fill loss); OPUS =
// bandwidth-efficient (multi-source) + PLC (loss concealment); LR = distance;
// HIFI+ = full-band 48 kHz stereo, the highest-quality mode (SOLID48).
static const char* const FAM_HINT[FAM_COUNT] = { "simple / proven", "efficient / PLC", "distance", "full-band, max quality" };
static const uint8_t FAM_ADPCM_MODES[] = { MODE_TURBO, MODE_MONO8K, MODE_ADPCM };      // TURBO, LITE, SOLID (low-latency -> robust)
static const uint8_t FAM_OPUS_MODES[]  = { MODE_L, MODE_M, MODE_Q, MODE_FINE, MODE_STEREO, MODE_HIFI }; // FAST, BALANCED, SMOOTH, FINE, STEREO, HIFI (low-latency -> high-quality)
static const uint8_t FAM_AUDIO_MODES[] = { MODE_SOLID48 };   // SOLID48 — Opus 48k stereo, highest quality
struct FamilyDef { const uint8_t* modes; uint8_t count; };
static const FamilyDef FAMILY[FAM_COUNT] = {
    { FAM_ADPCM_MODES, 3 },
    { FAM_OPUS_MODES,  6 },
    { nullptr,         LR_PRESET_COUNT },   // LONG RANGE grid renders LR_PRESET[], not modes
    { FAM_AUDIO_MODES, 1 },
};
// Reverse-map the wire mode → family for highlighting. RANGE=LONG borrows wire
// id3 but its family is LONG RANGE, so s_range_long wins.
static inline uint8_t familyOf(uint8_t mode) {
    if (s_range_long) return FAM_LR;
    if (mode == MODE_SOLID48) return FAM_AUDIO;
    return (mode == MODE_ADPCM || mode == MODE_MONO8K || mode == MODE_TURBO) ? FAM_ADPCM : FAM_OPUS;
}
static uint8_t s_uiFamily = FAM_ADPCM;   // family the MODE grid is currently browsing
// SELECT-TYPE (UI_FAMILY) grid geometry — generalized from the old hardcoded
// 3-cell layout (x = 6 + f*104, w = 98) so a 4th (or Nth) family still fits the
// ~320 px panel width: for FAM_COUNT=3 this reproduces the exact prior pixel
// values (98 W / 104 pitch); for FAM_COUNT=4 it yields 72 W / 78 pitch.
// Shared by render (uiRepaint) + touch hit-test (uiTouch).
static const int UI_FAM_W = (308 - (FAM_COUNT - 1) * 6) / FAM_COUNT;   // cell width
static const int UI_FAM_P = UI_FAM_W + 6;                               // pitch (x stride)

// MODE-select 3×2 grid geometry — row-major, i = row*2 + col (e.g. the OPUS
// family fills FAST|BALANCED / SMOOTH|FINE / STEREO|HIFI). Larger cells than the old 6-row list to stop
// mis-taps. 3 rows fit the 240 px height (y0 42 + 2×64 + 58 = 228 < 240).
// Shared by render (uiRepaint) + touch hit-test (uiTouch).
static const int UI_MODE_X0    = 6;      // left edge of column 0
static const int UI_MODE_COLP  = 156;    // column pitch (x stride)
static const int UI_MODE_COLW  = 150;    // cell width
static const int UI_MODE_Y0    = 42;     // top edge of row 0
static const int UI_MODE_ROWP  = 64;     // row pitch (y stride)
static const int UI_MODE_CELLH = 58;     // cell height

// HOME input-level meter geometry (developer diagnostic, right of the mode info,
// in the blank area above the CHANNEL button). x kept ≥166 so the widest
// MODE_DESC ("ADPCM 4ms buf") never reaches the bars. Two rows: L (upper),
// R (lower); frame drawn in uiRepaint, fill/readout in uiUpdateHome.
static const int MTR_X   = 166;   // left edge of the meter column
static const int MTR_W   = 148;   // bar/frame width (166..314)
static const int MTR_H   = 10;    // bar height
static const int MTR_LTX = 56;    // L numeric-readout text y
static const int MTR_LBY = 68;    // L bar frame y
static const int MTR_RTX = 90;    // R numeric-readout text y
static const int MTR_RBY = 102;   // R bar frame y

// Draw one input-level meter row: bar fill (proportional to peak/full-scale) +
// numeric %/CLIP. peak = |sample| 0..32768 (full scale). Color green <70% /
// yellow 70..94% / red ≥95%; a bright-red "CLIP" replaces the % at peak ≥32000
// (ADC near ±full-scale — the key "back off the source" signal). Interior fill
// only, so the 1px static frame from uiRepaint stays intact.
static void uiMeterRow(int barY, int txtY, char tag, uint16_t peak, uint16_t rawPeak) {
    auto& d = M5.Display;
    uint32_t pct = (uint32_t)peak * 100u / 32768u;
    if (pct > 100) pct = 100;
    int fill = (int)((uint32_t)peak * (uint32_t)(MTR_W - 2) / 32768u);
    if (fill > MTR_W - 2) fill = MTR_W - 2;
    uint16_t col = (pct >= 95) ? TFT_RED : (pct >= 70) ? TFT_YELLOW : TFT_GREEN;
    d.fillRect(MTR_X + 1, barY + 1, MTR_W - 2, MTR_H - 2, TFT_BLACK);
    if (fill > 0) d.fillRect(MTR_X + 1, barY + 1, fill, MTR_H - 2, col);
    // CLIP judged on the RAW ADC (pre-trim) — the true saturation point. R can
    // hit the ADC rail while its trimmed bar reads well under 100%.
    bool clip = (rawPeak >= 32000);
    char rd[16];
    if (clip) snprintf(rd, sizeof(rd), "%c CLIP ", tag);
    else      snprintf(rd, sizeof(rd), "%c %3lu%% ", tag, (unsigned long)pct);
    d.setTextSize(1);
    d.setTextColor(clip ? TFT_RED : col, TFT_BLACK);
    d.setTextDatum(textdatum_t::top_left);
    d.drawString(rd, MTR_X, txtY);
}

// Repaint the whole current screen (static parts).
static void uiRepaint() {
    auto& d = M5.Display;
    d.fillScreen(TFT_BLACK);
    uint8_t m = s_mode < MODE_COUNT ? s_mode : 0;
    if (s_ui == UI_HOME) {
        d.setTextSize(2); d.setTextColor(TFT_CYAN, TFT_BLACK);
        d.setTextDatum(textdatum_t::top_left);  d.drawString("HAPBEAT SRC", 6, 6);
        char hd[16]; snprintf(hd, sizeof(hd), "ch%u", s_channel);
        d.setTextColor(TFT_DARKGREY, TFT_BLACK);
        d.setTextDatum(textdatum_t::top_right);  d.drawString(hd, 314, 6);
        d.setTextDatum(textdatum_t::top_left);
        // RANGE badge (display-only now — RANGE is chosen in MODE > SELECT TYPE >
        // LONG RANGE; the badge just reflects the current state).
        {
            char rb[28];
            if (s_range_long) snprintf(rb, sizeof(rb), "RANGE: LONG (%s)", LR_PRESET[s_lr_preset].name);
            else              snprintf(rb, sizeof(rb), "RANGE: NORMAL");
            d.setTextSize(1); d.setTextColor(s_range_long ? TFT_YELLOW : TFT_DARKGREY, TFT_BLACK);
            d.drawString(rb, 8, 30);
        }
        d.setTextSize(1); d.setTextColor(TFT_DARKGREY, TFT_BLACK); d.drawString("MODE", 8, 44);
        // LONG borrows wire id3 but it is an LR preset — show "LR" + preset name +
        // format, not "SMOOTH", so the operator never confuses it with the mode.
        d.setTextSize(3); d.setTextColor(s_codecOk ? TFT_WHITE : TFT_RED, TFT_BLACK);
        d.drawString(s_range_long ? "LR" : MODE_NAME[m], 8, 58);
        d.setTextSize(2); d.setTextColor(TFT_DARKGREY, TFT_BLACK);
        d.drawString(s_range_long ? LR_PRESET[s_lr_preset].name : MODE_DESC[m], 8, 96);
        d.setTextSize(2); d.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        if (s_range_long) { char fb[24]; snprintf(fb, sizeof(fb), "8k mono 10ms %uk", LR_PRESET[s_lr_preset].bitrate / 1000); d.drawString(fb, 8, 120); }
        else              d.drawString(MODE_FMT[m], 8, 120);
        // Input-level meter frame + header (static; bars/readouts in uiUpdateHome).
        d.setTextSize(1); d.setTextColor(TFT_DARKGREY, TFT_BLACK);
        d.setTextDatum(textdatum_t::top_left); d.drawString("INPUT PEAK", MTR_X, 44);
        d.drawRect(MTR_X, MTR_LBY, MTR_W, MTR_H, TFT_DARKGREY);
        d.drawRect(MTR_X, MTR_RBY, MTR_W, MTR_H, TFT_DARKGREY);
        // pkt/s + drop: static labels here; only the numbers are redrawn (fixed
        // 3/4-digit width) in uiUpdateHome, so the labels never shift.
        d.setTextSize(2); d.setTextColor(TFT_DARKGREY, TFT_BLACK);
        d.setTextDatum(textdatum_t::top_left);
        d.drawString("pkt/s", 48, 210);
        d.drawString("drop", 122, 210);
        // Main drill-in buttons — unchanged full-size MODE / CHANNEL.
        uiBtn(10, 148, 140, 48, "MODE >", TFT_DARKCYAN, TFT_CYAN, false);
        uiBtn(170, 148, 140, 48, "CHANNEL >", TFT_DARKCYAN, TFT_CYAN, false);
        // RX VOL (receiver volume-max, §3.5 p5) — a small, LOW-KEY button tucked
        // into the bottom-right corner (dim grey, half height) so it doesn't
        // compete with MODE/CHANNEL (user 2026-07-11). Bottom-right is free: the
        // meter is top-right, the pkt/s·drop stats are bottom-left.
        uiBtn(232, 206, 82, 30, "RX VOL", TFT_DARKGREY, TFT_DARKCYAN, false);
    } else if (s_ui == UI_FAMILY) {
        d.setTextSize(2); d.setTextColor(TFT_CYAN, TFT_BLACK);
        d.setTextDatum(textdatum_t::top_left); d.drawString("< SELECT TYPE", 6, 6);
        uint8_t curFam = familyOf(m);
        // FAM_COUNT cells (ADPCM / OPUS / LONG RANGE / HIFI+) sized by
        // UI_FAM_W/UI_FAM_P so they always fit the panel width; the two-word
        // "LONG RANGE" name wraps to 2 lines.
        for (int f = 0; f < FAM_COUNT; f++) {
            int x = 6 + f * UI_FAM_P, y = 54, w = UI_FAM_W, h = 150;
            int mx = x + w / 2, my = y + h / 2;
            bool cur = (f == curFam);
            uint16_t bg = cur ? UI_SEL_BG : TFT_BLACK;
            if (cur) d.fillRoundRect(x, y, w, h, 8, UI_SEL_BG);
            d.drawRoundRect(x, y, w, h, 8, cur ? TFT_CYAN : TFT_DARKCYAN);
            d.setTextDatum(textdatum_t::middle_center);
            d.setTextSize(2); d.setTextColor(cur ? TFT_WHITE : TFT_CYAN, bg);
            const char* nm = FAM_NAME[f];
            const char* sp = strchr(nm, ' ');
            if (sp) {  // two-word name (LONG RANGE) → 2 lines
                char w1[12]; size_t n1 = (size_t)(sp - nm); if (n1 > 11) n1 = 11;
                memcpy(w1, nm, n1); w1[n1] = 0;
                d.drawString(w1, mx, my - 26);
                d.drawString(sp + 1, mx, my - 6);
            } else {
                d.drawString(nm, mx, my - 14);
            }
            d.setTextSize(1); d.setTextColor(TFT_DARKGREY, bg);
            d.drawString(FAM_HINT[f], mx, my + 24);
        }
        d.setTextDatum(textdatum_t::top_left);
    } else if (s_ui == UI_MODE) {
        d.setTextSize(2); d.setTextColor(TFT_CYAN, TFT_BLACK);
        d.setTextDatum(textdatum_t::top_left);
        char hdr[24]; snprintf(hdr, sizeof(hdr), "< %s", FAM_NAME[s_uiFamily]);
        d.drawString(hdr, 6, 6);
        // The LONG RANGE family renders LR presets (bitrate + 2SRC/1SRC); the
        // others render wire modes. Same 3×2 geometry.
        bool isLr  = (s_uiFamily == FAM_LR);
        int  count = isLr ? LR_PRESET_COUNT : FAMILY[s_uiFamily].count;
        for (int i = 0; i < count; i++) {
            int col = i & 1, row = i >> 1;
            int x = UI_MODE_X0 + col * UI_MODE_COLP;
            int y = UI_MODE_Y0 + row * UI_MODE_ROWP;
            const char* nm; char desc[24]; bool cur;
            if (isLr) {
                nm  = LR_PRESET[i].name;
                cur = (s_range_long && i == s_lr_preset);
                snprintf(desc, sizeof(desc), "%uk  %s", LR_PRESET[i].bitrate / 1000,
                         LR_PRESET[i].two_src ? "2SRC" : "1SRC");
            } else {
                uint8_t mode = FAMILY[s_uiFamily].modes[i];
                nm  = MODE_NAME[mode];
                cur = (!s_range_long && mode == m);
                strncpy(desc, MODE_DESC[mode], sizeof(desc)); desc[sizeof(desc) - 1] = 0;
            }
            uint16_t bg = cur ? UI_SEL_BG : TFT_BLACK;
            if (cur) d.fillRoundRect(x, y, UI_MODE_COLW, UI_MODE_CELLH, 6, UI_SEL_BG);
            d.drawRoundRect(x, y, UI_MODE_COLW, UI_MODE_CELLH, 6, cur ? TFT_CYAN : TFT_DARKGREY);
            d.setTextDatum(textdatum_t::top_left);
            d.setTextSize(2); d.setTextColor(cur ? TFT_WHITE : TFT_LIGHTGREY, bg);
            d.drawString(nm, x + 10, y + 8);
            // 2SRC presets shown green (operator judges 2-source viability on screen).
            d.setTextSize(1);
            d.setTextColor(isLr && LR_PRESET[i].two_src ? TFT_GREEN : TFT_DARKGREY, bg);
            d.drawString(desc, x + 10, y + 38);
        }
        d.setTextDatum(textdatum_t::top_left);
    } else if (s_ui == UI_CH) {
        d.setTextSize(2); d.setTextColor(TFT_CYAN, TFT_BLACK);
        d.setTextDatum(textdatum_t::top_left); d.drawString("< SELECT CHANNEL", 6, 6);
        d.setTextSize(1); d.setTextColor(TFT_DARKGREY, TFT_BLACK);
        d.drawString("avoid 2.4GHz interference", 8, 40);
        const uint8_t chs[3] = {1, 6, 11};
        for (int i = 0; i < 3; i++) {
            char lbl[8]; snprintf(lbl, sizeof(lbl), "CH%u", chs[i]);
            uiBtn(12 + i * 102, 90, 92, 62, lbl, TFT_DARKCYAN, TFT_CYAN, chs[i] == s_channel);
        }
        d.setTextSize(1); d.setTextColor(TFT_YELLOW, TFT_BLACK);
        d.setTextDatum(textdatum_t::top_left); d.drawString("applies on select (reboot)", 8, 172);
    } else {  // UI_VOLMAX — receiver volume ceiling (§3.5 param 5), fleet-wide
        d.setTextSize(2); d.setTextColor(TFT_CYAN, TFT_BLACK);
        d.setTextDatum(textdatum_t::top_left); d.drawString("< RX VOL MAX", 6, 6);
        d.setTextSize(1); d.setTextColor(TFT_DARKGREY, TFT_BLACK);
        d.drawString("all receivers: lower % = finer resolution", 8, 30);
        // Highlight the current fleet value; before any selection show 100%
        // (the receivers' default when nothing is broadcast).
        uint8_t cur = s_fleet_set[5] ? s_fleet_val[5] : 100;
        for (int i = 0; i < 6; i++) {
            int col = i & 1, row = i >> 1;
            int x = UI_MODE_X0 + col * UI_MODE_COLP;
            int y = UI_MODE_Y0 + row * UI_MODE_ROWP;
            bool sel = (VOLMAX_PCTS[i] == cur);
            char lbl[8]; snprintf(lbl, sizeof(lbl), "%u%%", VOLMAX_PCTS[i]);
            uint16_t bg = sel ? UI_SEL_BG : TFT_BLACK;
            if (sel) d.fillRoundRect(x, y, UI_MODE_COLW, UI_MODE_CELLH, 6, UI_SEL_BG);
            d.drawRoundRect(x, y, UI_MODE_COLW, UI_MODE_CELLH, 6, sel ? TFT_CYAN : TFT_DARKGREY);
            d.setTextDatum(textdatum_t::middle_center);
            d.setTextSize(3); d.setTextColor(sel ? TFT_WHITE : TFT_LIGHTGREY, bg);
            d.drawString(lbl, x + UI_MODE_COLW / 2, y + UI_MODE_CELLH / 2);
        }
        d.setTextDatum(textdatum_t::top_left);
    }
    s_uiDirty = false;
}

// HOME live stats (pkt/s + drop) — opaque overwrite, no full repaint.
static void uiUpdateHome() {
    if (s_ui != UI_HOME) return;
    static uint32_t last = 0, lastPkt = 0;
    uint32_t now = millis();
    if (now - last < 400) return;
    uint32_t dt = now - last; last = now;
    uint32_t pps = dt ? (uint32_t)((uint64_t)(s_txPktSent - lastPkt) * 1000 / dt) : 0;
    lastPkt = s_txPktSent;
    auto& d = M5.Display;
    d.setTextSize(2); d.setTextDatum(textdatum_t::top_left);
    // Redraw ONLY the numbers (fixed width, opaque) — the "pkt/s"/"drop" labels are
    // static (uiRepaint), so nothing shifts as the digit count changes.
    char nb[8];
    if (pps > 999) pps = 999;
    snprintf(nb, sizeof(nb), "%3lu", (unsigned long)pps);
    d.setTextColor(TFT_GREEN, TFT_BLACK); d.drawString(nb, 8, 210);
    uint32_t drp = s_txDroppedBusy > 9999 ? 9999 : s_txDroppedBusy;
    snprintf(nb, sizeof(nb), "%4lu", (unsigned long)drp);
    d.setTextColor(drp ? TFT_YELLOW : TFT_DARKGREY, TFT_BLACK); d.drawString(nb, 176, 210);

    // Input-level meter: snapshot + reset the peak trackers (last ~0.4 s) and
    // render the L/R bars + %/CLIP. Reset so each window reflects recent audio.
    uint16_t lpk = s_uiLpk, rpk = s_uiRpk;
    uint16_t lraw = s_rawLpk, rraw = s_rawRpk;
    s_uiLpk = 0; s_uiRpk = 0; s_rawLpk = 0; s_rawRpk = 0;
    uiMeterRow(MTR_LBY, MTR_LTX, 'L', lpk, lraw);
    uiMeterRow(MTR_RBY, MTR_RTX, 'R', rpk, rraw);
}

// Dispatch one touch-release at (x,y) based on the current screen.
static void uiTouch(int x, int y) {
    if (s_ui == UI_HOME) {
        // Main buttons (y148-196): MODE (x<160) / CHANNEL (else). RANGE is chosen
        // via MODE > SELECT TYPE > LONG RANGE (badge display-only).
        if (y >= 148 && y <= 196) { s_ui = (x < 160) ? UI_FAMILY : UI_CH; s_uiDirty = true; }
        // Small RX VOL button, bottom-right corner (below the main button row).
        else if (y >= 200 && x >= 220) { s_ui = UI_VOLMAX; s_uiDirty = true; }
    } else if (s_ui == UI_FAMILY) {
        if (y < 36) { s_ui = UI_HOME; s_uiDirty = true; return; }   // header = back to HOME
        if (y >= 54 && y <= 204) {                                  // FAM_COUNT family cells
            for (int f = 0; f < FAM_COUNT; f++) {
                int cx0 = 6 + f * UI_FAM_P;
                if (x >= cx0 && x < cx0 + UI_FAM_W) { s_uiFamily = (uint8_t)f; s_ui = UI_MODE; s_uiDirty = true; break; }
            }
        }
    } else if (s_ui == UI_MODE) {
        if (y < 36) { s_ui = UI_FAMILY; s_uiDirty = true; return; } // header = back to family
        if (x < UI_MODE_X0) return;                                 // left of the grid
        int col = (x - UI_MODE_X0) / UI_MODE_COLP;
        int row = (y - UI_MODE_Y0) / UI_MODE_ROWP;
        if (col < 0 || col > 1 || row < 0) return;
        // Reject taps that land in the gap band between cells (outside the cell rect).
        int cx = x - (UI_MODE_X0 + col * UI_MODE_COLP);
        int cy = y - (UI_MODE_Y0 + row * UI_MODE_ROWP);
        if (cx >= UI_MODE_COLW || cy < 0 || cy >= UI_MODE_CELLH) return;
        int i = row * 2 + col;
        if (s_uiFamily == FAM_LR) {
            if (i >= 0 && i < LR_PRESET_COUNT) audioSourceSetLrPreset(i);   // saves NVS + reboots
        } else {
            const FamilyDef& fam = FAMILY[s_uiFamily];
            if (i >= 0 && i < fam.count) audioSourceSetMode(fam.modes[i]);  // saves NVS + reboots
        }
    } else if (s_ui == UI_CH) {
        if (y < 36) { s_ui = UI_HOME; s_uiDirty = true; return; }
        if (y >= 90 && y <= 152) {
            const uint8_t chs[3] = {1, 6, 11};
            int i = (x - 12) / 102;
            if (i >= 0 && i < 3 && chs[i] != s_channel) {
                Preferences p; p.begin("espnow", false); p.putUChar("channel", chs[i]); p.end();
                Serial.printf("[AUDIO-SRC] channel -> %u, rebooting\n", chs[i]);
                delay(150); ESP.restart();
            }
        }
    } else {  // UI_VOLMAX — tap a % tile → 0xAC param 5 to the whole fleet
        if (y < 36) { s_ui = UI_HOME; s_uiDirty = true; return; }   // header = back
        if (x < UI_MODE_X0) return;
        int col = (x - UI_MODE_X0) / UI_MODE_COLP;
        int row = (y - UI_MODE_Y0) / UI_MODE_ROWP;
        if (col < 0 || col > 1 || row < 0) return;
        int cx = x - (UI_MODE_X0 + col * UI_MODE_COLP);
        int cy = y - (UI_MODE_Y0 + row * UI_MODE_ROWP);
        if (cx >= UI_MODE_COLW || cy < 0 || cy >= UI_MODE_CELLH) return;
        int i = row * 2 + col;
        if (i >= 0 && i < 6) {
            audioSourceSetFleetParam(5, VOLMAX_PCTS[i]);   // 0xAC burst ×3 + 5 s resend
            s_ui = UI_HOME; s_uiDirty = true;               // runtime beacon — no reboot
        }
    }
}
#endif

// ---------------------------------------------------------------------------
// ---- Fleet-tune (0xAC beacon, §3.5) — send side ---------------------------
// Broadcast one 0xAC beacon. Mirrors the audio path's s_inflight accounting so
// the shared send-callback (audioOnSendCb) stays balanced.
static void sendFleetBeacon(uint8_t param, uint8_t value, uint8_t epoch) {
    // [4] = epoch (P1.1, §3.5 epoch sync) — lets a receiving TX/receiver tell a
    // newer value from a stale duplicate. 5 B total (was 4 B pre-epoch).
    uint8_t pkt[5] = { 0xAC, 0x01, param, value, epoch };   // [1] = version 1, RELAYED=0
    s_inflight++;
    if (esp_now_send(BROADCAST_MAC, pkt, sizeof(pkt)) != ESP_OK) s_inflight--;
}

// Resend the set fleet params: burst ×3 on a change, then a single refresh every
// 5 s (late-booting receivers catch up). Called from audioSourceLoop (loopTask).
static void fleetTick() {
    // Drain params adopted from a peer TX (audioOnRecvCb, WiFi task) — the NVS
    // write + log line are deferred here (loop task) so the recv callback stays
    // free of blocking I/O (self-review F2 fix). `&= ~pending` (not `= 0`) so a
    // bit the callback sets DURING this drain survives to the next tick instead
    // of being silently cleared.
    uint8_t pending = s_fleet_persist_pending;
    if (pending) {
        s_fleet_persist_pending &= (uint8_t)~pending;
        for (uint8_t pid = 1; pid <= 5; pid++) {
            if (!(pending & (1 << (pid - 1)))) continue;
            persistFleetParam(pid);
            Serial.printf("[AUDIO-SRC] fleet param %u adopted from peer TX (val=%u epoch=%u)\n",
                          pid, s_fleet_val[pid], s_fleet_epoch[pid]);
        }
    }
    static uint32_t s_last = 0;
    uint32_t now = millis();
    bool burst = s_fleet_dirty;
    if (!burst && (now - s_last) < 5000) return;
    s_fleet_dirty = false;
    s_last = now;
    for (int r = 0; r < (burst ? 3 : 1); r++)
        for (uint8_t pid = 1; pid <= 5; pid++)
            if (s_fleet_set[pid]) sendFleetBeacon(pid, s_fleet_val[pid], s_fleet_epoch[pid]);
}

// Public (serial set_fleet_param / RX VOL touch): 1=buffer_ms 2=selection
// 3=lock_timeout(×10ms) 4=resync_gap 5=volume_max(%). Stores the value, bumps
// this param's LOCAL epoch (P1.1 — every operator-initiated change is a new
// epoch, whether or not the value differs from what's already broadcasting, so
// a peer TX always converges to the freshest operator intent), persists both
// (P1.4 — reboot-safe), and triggers a ×3 burst; then resent every 5 s.
void audioSourceSetFleetParam(int param, int value) {
    if (param < 1 || param > 5 || value < 0 || value > 255) return;
    s_fleet_val[param]   = (uint8_t)value;
    s_fleet_epoch[param] = (uint8_t)(s_fleet_epoch[param] + 1);   // wraps by design (wrap-safe compare)
    s_fleet_set[param]   = true;
    s_fleet_dirty        = true;
    persistFleetParam(param);
    Serial.printf("[AUDIO-SRC] fleet param %d = %d (epoch %u, burst + 5s resend)\n",
                  param, value, s_fleet_epoch[param]);
}

// Public (serial set_range_mode): persist RANGE (keeps the current LR preset) +
// reboot into it (CoreS3-only — the LR profile is Opus).
void audioSourceSetRange(bool long_range) {
#ifndef BOARD_CORES3
    (void)long_range;
    Serial.println("[AUDIO-SRC] set_range_mode ignored — LONGRANGE needs CoreS3 (Opus)");
#else
    Preferences p; p.begin("tx", false);
    p.putBool("range_long", long_range);
    p.end();
    Serial.printf("[AUDIO-SRC] RANGE -> %s (preset %s), rebooting\n",
                  long_range ? "LONG" : "NORMAL", LR_PRESET[s_lr_preset].name);
    delay(150);
    ESP.restart();
#endif
}

// Public (serial set_lr_preset / MODE > LONG RANGE tile): enter LONGRANGE with an
// LR preset (0..5) + persist + reboot. CoreS3-only (Opus).
void audioSourceSetLrPreset(int preset) {
#ifndef BOARD_CORES3
    (void)preset;
    Serial.println("[AUDIO-SRC] set_lr_preset ignored — LONGRANGE needs CoreS3 (Opus)");
#else
    if (preset < 0 || preset >= LR_PRESET_COUNT) return;
    if (s_range_long && (uint8_t)preset == s_lr_preset) return;   // no change
    Preferences p; p.begin("tx", false);
    p.putBool("range_long", true);
    p.putUChar("lr_preset", (uint8_t)preset);
    p.end();
    Serial.printf("[AUDIO-SRC] LR preset -> %s, rebooting\n", LR_PRESET[preset].name);
    delay(150);
    ESP.restart();
#endif
}

// Deprecated (serial set_lr_bitrate): compat alias mapping a raw bitrate to its
// pb1 preset. set_lr_preset is canonical.
void audioSourceSetLrBitrate(int bitrate) {
    audioSourceSetLrPreset(bitrate == 48000 ? 4 : bitrate == 32000 ? 2 : 0);  // HQ / STD / ECO
}

bool        audioSourceGetRange()     { return s_range_long; }
int         audioSourceGetLrBitrate() { return LR_PRESET[s_lr_preset].bitrate; }
const char* audioSourceGetLrPreset()  { return LR_PRESET[s_lr_preset].name; }

void audioSourceLoop() {
    heartbeatTick();
    fleetTick();

#ifdef BOARD_CORES3
    // Hierarchical touch UI (HOME → MODE / CHANNEL). Capture/encode/send run in
    // audioRxTask; here we only service touch + refresh the LCD.
    M5.update();
    if (M5.Touch.getCount() > 0) {
        auto t = M5.Touch.getDetail(0);
        if (t.wasReleased()) uiTouch(t.x, t.y);
    }
    if (s_uiDirty) uiRepaint();
    uiUpdateHome();
    delay(20);
#else
    drainI2sEvents();   // tally RX overflow / DMA errors (classic capture path)
    static int16_t cap[CAP_FRAMES * 2];   // interleaved L,R @ 48 kHz
    static int16_t pcm[FRAMES_PKT * 2];   // interleaved L,R @ 16 kHz

    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_NUM_0, cap, sizeof(cap),
                              &bytes_read, pdMS_TO_TICKS(50));
    if (err != ESP_OK || bytes_read < sizeof(cap)) { s_capFail++; return; }

    for (int o = 0; o < FRAMES_PKT; o++) {
        int32_t accL = 0, accR = 0;
        for (int k = 0; k < DECIMATE; k++) {
            int idx = (o * DECIMATE + k) * 2;
            accL += cap[idx];
            accR += cap[idx + 1];
        }
        rawStat(cap[o * DECIMATE * 2], cap[o * DECIMATE * 2 + 1]);
        int32_t l = (accL / DECIMATE) * s_input_level / 50;
        int32_t r = (accR / DECIMATE) * s_input_level / 50;
        if (l > 32767)  l = 32767;  else if (l < -32768) l = -32768;
        if (r > 32767)  r = 32767;  else if (r < -32768) r = -32768;
        pcm[o * 2]     = (int16_t)l;
        pcm[o * 2 + 1] = (int16_t)r;
    }

    streamEncodeAndSend(pcm);
#endif
}

// ---------------------------------------------------------------------------
// Live input-level update (Studio set_input_level). On CoreS3 this drives the
// ES8388 analog PGA gain (better SNR than digital); on classic it scales the
// decimated PCM. 50 = unity / 0 dB.
void audioSourceApplyInputLevel(int level) {
    if (level < 0) level = 0;
    if (level > 100) level = 100;
    s_input_level = level;
#ifdef BOARD_CORES3
    if (s_codecOk) s_audio.setMicGain(pgaFromLevel(level));
#endif
}

// Live stream-mode select (0=SOLID/ADPCM, 1=FAST, 2=BALANCED, 3=SMOOTH,
// 4=STEREO, 5=HIFI, 6=LITE/ADPCM-mono, 7=TURBO/ADPCM-mono, 8=FINE/Opus-16k-mono,
// 9=SOLID48/Opus-48k-stereo).
// Persisted to NVS,
// then reboots into the mode (a clean boot per mode is proven stable; a live
// re-init crashed the encoder). On the classic (non-CoreS3) sender only mode 0
// is available — Opus needs the CoreS3, and the LITE (6) / TURBO (7) mono
// decimate/encode path also lives in the CoreS3-only audioRxTask (the classic
// loop always ships mode 0).
void audioSourceSetMode(int mode) {
    if (mode < 0 || mode >= MODE_COUNT) return;
#ifndef BOARD_CORES3
    if (mode != MODE_ADPCM) return;   // classic sender has no Opus encoder
#endif
    // Picking an ADPCM/OPUS mode also EXITS LONGRANGE (§1). Skip only when the mode
    // is unchanged AND we're already in NORMAL.
    if ((uint8_t)mode == s_mode && !s_range_long) return;
    Preferences p;
    p.begin("tx", false);
    p.putUChar("mode", (uint8_t)mode);
    p.putBool("range_long", false);   // exit LR back to NORMAL (6M)
    p.end();
    // Reboot into the new mode instead of live-switching. Re-initializing the
    // Opus encoder for a new frame size mid-stream is unreliable on this codec
    // build (crashes); a clean boot into each mode is proven stable. The ~2 s
    // reboot is fine for a deliberate mode change (setup / A-B sensory test).
    Serial.printf("[AUDIO-SRC] stream mode -> %s (%d), rebooting\n",
                  MODE_NAME[mode], mode);
    delay(150);       // flush serial + let the NVS commit settle
    ESP.restart();
}

int audioSourceGetMode() { return s_mode; }

#endif // AUDIO_SOURCE
