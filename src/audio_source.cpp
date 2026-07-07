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
//   * Raw driver/i2s.h with SMALL DMA buffers (3 x 32) read in tiny chunks,
//     to minimise capture/DMA latency (large buffers add ~10 ms).
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
// espnow_stream.cpp MODE_DEFS. See docs/instructions-espnow-modes-button-ui-*.
//   0 = ADPCM 16k stereo 64-frame(4ms)  pb1   compat / floor (classic + CoreS3)
//   1 = OPUS  8k mono     40-smp(5ms)    pb1   L  low-latency
//   2 = OPUS  8k mono     40-smp(5ms)    pb1   M  middle
//   3 = OPUS  8k mono     80-smp(10ms)   pb2   Q  quality
enum StreamMode : uint8_t { MODE_ADPCM = 0, MODE_L = 1, MODE_M = 2, MODE_Q = 3, MODE_COUNT = 4 };
struct ModeDef { uint8_t codec; uint16_t opus_frame; uint8_t piggyback; uint16_t bitrate; };
// codec: 0 = ADPCM, 1 = OPUS. opus_frame = samples/frame at STREAM_OPUS_RATE.
static const ModeDef MODE_DEFS[MODE_COUNT] = {
    { 0,  0, 1,     0 },   // 0 ADPCM
    { 1, 40, 1, 16000 },   // 1 L  5 ms
    { 1, 40, 1, 16000 },   // 2 M  5 ms
    { 1, 80, 2, 16000 },   // 3 Q 10 ms
};
// User-facing names (RAW=ADPCM baseline, FAST/BALANCED/SMOOTH=Opus low/mid/robust)
static const char*   MODE_NAME[MODE_COUNT] = { "RAW", "FAST", "BALANCED", "SMOOTH" };
static const char*   MODE_DESC[MODE_COUNT] = { "ADPCM 30ms", "Opus 10ms",
                                               "Opus 15ms", "Opus 28ms" };
// Audio format per mode (rate + channels). ADPCM carries the full 16 kHz stereo
// capture; the Opus modes downmix to 8 kHz mono (v3 is haptic-only, so the
// narrower band + mono is the accepted trade for PLC robustness / low airtime).
static const char*   MODE_FMT[MODE_COUNT]  = { "16kHz Stereo", "8kHz Mono",
                                               "8kHz Mono", "8kHz Mono" };
static const uint16_t STREAM_OPUS_RATE     = 8000;   // Opus encode rate (mono)
static const int      OPUS_MAX_LEN         = 120;    // max opus frame bytes (headroom)
static const int      OPUS_MAX_SMP         = 80;     // max samples/frame (10 ms @8k)

// Live-selectable current mode (NVS "tx"/"mode", default 0 = ADPCM known-good).
static volatile uint8_t s_mode = MODE_ADPCM;

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

// ---- raw-input signal stats (raw L & R, pre-decimation) for the heartbeat --
static          int32_t  s_lmin = 32767, s_lmax = -32768;
static          int64_t  s_lsum = 0;
static          int32_t  s_rmin = 32767, s_rmax = -32768;
static          int64_t  s_rsum = 0;
static          uint32_t s_scnt = 0;
static          uint32_t s_capFail = 0;
static          bool     s_codecOk = true;   // CoreS3: ES8388/STM32 begin() result

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
}

// Periodic serial heartbeat (~2 s) for headless verification.
static void heartbeatTick() {
    static uint32_t s_last_hb = 0;
    uint32_t now = millis();
    if (now - s_last_hb < 2000) return;
    s_last_hb = now;
    long lmean = s_scnt ? (long)(s_lsum / (int64_t)s_scnt) : 0;
    long rmean = s_scnt ? (long)(s_rsum / (int64_t)s_scnt) : 0;
    Serial.printf("[AUDIO-SRC] hb codec=%d pkt=%u capfail=%u drop=%u fail=%u "
                  "L[%ld..%ld m%ld] R[%ld..%ld m%ld]\n",
                  s_codecOk ? 1 : 0,
                  s_txPktSent, s_capFail, s_txDroppedBusy, (uint32_t)s_txSendFail,
                  (long)s_lmin, (long)s_lmax, lmean,
                  (long)s_rmin, (long)s_rmax, rmean);
    s_lmin = 32767; s_lmax = -32768; s_lsum = 0;
    s_rmin = 32767; s_rmax = -32768; s_rsum = 0;
    s_scnt = 0;
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
    cfg.dma_buf_count        = 3;    // small buffers → low capture latency
    cfg.dma_buf_len          = 32;   // tiny read chunk (~0.67 ms @ 48 kHz)
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

    i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
    i2s_set_pin(I2S_NUM_0, &pins);
    i2s_set_clk(I2S_NUM_0, CAPTURE_RATE, I2S_BITS_PER_SAMPLE_16BIT,
                I2S_CHANNEL_STEREO);
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
static int16_t      s_opusAccum[OPUS_MAX_SMP];  // 8 kHz mono frame accumulator
static int          s_opusAccumN = 0;
static int          s_decim2     = 0;           // 16k→8k 2:1 decimation phase
static int32_t      s_decim2prev = 0;           // 2-tap average state
// Opus frame history for piggyback (hist[0]=most recent prev, hist[1]=older).
static uint8_t      s_opHist[2][OPUS_MAX_LEN];
static uint8_t      s_opHistLen[2] = {0, 0};
static uint8_t      s_opHistSeq[2] = {0, 0};
static int          s_opHistN      = 0;         // valid history entries (0..2)

// (Re)configure the Opus encoder for `mode` (1-3). Encoder state lives in
// INTERNAL SRAM (PSRAM wait states would ~2× the encode time — see gate①).
static void opusEncoderConfig(uint8_t mode) {
    if (mode < MODE_L || mode >= MODE_COUNT) return;
    // Pre-seed libopus's global scratch (see extern above) — internal SRAM for
    // fast encode, PSRAM fallback if no 64 KB contiguous internal block remains.
    if (!global_stack) {
        global_stack = (char*)heap_caps_malloc(64 * 1024,
                             MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!global_stack)
            global_stack = (char*)heap_caps_malloc(128 * 1024, MALLOC_CAP_SPIRAM);
        Serial.printf("[AUDIO-SRC] opus scratch @ %p (largest internal free %u)\n",
                      (void*)global_stack,
                      (unsigned)heap_caps_get_largest_free_block(
                          MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (!s_enc) {
        // Create ONCE; reused for all Opus modes. Opus takes the frame size as a
        // per-encode-call argument, so L/M/Q (40/40/80 samples, same bitrate)
        // share one encoder — no per-switch reconfigure. (A RESET_STATE + re-ctl
        // on the live switch was crashing the encoder mid-stream.)
        int sz = opus_encoder_get_size(1);
        s_enc = (OpusEncoder*)heap_caps_malloc((size_t)sz,
                                               MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_enc) { Serial.println("[AUDIO-SRC] opus enc alloc FAILED"); return; }
        opus_encoder_init(s_enc, STREAM_OPUS_RATE, 1, OPUS_APPLICATION_RESTRICTED_LOWDELAY);
        opus_encoder_ctl(s_enc, OPUS_SET_BITRATE(MODE_DEFS[mode].bitrate));
        opus_encoder_ctl(s_enc, OPUS_SET_COMPLEXITY(0));
        opus_encoder_ctl(s_enc, OPUS_SET_VBR(0));
    }
    s_enc_mode   = mode;   // switches opusPush8k's frame size to this mode
    s_opusAccumN = 0;      // drop the partial frame from the old size
    s_opHistN    = 0;      // old-size piggyback frames are invalid now
    s_decim2     = 0;
    Serial.printf("[AUDIO-SRC] opus enc mode=%s (%u Hz mono, %u smp, %u bps)\n",
                  MODE_NAME[mode], STREAM_OPUS_RATE, MODE_DEFS[mode].opus_frame,
                  MODE_DEFS[mode].bitrate);
}

// Assemble + broadcast one Opus 0xAA packet (mode 1-3) with piggyback.
// Body: [4]=opus_len(1) [5..]=frame ; then pb× [prev_seq(1)][len(1)][data].
static void sendOpusPacket(const uint8_t* frame, int flen) {
    uint8_t mode = s_mode;
    int depth = MODE_DEFS[mode].piggyback;
    int pbn   = s_opHistN < depth ? s_opHistN : depth;

    if (s_inflight >= STREAM_MAX_INFLIGHT) {
        s_txDroppedBusy++;
    } else {
        uint8_t pkt[4 + 1 + OPUS_MAX_LEN + 2 * (2 + OPUS_MAX_LEN)];
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
        s_inflight++;
        esp_err_t e = esp_now_send(BROADCAST_MAC, pkt, p);
        if (e != ESP_OK) { s_inflight--; s_txSendFail++; }
        else s_txPktSent++;
    }
    // Record current frame into history (shift) + advance seq — done even on a
    // busy-drop so the NEXT packet's piggyback can still recover this frame.
    memcpy(s_opHist[1], s_opHist[0], s_opHistLen[0]);
    s_opHistLen[1] = s_opHistLen[0]; s_opHistSeq[1] = s_opHistSeq[0];
    memcpy(s_opHist[0], frame, (size_t)flen);
    s_opHistLen[0] = (uint8_t)flen;  s_opHistSeq[0] = s_seq;
    if (s_opHistN < 2) s_opHistN++;
    s_seq++;
}

// Accumulate one 8 kHz mono sample; encode + send when a full frame is ready.
static inline void opusPush8k(int16_t s) {
    if (!s_enc || s_enc_mode < MODE_L) return;
    s_opusAccum[s_opusAccumN++] = s;
    int need = MODE_DEFS[s_enc_mode].opus_frame;
    if (s_opusAccumN >= need) {
        uint8_t buf[OPUS_MAX_LEN];
        int n = opus_encode(s_enc, s_opusAccum, need, buf, sizeof(buf));
        s_opusAccumN = 0;
        if (n > 0) sendOpusPacket(buf, n);
    }
}

// ---- CoreS3 capture task: raw i2s_read (small chunks) → FIR decimate → send -
static void audioRxTask(void* /*arg*/) {
    static int16_t accum[FRAMES_PKT * 2];   // 16 kHz stereo, one ADPCM packet
    int accumIdx = 0;
    uint32_t dcount = 0;
    uint8_t taskMode = 0xFF;                 // detect live mode switches
    static int16_t rx[FRAMES_PKT * 2];      // tiny read chunk (16 stereo frames)
    for (;;) {
        size_t br = 0;
        esp_err_t e = i2s_read(I2S_NUM_0, rx, sizeof(rx), &br, portMAX_DELAY);
        if (e != ESP_OK || br < 4) { s_capFail++; vTaskDelay(1); continue; }
        size_t n = br / sizeof(int16_t);
        for (size_t i = 0; i + 1 < n; i += 2) {
            // L/R swapped vs the raw I2S slot order (receiver was reversed).
            int16_t L = rx[i + 1];
            int16_t R = rx[i];
            rawStat(L, R);
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
                if (l > 32767) l = 32767; else if (l < -32768) l = -32768;
                if (r > 32767) r = 32767; else if (r < -32768) r = -32768;

                // Route by the (live-switchable) stream mode.
                uint8_t mode = s_mode;
                if (mode != taskMode) {
                    accumIdx = 0;
                    if (mode >= MODE_L) opusEncoderConfig(mode);
                    taskMode = mode;
                }
                if (mode == MODE_ADPCM) {
                    accum[accumIdx * 2]     = (int16_t)l;
                    accum[accumIdx * 2 + 1] = (int16_t)r;
                    if (++accumIdx >= FRAMES_PKT) { streamEncodeAndSend(accum); accumIdx = 0; }
                } else {
                    // downmix + 2:1 decimate 16k stereo → 8k mono (2-tap average).
                    int32_t mono16 = ((int32_t)l + r) / 2;
                    if (s_decim2 == 0) { s_decim2prev = mono16; s_decim2 = 1; }
                    else { s_decim2 = 0; opusPush8k((int16_t)((s_decim2prev + mono16) / 2)); }
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

    {
        Preferences p;
        p.begin("espnow", true);
        s_channel = p.getUChar("channel", 1);
        p.end();
        p.begin("tx", true);
        s_input_level = p.getInt("input_level", 50);
        s_mode        = p.getUChar("mode", MODE_ADPCM);
        if (s_mode >= MODE_COUNT) s_mode = MODE_ADPCM;
        p.end();
    }

    // EspNowSender::init() (main.cpp) already did WiFi STA + radio tuning
    // (6 Mbps, PS off) + esp_now_init. Replace its send callback with ours so
    // we can track in-flight depth.
    esp_now_register_send_cb(audioOnSendCb);

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
        // Disable ALC. The M5 driver's ES8388 init() enables stereo ALC "for
        // VOICE" (reg 0x0E=0xEA). Line-in does NOT want auto-leveling — ALC rides
        // the gain and pumps on sustained tones (adds noise) and can skew L/R.
        // reg 0x0E bits[7:6]=00 = ALC off; fixed PGA (setMicGain) then applies.
        Wire.beginTransmission(0x10);   // ES8388 I2C address
        Wire.write(0x0E); Wire.write(0x00);
        Wire.endTransmission();
        Serial.printf("[AUDIO-SRC] ES8388 ready (CoreS3, switch A), PGA idx=%d, ALC off\n",
                      (int)pgaFromLevel(s_input_level));
    }

    firInit();
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
enum { UI_HOME = 0, UI_MODE = 1, UI_CH = 2 };
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
        d.setTextSize(1); d.setTextColor(TFT_DARKGREY, TFT_BLACK); d.drawString("MODE", 8, 44);
        d.setTextSize(3); d.setTextColor(s_codecOk ? TFT_WHITE : TFT_RED, TFT_BLACK);
        d.drawString(MODE_NAME[m], 8, 58);
        d.setTextSize(2); d.setTextColor(TFT_DARKGREY, TFT_BLACK); d.drawString(MODE_DESC[m], 8, 96);
        d.setTextSize(2); d.setTextColor(TFT_LIGHTGREY, TFT_BLACK); d.drawString(MODE_FMT[m], 8, 120);
        uiBtn(10, 148, 140, 48, "MODE >", TFT_DARKCYAN, TFT_CYAN, false);
        uiBtn(170, 148, 140, 48, "CHANNEL >", TFT_DARKCYAN, TFT_CYAN, false);
    } else if (s_ui == UI_MODE) {
        d.setTextSize(2); d.setTextColor(TFT_CYAN, TFT_BLACK);
        d.setTextDatum(textdatum_t::top_left); d.drawString("< SELECT MODE", 6, 6);
        for (int i = 0; i < MODE_COUNT; i++) {
            int y = 42 + i * 47; bool cur = (i == m);
            if (cur) d.fillRoundRect(6, y, 308, 42, 6, UI_SEL_BG);
            d.drawRoundRect(6, y, 308, 42, 6, cur ? TFT_CYAN : TFT_DARKGREY);
            d.setTextSize(2); d.setTextColor(cur ? TFT_WHITE : TFT_LIGHTGREY, cur ? UI_SEL_BG : TFT_BLACK);
            d.setTextDatum(textdatum_t::middle_left);  d.drawString(MODE_NAME[i], 16, y + 21);
            d.setTextColor(TFT_DARKGREY, cur ? UI_SEL_BG : TFT_BLACK);
            d.setTextDatum(textdatum_t::middle_right); d.drawString(MODE_DESC[i], 302, y + 21);
        }
        d.setTextDatum(textdatum_t::top_left);
    } else {  // UI_CH
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
    d.setTextColor(s_txDroppedBusy ? TFT_YELLOW : TFT_GREEN, TFT_BLACK);
    char st[32]; snprintf(st, sizeof(st), "%lu pkt/s drop %lu    ",
                          (unsigned long)pps, (unsigned long)s_txDroppedBusy);
    d.drawString(st, 8, 210);
}

// Dispatch one touch-release at (x,y) based on the current screen.
static void uiTouch(int x, int y) {
    if (s_ui == UI_HOME) {
        if (y >= 148 && y <= 196) { s_ui = (x < 160) ? UI_MODE : UI_CH; s_uiDirty = true; }
    } else if (s_ui == UI_MODE) {
        if (y < 36) { s_ui = UI_HOME; s_uiDirty = true; return; }   // header = back
        int i = (y - 42) / 47;
        if (i >= 0 && i < MODE_COUNT) audioSourceSetMode(i);        // saves NVS + reboots
    } else {  // UI_CH
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
    }
}
#endif

// ---------------------------------------------------------------------------
void audioSourceLoop() {
    heartbeatTick();

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

// Live stream-mode select (0=ADPCM, 1=L, 2=M, 3=Q). Persisted to NVS. On the
// classic (non-CoreS3) sender only ADPCM (0) is available; Opus modes are
// accepted but the classic loop still emits ADPCM (no encoder). The CoreS3
// capture task reconfigures the Opus encoder on the next block.
void audioSourceSetMode(int mode) {
    if (mode < 0 || mode >= MODE_COUNT) return;
#ifndef BOARD_CORES3
    if (mode != MODE_ADPCM) return;   // classic sender has no Opus encoder
#endif
    if ((uint8_t)mode == s_mode) return;   // no change
    Preferences p;
    p.begin("tx", false);
    p.putUChar("mode", (uint8_t)mode);
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
