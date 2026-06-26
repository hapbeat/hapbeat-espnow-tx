// ---------------------------------------------------------------------------
// audio_source.cpp — ESP-NOW live audio source (see header). #ifdef AUDIO_SOURCE.
//
// Captures PA line-in via an I2S codec (M5 Module-Audio / ES8388), decimates
// 48 kHz → 16 kHz, IMA-ADPCM-encodes, and broadcasts 0xAA stream packets
// over ESP-NOW. See contracts/specs/espnow-stream.md.
//
// DEC-033 additions (2026-06-26):
//  - Piggyback (§3.2): each packet carries the previous packet's data so
//    the receiver can recover single-packet losses without silence.
//  - In-flight control: STREAM_MAX_INFLIGHT=2 prevents piling up of
//    unacknowledged sends; excess packets are dropped (encoder state still
//    advances to keep the next packet valid).
//  - WiFi.setSleep(false): eliminates modem-sleep jitter from Tx timing.
//  - Minimal display stats updated at ~4 fps.
//
// NOTE (hardware bring-up): ES8388 register sequence and I2S pin map below
// are M5Stack Core + Module-Audio reference values. Confirm/tune against the
// actual board and line level before use.
// ---------------------------------------------------------------------------

#include "audio_source.h"

#ifdef AUDIO_SOURCE

#include "config.h"        // BROADCAST_MAC
#include "display.h"
#include "ima_adpcm.h"

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include <esp_now.h>
#include <Preferences.h>
#include <cstring>

// ---- audio params (must match the receiver) --------------------------------
static const uint32_t CAPTURE_RATE       = 48000;
static const uint32_t STREAM_RATE        = 16000;
static const int      DECIMATE           = CAPTURE_RATE / STREAM_RATE;  // 3
static const int      FRAMES_PKT         = 16;
static const int      CAP_FRAMES         = FRAMES_PKT * DECIMATE;       // 48
static const uint8_t  STREAM_TYPE        = 0xAA;
static const int      STREAM_MAX_INFLIGHT = 2;

// Piggyback size: prev_seq(1) + prev_state(6) + prev_data(FRAMES_PKT) = 7+N
static const int PIGGYBACK_HDR = 7;

// ---- display update interval -----------------------------------------------
static const uint32_t DISPLAY_INTERVAL_MS = 250;  // ~4 fps

// ---- ES8388 (M5 Module-Audio) — I2C + I2S pins ----------------------------
static const uint8_t  ES8388_ADDR = 0x10;
static const int      I2C_SDA = 21, I2C_SCL = 22;
static const int      I2S_MCLK = 0, I2S_BCLK = 13, I2S_LRCK = 12, I2S_DIN = 34;

// ---- runtime config (NVS) -------------------------------------------------
static int     s_input_level = 50;   // 0..100, 50 = unity
static uint8_t s_channel     = 1;    // ESP-NOW channel (for display)

// ---- ADPCM encoder state (continuous across packets) ----------------------
static AdpcmState s_sl, s_sr;
static uint8_t    s_seq = 0;

// ---- in-flight control (shared with send callback in WiFi task) -----------
// WiFi task is non-ISR but runs on a different FreeRTOS task; volatile is
// sufficient for the simple counter. The worst-case race (incrementing just
// as callback fires) causes at most one extra in-flight slot — acceptable.
static volatile int      s_inflight      = 0;
static volatile uint32_t s_txSendFail    = 0;
static          uint32_t s_txDroppedBusy = 0;
static          uint32_t s_txPktSent     = 0;

// ---- piggyback state (saved from the last successfully enqueued packet) ---
static bool       s_has_prev      = false;
static uint8_t    s_prev_seq;
static AdpcmState s_prev_sl, s_prev_sr;   // encoder state BEFORE that block
static uint8_t    s_prev_data[FRAMES_PKT];

// ---------------------------------------------------------------------------
// Send callback — registered in audioSourceSetup(), replaces EspNowSender's.
// Called from WiFi task after each esp_now_send() completes (success or fail).
// ---------------------------------------------------------------------------
static void audioOnSendCb(const uint8_t* /*mac*/, esp_now_send_status_t status) {
    if (s_inflight > 0) s_inflight--;
    if (status != ESP_NOW_SEND_SUCCESS) s_txSendFail++;
}

// ---- ES8388 helpers --------------------------------------------------------
static void es8388Write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ES8388_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Minimal ES8388 line-in → ADC → I2S record config. Register values follow
// the common ES8388 record recipe; tune on hardware (input select / gain).
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

static void i2sRxInit() {
    i2s_config_t cfg = {};
    cfg.mode              = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    cfg.sample_rate       = CAPTURE_RATE;
    cfg.bits_per_sample   = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format    = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags  = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count     = 4;
    cfg.dma_buf_len       = 256;
    cfg.use_apll          = true;
    cfg.tx_desc_auto_clear = false;
    cfg.fixed_mclk        = 0;

    i2s_pin_config_t pins = {};
    pins.mck_io_num    = I2S_MCLK;
    pins.bck_io_num    = I2S_BCLK;
    pins.ws_io_num     = I2S_LRCK;
    pins.data_out_num  = I2S_PIN_NO_CHANGE;
    pins.data_in_num   = I2S_DIN;

    i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
    i2s_set_pin(I2S_NUM_0, &pins);
    i2s_set_clk(I2S_NUM_0, CAPTURE_RATE, I2S_BITS_PER_SAMPLE_16BIT,
                I2S_CHANNEL_STEREO);
}

// ---------------------------------------------------------------------------
void audioSourceSetup() {
    // Load runtime config from NVS
    {
        Preferences p;
        p.begin("espnow", true);
        s_channel = p.getUChar("channel", 1);
        p.end();
        p.begin("tx", true);
        s_input_level = p.getInt("input_level", 50);
        p.end();
    }

    // Disable modem sleep to eliminate Tx timing jitter (contracts §4 / DEC-033).
    WiFi.setSleep(false);

    // Replace EspNowSender's send callback with ours to track in-flight count.
    // EspNowSender.init() was already called in main.cpp setup() before us.
    esp_now_register_send_cb(audioOnSendCb);

    Wire.begin(I2C_SDA, I2C_SCL);
    es8388Init();
    i2sRxInit();
    adpcmStateInit(&s_sl);
    adpcmStateInit(&s_sr);

    Serial.printf("[AUDIO-SRC] ch=%u capture=%u Hz -> stream=%u Hz, "
                  "input_level=%d, max_inflight=%d\n",
                  s_channel, CAPTURE_RATE, STREAM_RATE,
                  s_input_level, STREAM_MAX_INFLIGHT);
}

// ---------------------------------------------------------------------------
// Read one packet's worth of audio, decimate, encode, and broadcast.
// ---------------------------------------------------------------------------
void audioSourceLoop() {
    static int16_t cap[CAP_FRAMES * 2];   // interleaved L,R @ 48 kHz
    static int16_t pcm[FRAMES_PKT * 2];   // interleaved L,R @ 16 kHz

    // Read a packet's worth of raw audio from DMA
    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_NUM_0, cap, sizeof(cap),
                              &bytes_read, pdMS_TO_TICKS(50));
    if (err != ESP_OK || bytes_read < sizeof(cap)) return;

    // Box-filter decimate 48→16 kHz and apply input_level gain.
    for (int o = 0; o < FRAMES_PKT; o++) {
        int32_t accL = 0, accR = 0;
        for (int k = 0; k < DECIMATE; k++) {
            int idx = (o * DECIMATE + k) * 2;
            accL += cap[idx];
            accR += cap[idx + 1];
        }
        int32_t l = (accL / DECIMATE) * s_input_level / 50;
        int32_t r = (accR / DECIMATE) * s_input_level / 50;
        if (l > 32767)  l = 32767;  else if (l < -32768) l = -32768;
        if (r > 32767)  r = 32767;  else if (r < -32768) r = -32768;
        pcm[o * 2]     = (int16_t)l;
        pcm[o * 2 + 1] = (int16_t)r;
    }

    // Snapshot encoder state BEFORE this block — this is what goes into the
    // packet header so the receiver can re-sync from each packet independently.
    AdpcmState pre_l = s_sl, pre_r = s_sr;

    // Encode (must advance encoder state even when we decide to drop the packet
    // so that the next packet's header reflects the correct predictor state).
    uint8_t data[FRAMES_PKT];
    adpcmEncodeBlockStereo(pcm, data, FRAMES_PKT, &s_sl, &s_sr);

    // In-flight control: drop if too many unacknowledged sends outstanding.
    // Still increment seq so the receiver sees a gap (handled as packet loss)
    // rather than corruption.
    if (s_inflight >= STREAM_MAX_INFLIGHT) {
        s_txDroppedBusy++;
        s_seq++;
        return;
    }

    // Build STREAM packet with optional piggyback (contracts espnow-stream.md §3).
    // Max size: header(10) + data(N) + piggyback_hdr(7) + piggyback_data(N) = 17+2N
    // For N=16: 49 bytes — well within the 250-byte ESP-NOW limit.
    uint8_t pkt[10 + FRAMES_PKT + PIGGYBACK_HDR + FRAMES_PKT];
    size_t  pkt_len = 0;

    pkt[pkt_len++] = STREAM_TYPE;
    pkt[pkt_len++] = s_seq;
    pkt[pkt_len++] = (uint8_t)(FRAMES_PKT & 0xFF);
    pkt[pkt_len++] = (uint8_t)(FRAMES_PKT >> 8);
    pkt[pkt_len++] = (uint8_t)(pre_l.predictor & 0xFF);
    pkt[pkt_len++] = (uint8_t)((pre_l.predictor >> 8) & 0xFF);
    pkt[pkt_len++] = pre_l.step_index;
    pkt[pkt_len++] = (uint8_t)(pre_r.predictor & 0xFF);
    pkt[pkt_len++] = (uint8_t)((pre_r.predictor >> 8) & 0xFF);
    pkt[pkt_len++] = pre_r.step_index;
    memcpy(pkt + pkt_len, data, FRAMES_PKT);
    pkt_len += FRAMES_PKT;

    // Piggyback §3.2: append previous packet so receiver can recover single losses.
    if (s_has_prev) {
        pkt[pkt_len++] = s_prev_seq;
        // prev_state: L predictor (2B LE), L step (1B), R predictor (2B LE), R step (1B)
        pkt[pkt_len++] = (uint8_t)(s_prev_sl.predictor & 0xFF);
        pkt[pkt_len++] = (uint8_t)((s_prev_sl.predictor >> 8) & 0xFF);
        pkt[pkt_len++] = s_prev_sl.step_index;
        pkt[pkt_len++] = (uint8_t)(s_prev_sr.predictor & 0xFF);
        pkt[pkt_len++] = (uint8_t)((s_prev_sr.predictor >> 8) & 0xFF);
        pkt[pkt_len++] = s_prev_sr.step_index;
        memcpy(pkt + pkt_len, s_prev_data, FRAMES_PKT);
        pkt_len += FRAMES_PKT;
    }

    // Reserve in-flight slot before sending (decrement in callback if send fails).
    s_inflight++;
    esp_err_t send_err = esp_now_send(BROADCAST_MAC, pkt, pkt_len);
    if (send_err != ESP_OK) {
        // esp_now_send failed synchronously — callback will NOT fire, so release slot.
        s_inflight--;
        s_txSendFail++;
    } else {
        // Packet enqueued: save state for next packet's piggyback.
        s_txPktSent++;
        s_has_prev   = true;
        s_prev_seq   = s_seq;
        s_prev_sl    = pre_l;
        s_prev_sr    = pre_r;
        memcpy(s_prev_data, data, FRAMES_PKT);
    }
    s_seq++;

    // Periodic display update (~4 fps).
    static uint32_t s_last_display = 0;
    uint32_t now = millis();
    if (now - s_last_display >= DISPLAY_INTERVAL_MS) {
        s_last_display = now;
        displayUpdateAudioStats(s_channel, s_input_level, s_txPktSent,
                                s_inflight, s_txDroppedBusy,
                                (uint32_t)s_txSendFail);
    }
}

#endif // AUDIO_SOURCE
