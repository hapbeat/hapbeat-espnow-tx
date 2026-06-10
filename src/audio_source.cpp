// ---------------------------------------------------------------------------
// audio_source.cpp — ESP-NOW live audio source (see header). #ifdef AUDIO_SOURCE.
//
// NOTE (hardware bring-up): the ES8388 register sequence and the I2S pin map
// below are the M5Stack Core + Module-Audio reference values; confirm/tune
// them against the actual board + line level. The capture → decimate → ADPCM →
// broadcast pipeline itself is board-agnostic.
// ---------------------------------------------------------------------------

#include "audio_source.h"

#ifdef AUDIO_SOURCE

#include "config.h"        // BROADCAST_MAC
#include "ima_adpcm.h"

#include <Arduino.h>
#include <Wire.h>
#include <driver/i2s.h>
#include <esp_now.h>
#include <Preferences.h>

// ---- audio params (must match the receiver) -------------------------------
static const uint32_t CAPTURE_RATE = 48000;
static const uint32_t STREAM_RATE  = 16000;
static const int      DECIMATE     = CAPTURE_RATE / STREAM_RATE;   // 3
static const int      FRAMES_PKT   = 16;                            // 16 kHz frames / packet
static const int      CAP_FRAMES   = FRAMES_PKT * DECIMATE;         // 48 input frames / packet
static const uint8_t  STREAM_TYPE  = 0xAA;

// ---- ES8388 (M5 Module-Audio) — I2C + I2S pins ----------------------------
static const uint8_t  ES8388_ADDR = 0x10;
static const int      I2C_SDA = 21, I2C_SCL = 22;      // M5 Core internal I2C
static const int      I2S_MCLK = 0, I2S_BCLK = 13, I2S_LRCK = 12, I2S_DIN = 34;

// ---- runtime config (NVS) -------------------------------------------------
static int s_input_level = 50;   // 0..100, 50 = unity

// ---- ADPCM encoder state (continuous across packets) ----------------------
static AdpcmState s_sl, s_sr;
static uint8_t    s_seq = 0;

static void es8388Write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ES8388_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Minimal ES8388 line-in → ADC → I2S record config. Register values follow the
// common ES8388 record recipe; tune on hardware (input select / ADC gain).
static void es8388Init() {
    es8388Write(0x00, 0x80); delay(10);   // reset
    es8388Write(0x00, 0x00);
    es8388Write(0x01, 0x50);              // power management
    es8388Write(0x02, 0x00);              // power up
    es8388Write(0x03, 0x00);              // ADC power on
    es8388Write(0x04, 0xFC);              // DAC power down (record-only)
    es8388Write(0x08, 0x00);              // I2S master/slave (slave; ESP32 is master)
    es8388Write(0x09, 0x88);              // ADC: line-in (LINPUT1/RINPUT1) + 0 dB
    es8388Write(0x0A, 0x00);              // input select
    es8388Write(0x0B, 0x02);
    es8388Write(0x0C, 0x0C);              // ADC I2S: 16-bit, standard I2S
    es8388Write(0x0D, 0x02);              // ADC MCLK/LRCK ratio
    es8388Write(0x10, 0x00);              // ADC L volume 0 dB
    es8388Write(0x11, 0x00);              // ADC R volume 0 dB
}

static void i2sRxInit() {
    i2s_config_t cfg = {};
    cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    cfg.sample_rate = CAPTURE_RATE;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;   // stereo
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    cfg.dma_buf_count = 4;
    cfg.dma_buf_len = 256;
    cfg.use_apll = true;
    cfg.tx_desc_auto_clear = false;
    cfg.fixed_mclk = 0;

    i2s_pin_config_t pins = {};
    pins.mck_io_num = I2S_MCLK;
    pins.bck_io_num = I2S_BCLK;
    pins.ws_io_num = I2S_LRCK;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num = I2S_DIN;

    i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
    i2s_set_pin(I2S_NUM_0, &pins);
    i2s_set_clk(I2S_NUM_0, CAPTURE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

void audioSourceSetup() {
    Serial.println("[AUDIO-SRC] role=transmitter boot (line-in -> ESP-NOW)");
    Preferences p;
    p.begin("tx", true);
    s_input_level = p.getInt("input_level", 50);
    p.end();

    Wire.begin(I2C_SDA, I2C_SCL);
    es8388Init();
    i2sRxInit();
    adpcmStateInit(&s_sl);
    adpcmStateInit(&s_sr);
    Serial.printf("[AUDIO-SRC] capture=%u Hz -> stream=%u Hz, input_level=%d\n",
                  CAPTURE_RATE, STREAM_RATE, s_input_level);
}

// Read one packet's worth of audio, decimate, encode, and broadcast.
void audioSourceLoop() {
    static int16_t cap[CAP_FRAMES * 2];   // interleaved L,R @ 48 kHz
    static int16_t pcm[FRAMES_PKT * 2];   // interleaved L,R @ 16 kHz

    size_t bytes_read = 0;
    esp_err_t err = i2s_read(I2S_NUM_0, cap, sizeof(cap), &bytes_read, pdMS_TO_TICKS(50));
    if (err != ESP_OK || bytes_read < sizeof(cap)) return;

    // Box-filter decimate 48 -> 16 kHz (average each group of DECIMATE frames)
    // and apply input_level gain.
    for (int o = 0; o < FRAMES_PKT; o++) {
        int32_t accL = 0, accR = 0;
        for (int k = 0; k < DECIMATE; k++) {
            int idx = (o * DECIMATE + k) * 2;
            accL += cap[idx];
            accR += cap[idx + 1];
        }
        int32_t l = (accL / DECIMATE) * s_input_level / 50;
        int32_t r = (accR / DECIMATE) * s_input_level / 50;
        if (l > 32767) l = 32767; else if (l < -32768) l = -32768;
        if (r > 32767) r = 32767; else if (r < -32768) r = -32768;
        pcm[o * 2]     = (int16_t)l;
        pcm[o * 2 + 1] = (int16_t)r;
    }

    // Build 0xAA stream packet. Header carries the encoder state BEFORE this
    // block so the receiver re-syncs each packet (single-packet-loss tolerant).
    uint8_t pkt[10 + FRAMES_PKT];
    AdpcmState pre_l = s_sl, pre_r = s_sr;
    pkt[0] = STREAM_TYPE;
    pkt[1] = s_seq++;
    pkt[2] = (uint8_t)(FRAMES_PKT & 0xFF);
    pkt[3] = (uint8_t)(FRAMES_PKT >> 8);
    pkt[4] = (uint8_t)(pre_l.predictor & 0xFF);
    pkt[5] = (uint8_t)((pre_l.predictor >> 8) & 0xFF);
    pkt[6] = pre_l.step_index;
    pkt[7] = (uint8_t)(pre_r.predictor & 0xFF);
    pkt[8] = (uint8_t)((pre_r.predictor >> 8) & 0xFF);
    pkt[9] = pre_r.step_index;
    adpcmEncodeBlockStereo(pcm, pkt + 10, FRAMES_PKT, &s_sl, &s_sr);

    esp_now_send(BROADCAST_MAC, pkt, sizeof(pkt));
}

#endif // AUDIO_SOURCE
