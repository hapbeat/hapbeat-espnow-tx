// ---------------------------------------------------------------------------
// repeater.cpp — ESP-NOW repeater (see header). #ifdef REPEATER.
//
// Receives 0xAA stream packets and re-broadcasts them with the RELAYED flag
// (byte[1] bit7) set — the ONLY byte changed (DEC-043 §7.2). The re-broadcast's
// ESP-NOW source MAC becomes *this device's* MAC, so receivers see the repeater
// as an independent source and select between direct + repeater by delivery rate.
//
// Loop-prevention (structural): a packet that already has RELAYED set is never
// relayed → "a relayed packet is never relayed again" bounds depth at 1 hop
// (origin → repeater → receiver) for ANY number of repeaters, with zero on-site
// config. Origin selection is either:
//   - auto origin-follow (default): lock the first RELAYED=0 MAC; release after
//     R_TIMEOUT_MS of silence and re-lock the next origin → the second audio
//     source becomes a true hot standby.
//   - strict manual pin (NVS relay_src set): relay ONLY that MAC; if it dies,
//     relaying stops (no failover). Empty MAC restores auto-follow.
//
// R_TIMEOUT_MS (250) MUST exceed the receiver's LOCK_TIMEOUT (150 ms, spec §7.2)
// so an overlap-zone receiver migrates to the direct origin before the repeater
// re-locks — the receiver then sees a seq discontinuity and resyncs (§7.1.1).
//
// Architecture note: the ESP-NOW receive callback runs in the WiFi task — calling
// esp_now_send() there would risk re-entering the WiFi stack. The callback is a
// single producer into a 4-slot SPSC ring; repeaterLoop() (Arduino loop task) is
// the single consumer that sends. 4 slots cover the M5 LCD's 250 ms refresh.
// ---------------------------------------------------------------------------

#include "repeater.h"

#ifdef REPEATER

#include "config.h"    // BROADCAST_MAC
#ifndef BOARD_XIAO_C6
#include "display.h"   // LCD stats (M5 only — XIAO C6 is headless)
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <cstring>
#include <cstdio>
#ifdef BOARD_XIAO_C6
#include <Wire.h>
#include <U8x8lib.h>   // Seeed XIAO Expansion Board 0.96" OLED (SSD1306 128x64)
#endif

static const uint8_t  STREAM_TYPE         = 0xAA;   // audio stream packet
static const uint8_t  FLEET_TYPE          = 0xAC;   // fleet-tune beacon (§3.5, 4 B)
static const uint32_t DISPLAY_INTERVAL_MS = 250;   // ~4 fps (M5 LCD)
static const uint32_t R_TIMEOUT_MS        = 250;   // origin silence → release (MUST > receiver LOCK_TIMEOUT 150)
static const uint32_t HEARTBEAT_MS        = 2000;  // serial heartbeat cadence

// ---- Origin selection ------------------------------------------------------
// Manual pin (NVS relay_src): strict — auto-follow disabled.
static uint8_t s_relay_src[6] = {};
static bool    s_pin_active   = false;
// Auto origin-follow (when not pinned): first RELAYED=0 MAC, released after
// R_TIMEOUT_MS of silence. s_origin_last_ms is written by the callback (WiFi
// task) and read by the loop — volatile is sufficient on ESP32 (32-bit access).
static uint8_t s_origin[6] = {};
static volatile bool     s_origin_locked  = false;
static volatile uint32_t s_origin_last_ms = 0;

static uint8_t s_channel = 1;   // for display / heartbeat

// ---- stats -----------------------------------------------------------------
static volatile uint32_t s_relayed    = 0;    // packets successfully re-broadcast
static volatile uint32_t s_slot_drops = 0;    // dropped because the ring was full
static volatile int8_t   s_origin_rssi = 127; // last origin RSSI (IDF5 recv adapter); 127 = n/a
static volatile uint8_t  s_last_mode   = 0xFF; // mode_id of the last relayed 0xAA stream (0xFF = none)

// Compact mode-name table for the XIAO expansion-board OLED. Index = mode_id,
// matching the receiver STREAM_MODE_NAME / sender MODE_NAME order.
static const char* const RPT_MODE_NAME[9] = {
    "SOLID", "FAST", "BALANCED", "SMOOTH", "STEREO", "HIFI", "LITE", "TURBO", "FINE" };

// ---- 4-slot SPSC ring (lock-free, single producer = callback) --------------
static const int SLOT_COUNT = 4;
struct RelaySlot { uint8_t buf[250]; int len; };
static RelaySlot         s_slots[SLOT_COUNT];
static volatile uint32_t s_slot_head = 0;   // producer advances after writing a slot
static volatile uint32_t s_slot_tail = 0;   // consumer advances after sending a slot

// ---- helper: MAC → string --------------------------------------------------
static void macToStr(const uint8_t* mac, char* buf, size_t bufsz) {
    snprintf(buf, bufsz, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ---- relay logic (called from the WiFi task — do NOT esp_now_send here) -----
// `src` is the 6-byte sender MAC (both callback signatures resolve to this).
static void onReceiveCbBody(const uint8_t* src, const uint8_t* data, int len) {
    // Relay the audio stream (0xAA, >=5 B) AND the fleet-tune beacon (0xAC, 4 B).
    // Both carry the RELAYED flag in byte[1] bit7 (DEC-043 §3.5 / §7.2), so the
    // set-bit7 + no-relay-relayed 1-hop logic below is identical for either.
    bool is_stream = (len >= 5 && len <= 250 && data[0] == STREAM_TYPE);
    bool is_fleet  = (len == 4 && data[0] == FLEET_TYPE);
    if (!is_stream && !is_fleet) return;
    if (data[1] & 0x80) return;                    // RELAYED already set → never relay (1-hop cap)

    if (s_pin_active) {                            // strict manual pin
        if (memcmp(src, s_relay_src, 6) != 0) return;
    } else {                                       // auto origin-follow
        if (!s_origin_locked) {
            memcpy(s_origin, src, 6);
            s_origin_locked = true;                // lock the first RELAYED=0 MAC
        } else if (memcmp(src, s_origin, 6) != 0) {
            return;                                // not our locked origin
        }
        s_origin_last_ms = millis();
    }

    if (is_stream) s_last_mode = data[1] & 0x7F;   // relayed-stream mode for the OLED

    // Enqueue into the SPSC ring. Set RELAYED (bit7) on the copy — the ONLY byte
    // changed; seq/body/piggyback are verbatim (receiver piggyback matches
    // prev_seq by absolute value, so seq MUST NOT be rewritten).
    uint32_t head = s_slot_head;
    if ((head - s_slot_tail) >= (uint32_t)SLOT_COUNT) { s_slot_drops = s_slot_drops + 1; return; }
    RelaySlot& sl = s_slots[head % SLOT_COUNT];
    memcpy(sl.buf, data, (size_t)len);
    sl.buf[1] |= 0x80;
    sl.len = len;
    s_slot_head = head + 1;                         // publish
}

// arduino-esp32 3.x (IDF5 / XIAO C6) passes esp_now_recv_info_t; 2.x passes the
// bare src MAC. One adapter each → shared onReceiveCbBody.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onReceiveCb(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    s_origin_rssi = (info && info->rx_ctrl) ? info->rx_ctrl->rssi : 127;  // placement diag
    onReceiveCbBody(info->src_addr, data, len);
}
#else
static void onReceiveCb(const uint8_t* src, const uint8_t* data, int len) {
    onReceiveCbBody(src, data, len);
}
#endif

// ---- XIAO ESP32-C6 headless UI: external antenna + BOOT button + user LED ----
// GPIO per the Seeed XIAO ESP32-C6 wiki (verified 2026-07-09):
//   WIFI_ENABLE (GPIO3)      = RF switch enable — LOW activates the switch
//   WIFI_ANT_CONFIG (GPIO14) = antenna select  — HIGH = external u.FL, LOW = ceramic
//   BOOT button = GPIO9 (active-LOW), User LED = LED_BUILTIN (GPIO15)
// WIFI_ENABLE / WIFI_ANT_CONFIG / LED_BUILTIN are the core's pins_arduino.h macros.
#ifdef BOARD_XIAO_C6
static const int     XIAO_BOOT_PIN = 9;      // BOOT button (Seeed wiki), active-LOW
// User LED polarity is undocumented; assumed active-LOW (XIAO convention /
// ESPHome inverted:true). Cosmetic — a wrong guess only inverts the blink.
static const uint8_t XIAO_LED_ON  = LOW;
static const uint8_t XIAO_LED_OFF = HIGH;

static bool     s_led_on        = false;
static uint8_t  s_blink_pending = 0;         // queued channel-feedback blinks (1/2/3)
static uint32_t s_blink_at      = 0;

static inline void xiaoLedSet(bool on) {
    s_led_on = on;
    digitalWrite(LED_BUILTIN, on ? XIAO_LED_ON : XIAO_LED_OFF);
}

// Seeed XIAO Expansion Board OLED (SSD1306 128x64 @ 0x3C, I2C SDA=22 / SCL=23).
// Auto-detected at boot — a bare XIAO (no expansion board) simply skips it. Shows
// channel / origin / relayed rate / relayed-stream mode so an operator can read a
// repeater's settings at a glance (esp. that its channel matches the fleet).
static U8X8_SSD1306_128X64_NONAME_HW_I2C s_oled(U8X8_PIN_NONE, SCL, SDA);
static bool s_has_oled = false;

static void xiaoOledInit() {
    Wire.begin(SDA, SCL);
    Wire.beginTransmission(0x3C);
    s_has_oled = (Wire.endTransmission() == 0);   // ACK → expansion board present
    if (!s_has_oled) { Serial.println("[REPEATER] XIAO: no expansion OLED (0x3C)"); return; }
    s_oled.begin();
    s_oled.setFont(u8x8_font_chroma48medium8_r);
    s_oled.clear();
    s_oled.drawString(0, 0, "HAPBEAT RPT");
    Serial.println("[REPEATER] XIAO: expansion OLED found (SSD1306 0x3C)");
}

// ~2 Hz status refresh. drawString does not clear, so every field is space-padded
// to overwrite the previous value (no full clear → no flicker).
static void xiaoOledDraw() {
    if (!s_has_oled) return;
    static uint32_t s_last = 0, s_last_relayed = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - s_last) < 500) return;
    uint32_t dt = now - s_last; s_last = now;
    uint32_t pps = dt ? (uint32_t)((uint64_t)(s_relayed - s_last_relayed) * 1000 / dt) : 0;
    s_last_relayed = s_relayed;

    char line[20];
    snprintf(line, sizeof(line), "ch:%-2u  pin:%d   ", s_channel, s_pin_active ? 1 : 0);
    s_oled.drawString(0, 1, line);
    if (s_pin_active || s_origin_locked) {
        const uint8_t* o = s_pin_active ? s_relay_src : s_origin;
        snprintf(line, sizeof(line), "org:%02X%02X       ", o[4], o[5]);
    } else {
        snprintf(line, sizeof(line), "org:none       ");
    }
    s_oled.drawString(0, 2, line);
    uint8_t md = s_last_mode;
    snprintf(line, sizeof(line), "mode:%-8s ", (md < 9) ? RPT_MODE_NAME[md] : "--");
    s_oled.drawString(0, 3, line);
    snprintf(line, sizeof(line), "rly:%lu/s        ", (unsigned long)(pps > 9999 ? 9999 : pps));
    s_oled.drawString(0, 4, line);
    snprintf(line, sizeof(line), "drop:%lu          ", (unsigned long)(s_slot_drops > 99999 ? 99999 : s_slot_drops));
    s_oled.drawString(0, 5, line);
    if (s_origin_rssi != 127) {
        snprintf(line, sizeof(line), "rssi:%-4d       ", (int)s_origin_rssi);
        s_oled.drawString(0, 6, line);
    }
}

static void xiaoUiSetup() {
    // Select the external u.FL antenna (better range for a coverage repeater).
    pinMode(WIFI_ENABLE, OUTPUT);      digitalWrite(WIFI_ENABLE, LOW);       // enable RF switch
    pinMode(WIFI_ANT_CONFIG, OUTPUT);  digitalWrite(WIFI_ANT_CONFIG, HIGH);  // external antenna
    pinMode(XIAO_BOOT_PIN, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);      xiaoLedSet(false);
    xiaoOledInit();                    // optional expansion-board status OLED
    Serial.println("[REPEATER] XIAO C6: external antenna, BOOT=ch-cycle");
}

static void xiaoUiLoop(uint32_t now) {
    xiaoOledDraw();                    // always refreshes (runs before the LED early-return)

    // BOOT short-press cycles the ESP-NOW channel 1 -> 6 -> 11 (persisted + live).
    static bool     prev_down = false;
    static uint32_t press_at  = 0;
    bool down = (digitalRead(XIAO_BOOT_PIN) == LOW);
    if (down && !prev_down) press_at = now;                     // press edge
    if (!down && prev_down) {                                   // release edge
        uint32_t held = now - press_at;
        if (held >= 30 && held < 1500) {                        // short press (30 ms debounce)
            uint8_t next = (s_channel == 1) ? 6 : (s_channel == 6) ? 11 : 1;
            s_channel = next;
            Preferences p; p.begin("espnow", false); p.putUChar("channel", next); p.end();
            esp_wifi_set_channel(next, WIFI_SECOND_CHAN_NONE);  // live apply
            s_blink_pending = (next == 1) ? 1 : (next == 6) ? 2 : 3;
            s_blink_at = now;
            Serial.printf("[REPEATER] channel -> %u\n", next);
        }
    }
    prev_down = down;

    // LED: channel-feedback blink train takes priority; else a liveness heartbeat
    // (brief flash ~every 2 s, faster while actively relaying).
    if (s_blink_pending > 0) {
        if ((int32_t)(now - s_blink_at) >= 0) {
            if (!s_led_on) { xiaoLedSet(true);  s_blink_at = now + 150; }
            else           { xiaoLedSet(false); s_blink_at = now + 150; s_blink_pending--; }
        }
        return;
    }
    static uint32_t s_hb_at = 0, s_hb_relayed = 0;
    if ((int32_t)(now - s_hb_at) >= 0) {
        bool relaying = (s_relayed != s_hb_relayed);
        s_hb_relayed = s_relayed;
        if (!s_led_on) { xiaoLedSet(true);  s_hb_at = now + 40; }
        else           { xiaoLedSet(false); s_hb_at = now + (relaying ? 460 : 1960); }
    }
}
#endif // BOARD_XIAO_C6

// ---------------------------------------------------------------------------
void repeaterSetup() {
    // Load channel + relay_src from NVS. A non-zero relay_src pins the source.
    {
        Preferences p;
        p.begin("espnow", true);
        s_channel = p.getUChar("channel", 1);
        if (p.getBytesLength("relay_src") == 6) {
            uint8_t m[6];
            p.getBytes("relay_src", m, 6);
            bool all_zero = true;
            for (int i = 0; i < 6; i++) if (m[i]) { all_zero = false; break; }
            if (!all_zero) { memcpy(s_relay_src, m, 6); s_pin_active = true; }
        }
        p.end();
    }

    // Apply channel (EspNowSender.init() already called WiFi.mode + esp_now_init).
    esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE);
    WiFi.setSleep(false);   // no modem sleep → minimum relay latency
    esp_now_register_recv_cb(onReceiveCb);

    if (s_pin_active) {
        char s[18]; macToStr(s_relay_src, s, sizeof(s));
        Serial.printf("[REPEATER] ch=%u mode=pinned src=%s\n", s_channel, s);
    } else {
        Serial.printf("[REPEATER] ch=%u mode=auto-follow\n", s_channel);
    }
#ifdef BOARD_XIAO_C6
    xiaoUiSetup();
#endif
}

// ---------------------------------------------------------------------------
void repeaterLoop() {
    // Drain the SPSC ring (consumer).
    while (s_slot_tail != s_slot_head) {
        RelaySlot& sl = s_slots[s_slot_tail % SLOT_COUNT];
        if (esp_now_send(BROADCAST_MAC, sl.buf, (size_t)sl.len) == ESP_OK) s_relayed = s_relayed + 1;
        s_slot_tail = s_slot_tail + 1;   // release slot
    }

    uint32_t now = millis();

    // Auto origin-lock release: origin silent > R_TIMEOUT → free for the next
    // origin (hot-standby failover). Never releases a manual pin.
    if (!s_pin_active && s_origin_locked && (now - s_origin_last_ms) > R_TIMEOUT_MS) {
        s_origin_locked = false;
    }

    // 2 s serial heartbeat.
    static uint32_t s_last_hb = 0, s_last_relayed = 0;
    if (now - s_last_hb >= HEARTBEAT_MS) {
        s_last_hb = now;
        uint32_t d = s_relayed - s_last_relayed; s_last_relayed = s_relayed;
        char origin_str[18] = "none";
        if (s_pin_active)            macToStr(s_relay_src, origin_str, sizeof(origin_str));
        else if (s_origin_locked)    macToStr(s_origin,    origin_str, sizeof(origin_str));
        char rssi_str[8];
        if (s_origin_rssi == 127) snprintf(rssi_str, sizeof(rssi_str), "n/a");
        else                      snprintf(rssi_str, sizeof(rssi_str), "%d", (int)s_origin_rssi);
        const char* md = (s_last_mode < 9) ? RPT_MODE_NAME[s_last_mode] : "--";
        Serial.printf("[RPT] ch=%u relayed/s=%u drops=%u origin=%s pin=%d mode=%s rssi=%s\n",
                      s_channel, (unsigned)(d / (HEARTBEAT_MS / 1000)), (unsigned)s_slot_drops,
                      origin_str, s_pin_active ? 1 : 0, md, rssi_str);
    }

#ifndef BOARD_XIAO_C6
    // Periodic LCD (M5 only).
    static uint32_t s_last_display = 0;
    if (now - s_last_display >= DISPLAY_INTERVAL_MS) {
        s_last_display = now;
        char mac_str[18] = "auto";
        if (s_pin_active)         macToStr(s_relay_src, mac_str, sizeof(mac_str));
        else if (s_origin_locked) macToStr(s_origin,    mac_str, sizeof(mac_str));
        displayUpdateRepeaterStats(s_channel, mac_str,
                                   (uint32_t)s_relayed, (uint32_t)s_slot_drops);
    }
#endif

#ifdef BOARD_XIAO_C6
    xiaoUiLoop(now);
#endif
}

// ---------------------------------------------------------------------------
// External API for node_serial_config.cpp — live update without reboot.
// Empty (all-zero) MAC restores auto origin-follow; a real MAC is a strict pin.
// ---------------------------------------------------------------------------
void repeaterSetRelaySrc(const uint8_t* mac) {
    bool all_zero = true;
    for (int i = 0; i < 6; i++) if (mac[i]) { all_zero = false; break; }
    if (all_zero) {
        s_pin_active = false;             // → auto origin-follow
    } else {
        memcpy(s_relay_src, mac, 6);
        s_pin_active = true;              // → strict manual pin
    }
}

bool repeaterGetRelaySrc(uint8_t* out_mac) {
    if (!s_pin_active) return false;
    memcpy(out_mac, s_relay_src, 6);
    return true;
}

#endif // REPEATER
