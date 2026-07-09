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

static const uint8_t  STREAM_TYPE         = 0xAA;
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
    if (len < 5 || len > 250 || data[0] != STREAM_TYPE) return;
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

    // Enqueue into the SPSC ring. Set RELAYED (bit7) on the copy — the ONLY byte
    // changed; seq/body/piggyback are verbatim (receiver piggyback matches
    // prev_seq by absolute value, so seq MUST NOT be rewritten).
    uint32_t head = s_slot_head;
    if ((head - s_slot_tail) >= (uint32_t)SLOT_COUNT) { s_slot_drops++; return; }
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
}

// ---------------------------------------------------------------------------
void repeaterLoop() {
    // Drain the SPSC ring (consumer).
    while (s_slot_tail != s_slot_head) {
        RelaySlot& sl = s_slots[s_slot_tail % SLOT_COUNT];
        if (esp_now_send(BROADCAST_MAC, sl.buf, (size_t)sl.len) == ESP_OK) s_relayed++;
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
        Serial.printf("[RPT] relayed/s=%u drops=%u origin=%s pin=%d rssi=%s\n",
                      (unsigned)(d / (HEARTBEAT_MS / 1000)), (unsigned)s_slot_drops,
                      origin_str, s_pin_active ? 1 : 0, rssi_str);
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
