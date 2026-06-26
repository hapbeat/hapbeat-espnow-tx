// ---------------------------------------------------------------------------
// repeater.cpp — ESP-NOW repeater (see header). #ifdef REPEATER.
//
// Receives 0xAA stream packets from the configured source MAC and re-broadcasts
// them verbatim. The re-broadcast's ESP-NOW source MAC becomes *this device's*
// MAC, so receivers see the repeater as an independent transmitter and can
// select between direct-source and repeater-source based on RSSI.
//
// Loop-prevention guarantee: only the single NVS-configured source MAC is
// relayed. Other MACs (other repeaters, other sources) are silently ignored.
// This bounds relay depth at 1 hop (source → repeater → receiver).
//
// Architecture note: ESP-NOW receive callback runs in the WiFi task — calling
// esp_now_send() from within the callback would risk re-entering the WiFi
// stack. We use a single-slot pending buffer and do the actual send from
// repeaterLoop() (Arduino loop task). At 1 ms/packet the main loop has ample
// time; if a second packet arrives before loop() drains the slot it is counted
// as a "slot busy" drop (acceptable for audio streaming).
// ---------------------------------------------------------------------------

#include "repeater.h"

#ifdef REPEATER

#include "config.h"    // BROADCAST_MAC
#include "display.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <cstring>
#include <cstdio>

static const uint8_t  STREAM_TYPE          = 0xAA;
static const uint32_t DISPLAY_INTERVAL_MS  = 250;  // ~4 fps

// ---- configured source MAC -------------------------------------------------
static uint8_t s_relay_src[6] = {};
static bool    s_relay_src_set = false;

// ---- runtime state ---------------------------------------------------------
static uint8_t s_channel = 1;   // for display

// ---- stats -----------------------------------------------------------------
// These are written from both WiFi task (receive callback) and loop task.
// Volatile is sufficient on ESP32 for 32-bit reads; worst-case miss is one
// count off — acceptable for display-only counters.
static volatile uint32_t s_relayed     = 0;  // packets successfully re-broadcast
static volatile uint32_t s_slot_drops  = 0;  // dropped because pending slot was full
// (wrong-source packets are silently ignored and not counted)

// ---- single-slot pending relay buffer (lock-free, single producer) ---------
// Callback writes here; loop() reads and sends. One slot is enough because
// packets arrive at ~1 kHz and loop() runs at MHz.
static volatile bool s_has_pkt = false;
static uint8_t       s_pkt_buf[250];    // max ESP-NOW payload
static int           s_pkt_len = 0;

// ---- helper: MAC → string --------------------------------------------------
static void macToStr(const uint8_t* mac, char* buf, size_t bufsz) {
    snprintf(buf, bufsz, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Parse "AA:BB:CC:DD:EE:FF" → 6 bytes. Returns true on success.
static bool parseMac(const char* str, uint8_t* mac) {
    unsigned int b[6];
    if (sscanf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6) return false;
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)b[i];
    return true;
}

// ---- ESP-NOW receive callback (WiFi task — do NOT call esp_now_send here) --
static void onReceiveCb(const uint8_t* src_mac, const uint8_t* data, int len) {
    // Only relay from the single configured source MAC.
    if (!s_relay_src_set) return;
    if (memcmp(src_mac, s_relay_src, 6) != 0) return;
    // Only relay 0xAA stream packets.
    if (len < 1 || data[0] != STREAM_TYPE) return;
    if (len > 250) return;

    if (!s_has_pkt) {
        // Slot is free — copy for loop() to send.
        memcpy(s_pkt_buf, data, (size_t)len);
        s_pkt_len = len;
        s_has_pkt = true;   // volatile write: visible to loop task
    } else {
        // Slot occupied — drop. The receiver tolerates sporadic gaps.
        s_slot_drops++;
    }
}

// ---------------------------------------------------------------------------
void repeaterSetup() {
    // Load channel and relay_src from NVS.
    {
        Preferences p;
        p.begin("espnow", true);
        s_channel = p.getUChar("channel", 1);
        size_t mac_len = p.getBytesLength("relay_src");
        if (mac_len == 6) {
            p.getBytes("relay_src", s_relay_src, 6);
            s_relay_src_set = true;
        }
        p.end();
    }

    // Apply channel (EspNowSender.init() already called WiFi.mode + esp_now_init).
    esp_wifi_set_channel(s_channel, WIFI_SECOND_CHAN_NONE);

    // Disable modem sleep to minimize relay latency.
    WiFi.setSleep(false);

    // Register receive callback.
    esp_now_register_recv_cb(onReceiveCb);

    char mac_str[18] = "not set";
    if (s_relay_src_set) macToStr(s_relay_src, mac_str, sizeof(mac_str));
    Serial.printf("[REPEATER] ch=%u relay_src=%s\n", s_channel, mac_str);
}

// ---------------------------------------------------------------------------
void repeaterLoop() {
    // Drain the pending relay slot if the receive callback filled it.
    if (s_has_pkt) {
        esp_err_t e = esp_now_send(BROADCAST_MAC, s_pkt_buf, (size_t)s_pkt_len);
        s_has_pkt = false;   // volatile write: release slot for callback
        if (e == ESP_OK) {
            s_relayed++;
        }
        // On failure: counted as a send error; we don't track separately here
        // since it's indistinguishable from a normal loss at the receiver.
    }

    // Periodic display update.
    static uint32_t s_last_display = 0;
    uint32_t now = millis();
    if (now - s_last_display >= DISPLAY_INTERVAL_MS) {
        s_last_display = now;
        char mac_str[18] = "not set";
        if (s_relay_src_set) macToStr(s_relay_src, mac_str, sizeof(mac_str));
        displayUpdateRepeaterStats(s_channel, mac_str,
                                   (uint32_t)s_relayed, (uint32_t)s_slot_drops);
    }
}

// ---------------------------------------------------------------------------
// External API for node_serial_config.cpp — live update without reboot.
// ---------------------------------------------------------------------------
void repeaterSetRelaySrc(const uint8_t* mac) {
    memcpy(s_relay_src, mac, 6);
    s_relay_src_set = true;
}

bool repeaterGetRelaySrc(uint8_t* out_mac) {
    if (!s_relay_src_set) return false;
    memcpy(out_mac, s_relay_src, 6);
    return true;
}

#endif // REPEATER
