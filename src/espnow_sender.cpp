#include "espnow_sender.h"
#include "config.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>   // ESP_IDF_VERSION / ESP_IDF_VERSION_VAL (rate-config gate)
#include <Preferences.h>
#include <cstring>

EspNowSender* EspNowSender::instance_ = nullptr;

// File-local ESP-NOW send callback. arduino-esp32 3.x (IDF5) changed the
// signature to esp_now_send_info_t* (= wifi_tx_info_t; dest MAC in des_addr);
// 2.x passed the bare MAC. Both delegate to handleSendResult.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void espnow_on_send(const esp_now_send_info_t* tx_info, esp_now_send_status_t status) {
    EspNowSender* inst = EspNowSender::getInstance();
    if (inst) {
        inst->handleSendResult(status == ESP_NOW_SEND_SUCCESS,
                               tx_info ? tx_info->des_addr : nullptr);
    }
}
#else
static void espnow_on_send(const uint8_t* mac_addr, esp_now_send_status_t status) {
    EspNowSender* inst = EspNowSender::getInstance();
    if (inst) {
        inst->handleSendResult(status == ESP_NOW_SEND_SUCCESS, mac_addr);
    }
}
#endif

EspNowSender::EspNowSender()
    : initialized_(false)
    , last_result_(false)
    , total_sent_(0)
    , total_failed_(0)
{
    instance_ = this;
}

bool EspNowSender::init() {
    // Set WiFi to station mode (required for ESP-NOW)
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.disconnect();

    // Radio tuning (mirrors the reference wireless-sender-firmware). Without
    // this, ESP-NOW broadcasts at the default 1 Mbps long-preamble rate
    // (~1 ms airtime per small packet → caps small-packet throughput near
    // ~800-1000/s, which forced a deep in-flight queue and added latency).
    esp_wifi_set_ps(WIFI_PS_NONE);                 // no modem sleep (latency/jitter)
#ifdef BOARD_XIAO_C6
    esp_wifi_set_max_tx_power(80);                  // C6 max (~20 dBm)
#else
    esp_wifi_set_max_tx_power(84);                  // S3 / classic ESP32 (~21 dBm)
#endif
#ifdef REPEATER
    // Repeater carries bgn+LR always so it can receive (and relay) a RANGE=LONG
    // (802.11 LR) origin as well as ordinary 6M origins (DEC-043 P5 §7.3). Its own
    // relay TX rate stays 6M here; matching the origin's LR rate is a RANGE-toggle
    // refinement (auto-follow on C6 rx_ctrl / NVS toggle on M5).
    esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
#else
    esp_wifi_set_protocol(WIFI_IF_STA,
                          WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
#endif
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
#if defined(AUDIO_SOURCE) || defined(REPEATER)
    // Live audio stream: 6 Mbps OFDM. ~150 µs airtime per 49 B packet lifts the
    // throughput ceiling well above the 1000 pkt/s the 16 kHz/16-frame stream
    // needs, so a shallow in-flight depth keeps latency low. 6 Mbps also has
    // ~+14 dB RX sensitivity vs 54 Mbps, so range stays good.
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_6M);  // IDF4 (S3 / classic)
#endif
#endif

    // Set the WiFi channel. NVS wins over the compile-time default so the
    // channel set from Studio (set_espnow_channel) survives reboot
    // (contracts/serial-config.md §4.9, DEC-034).
    uint8_t channel = ESPNOW_CHANNEL;
    {
        Preferences p;
        p.begin("espnow", true);
        channel = p.getUChar("channel", ESPNOW_CHANNEL);
        p.end();
        if (channel != 1 && channel != 6 && channel != 11) channel = ESPNOW_CHANNEL;
    }
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    log_i("WiFi MAC: %s", WiFi.macAddress().c_str());
    log_i("WiFi channel: %d", channel);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        log_e("ESP-NOW init failed");
        return false;
    }

    // Register send callback (file-local function with correct ESP-IDF signature)
    esp_now_register_send_cb(espnow_on_send);

    // Register broadcast peer
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, BROADCAST_MAC, 6);
    peer_info.channel = channel;
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        log_e("Failed to add broadcast peer");
        return false;
    }

#if defined(AUDIO_SOURCE) || defined(REPEATER)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    // IDF5 (XIAO C6): esp_wifi_config_espnow_rate is deprecated → per-peer rate,
    // applied after esp_now_add_peer().
    esp_now_rate_config_t rate_cfg = {};
    rate_cfg.phymode = WIFI_PHY_MODE_11G;
    rate_cfg.rate    = WIFI_PHY_RATE_6M;
    esp_err_t rate_err = esp_now_set_peer_rate_config(BROADCAST_MAC, &rate_cfg);
    log_i("ESP-NOW rate 6M (peer_rate_config) rc=%d", (int)rate_err);
#else
    // IDF4 (S3 / classic): re-apply AFTER esp_now_init() — applying it only
    // before init did not stick (throughput stayed at the 1 Mbps default).
    esp_err_t rate_err = esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_6M);
    log_i("ESP-NOW rate 6M (post-init) rc=%d", (int)rate_err);
#endif
#endif

    // Lock the channel with a verify-loop. The single set_channel above (before
    // esp_now_init) can be undone by the WiFi STA startup sequence — the same
    // "applied before init did not stick" caveat noted for the rate just above.
    // A transmitter silently on the wrong channel means the WHOLE fleet stops
    // receiving, so re-assert until esp_wifi_get_channel reports the target
    // (the receiver has the identical guard; user 2026-07-11 random-boot RX bug).
    {
        uint8_t prim = 0;
        wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
        int tries = 0;
        for (; tries < 8; tries++) {
            esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
            delay(10);
            if (esp_wifi_get_channel(&prim, &sec) == ESP_OK && prim == channel) break;
        }
        log_i("ESP-NOW channel locked to %d (radio=%d, tries=%d)", channel, prim, tries + 1);
    }

    initialized_ = true;
    log_i("ESP-NOW sender initialized");
    return true;
}

bool EspNowSender::sendBroadcast(const EspNowPacket& packet) {
    if (!initialized_) {
        log_e("ESP-NOW not initialized");
        return false;
    }

    esp_err_t result = esp_now_send(
        BROADCAST_MAC,
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(EspNowPacket)
    );

    if (result != ESP_OK) {
        log_e("ESP-NOW send failed: %d", result);
        total_failed_++;
        last_result_ = false;
        return false;
    }

    total_sent_++;
    log_d("ESP-NOW broadcast sent: cmd=0x%02X hash=0x%08X group=%u gain=%u",
          packet.command, packet.event_id_hash, packet.group, packet.gain);
    return true;
}

bool EspNowSender::sendToGroup(const EspNowPacket& packet, uint8_t group) {
    // Group filtering is handled on the device side.
    // The transmitter broadcasts the packet with the group field set,
    // and each device decides whether to act on it based on its own group assignment.
    return sendBroadcast(packet);
}

bool EspNowSender::getLastResult() const {
    return last_result_;
}

uint32_t EspNowSender::getTotalSent() const {
    return total_sent_;
}

uint32_t EspNowSender::getTotalFailed() const {
    return total_failed_;
}

void EspNowSender::handleSendResult(bool success, const uint8_t* mac_addr) {
    last_result_ = success;
    if (!success && mac_addr) {
        log_w("ESP-NOW delivery failed to %02X:%02X:%02X:%02X:%02X:%02X",
              mac_addr[0], mac_addr[1], mac_addr[2],
              mac_addr[3], mac_addr[4], mac_addr[5]);
    } else if (!success) {
        log_w("ESP-NOW delivery failed");
    }
}

