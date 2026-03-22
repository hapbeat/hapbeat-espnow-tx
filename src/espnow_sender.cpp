#include "espnow_sender.h"
#include "config.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <cstring>

EspNowSender* EspNowSender::instance_ = nullptr;

// File-local callback with the correct esp_now_send_status_t signature.
// Delegates to the public handleSendResult method via the singleton pointer.
static void espnow_on_send(const uint8_t* mac_addr, esp_now_send_status_t status) {
    EspNowSender* inst = EspNowSender::getInstance();
    if (inst) {
        inst->handleSendResult(status == ESP_NOW_SEND_SUCCESS, mac_addr);
    }
}

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
    WiFi.disconnect();

    // Set the WiFi channel
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    log_i("WiFi MAC: %s", WiFi.macAddress().c_str());
    log_i("WiFi channel: %d", ESPNOW_CHANNEL);

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
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) != ESP_OK) {
        log_e("Failed to add broadcast peer");
        return false;
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
    if (!success) {
        log_w("ESP-NOW delivery failed to %02X:%02X:%02X:%02X:%02X:%02X",
              mac_addr[0], mac_addr[1], mac_addr[2],
              mac_addr[3], mac_addr[4], mac_addr[5]);
    }
}

