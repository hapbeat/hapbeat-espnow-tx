#ifndef ESPNOW_SENDER_H
#define ESPNOW_SENDER_H

#include <cstdint>
#include "types.h"

// ESP-NOW sender for broadcasting packets to Hapbeat devices.
// Handles ESP-NOW initialization, peer registration, and packet transmission.
class EspNowSender {
public:
    EspNowSender();

    // Initialize WiFi in STA mode and set up ESP-NOW with send callback.
    // Returns true on success.
    bool init();

    // Send a packet to the broadcast address (all devices).
    bool sendBroadcast(const EspNowPacket& packet);

    // Send a packet targeting a specific group.
    // Currently implemented as broadcast; group filtering is done on the device side.
    bool sendToGroup(const EspNowPacket& packet, uint8_t group);

    // Get the result of the last send operation.
    // Returns true if the last send was successful.
    bool getLastResult() const;

    // Get total number of packets sent since boot.
    uint32_t getTotalSent() const;

    // Get total number of send failures since boot.
    uint32_t getTotalFailed() const;

    // Called from the ESP-NOW send callback (registered in init).
    // Updates last_result_ based on delivery status.
    void handleSendResult(bool success, const uint8_t* mac_addr);

    // Access the singleton instance (used by the file-local send callback).
    static EspNowSender* getInstance() { return instance_; }

private:
    bool     initialized_;
    bool     last_result_;
    uint32_t total_sent_;
    uint32_t total_failed_;

    // Singleton pointer for the send callback to access instance state.
    static EspNowSender* instance_;
};

#endif // ESPNOW_SENDER_H
