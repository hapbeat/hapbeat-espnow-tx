#ifndef TYPES_H
#define TYPES_H

#include <cstdint>
#include "config.h"

// Maximum payload size = MAX_SERIAL_BUFFER minus header/footer overhead
static constexpr uint16_t MAX_PAYLOAD_SIZE = MAX_SERIAL_BUFFER - FRAME_HEADER_SIZE - 2; // -2 for checksum + end_marker

// Parsed serial frame from Bridge
struct SerialFrame {
    uint8_t  command_type;
    uint16_t seq;
    uint8_t  payload[MAX_PAYLOAD_SIZE];
    uint16_t payload_length;
};

// ESP-NOW packet sent to Hapbeat devices (11 bytes total)
struct __attribute__((packed)) EspNowPacket {
    uint8_t  command;        // 0x01=play, 0x02=stop, 0x03=stop_all
    uint32_t event_id_hash;  // CRC32 hash of Event ID string
    uint32_t target_time;    // milliseconds
    uint8_t  group;          // target group (0=broadcast)
    uint8_t  gain;           // 0-255 (0=0.0, 255=1.0)
};

// Device information reported via DEVICE_INFO response
struct DeviceInfo {
    uint8_t mac[6];
    uint8_t group;
    int8_t  rssi;
    uint8_t battery;
};

#endif // TYPES_H
