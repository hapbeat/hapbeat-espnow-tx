#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

// Serial communication
static constexpr uint32_t SERIAL_BAUD = 921600;
static constexpr uint16_t MAX_SERIAL_BUFFER = 512;

// Serial frame markers
static constexpr uint8_t START_MARKER = 0xAA;
static constexpr uint8_t END_MARKER   = 0x55;

// Frame header: start_marker(1) + frame_length(2) + command_type(1) + seq(2) = 6 bytes
static constexpr uint8_t FRAME_HEADER_SIZE = 6;

// ESP-NOW
static constexpr uint8_t ESPNOW_CHANNEL = 1;
static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Command types received from Bridge
static constexpr uint8_t CMD_DISPATCH_PLAY     = 0x01;
static constexpr uint8_t CMD_DISPATCH_STOP     = 0x02;
static constexpr uint8_t CMD_DISPATCH_STOP_ALL = 0x03;
static constexpr uint8_t CMD_TIME_SYNC         = 0x20;
static constexpr uint8_t CMD_QUERY_STATUS      = 0x30;
static constexpr uint8_t CMD_DEVICE_SCAN       = 0x31;

// Response types sent to Bridge
static constexpr uint8_t RSP_ACK         = 0x80;
static constexpr uint8_t RSP_DEVICE_INFO = 0x81;
static constexpr uint8_t RSP_TX_STATUS   = 0x82;

// ESP-NOW command bytes
static constexpr uint8_t ESPNOW_CMD_PLAY     = 0x01;
static constexpr uint8_t ESPNOW_CMD_STOP     = 0x02;
static constexpr uint8_t ESPNOW_CMD_STOP_ALL = 0x03;

// ACK status codes
static constexpr uint8_t ACK_OK    = 0x00;
static constexpr uint8_t ACK_ERROR = 0x01;

// Status reporting interval (milliseconds)
static constexpr uint32_t STATUS_REPORT_INTERVAL_MS = 10000;

#endif // CONFIG_H
