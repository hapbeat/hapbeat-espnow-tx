#ifndef SERIAL_HANDLER_H
#define SERIAL_HANDLER_H

#include <Arduino.h>
#include "types.h"
#include "config.h"

// Serial frame parser for Bridge-to-Transmitter communication.
// Implements a state machine that accumulates bytes from the serial port
// and parses them into SerialFrame structures.
class SerialHandler {
public:
    SerialHandler();

    // Initialize the serial port at the configured baud rate.
    void init(HardwareSerial& serial);

    // Read available bytes from serial and attempt to parse frames.
    // Call this every loop iteration.
    void update();

    // Check if a complete frame is available.
    bool hasFrame() const;

    // Retrieve the latest parsed frame. Returns false if no frame is available.
    bool getFrame(SerialFrame& frame);

    // Timestamp (millis) of last successfully received frame. 0 if none yet.
    uint32_t getLastFrameTime() const { return last_frame_time_; }

    // Timestamp (millis) of last byte received on serial RX. 0 if none yet.
    uint32_t getLastRxByteTime() const { return last_rx_byte_time_; }

    // --- Response builders (Transmitter → Bridge) ---

    // Send ACK response for a given sequence number.
    void sendAck(uint16_t seq, uint8_t status);

    // Send DEVICE_INFO response with a list of discovered devices.
    void sendDeviceInfo(const DeviceInfo* devices, uint8_t count);

    // Send TX_STATUS response with transmitter state information.
    void sendTxStatus(uint8_t queue_depth, uint8_t last_result, uint32_t uptime_sec);

private:
    // Frame parsing state machine states
    enum ParseState {
        WAIT_START,
        READ_LENGTH_LOW,
        READ_LENGTH_HIGH,
        READ_COMMAND,
        READ_SEQ_LOW,
        READ_SEQ_HIGH,
        READ_PAYLOAD,
        READ_CHECKSUM,
        VERIFY_END,
    };

    // Compute XOR checksum over command_type + seq bytes + payload.
    static uint8_t computeChecksum(const uint8_t* data, uint16_t length);

    // Build a complete serial frame and write it to the serial port.
    void buildAndSendFrame(uint8_t command_type, uint16_t seq, const uint8_t* payload, uint16_t length);

    HardwareSerial* serial_;
    ParseState state_;

    // Frame accumulation buffer
    uint8_t  buffer_[MAX_SERIAL_BUFFER];
    uint16_t buffer_pos_;

    // In-progress frame fields
    uint16_t frame_length_;   // payload length from header
    uint8_t  command_type_;
    uint16_t seq_;
    uint16_t payload_pos_;
    uint8_t  expected_checksum_;

    // Completed frame
    SerialFrame completed_frame_;
    bool frame_ready_;

    // Timestamp of last valid frame
    uint32_t last_frame_time_;

    // Timestamp of last byte received (any byte, not just valid frames)
    uint32_t last_rx_byte_time_;

    // Response sequence counter (responses use seq=0)
    static constexpr uint16_t RESPONSE_SEQ = 0;

    // Reset the parser to initial state.
    void resetParser();
};

#endif // SERIAL_HANDLER_H
