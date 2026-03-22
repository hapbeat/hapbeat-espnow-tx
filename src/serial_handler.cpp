#include "serial_handler.h"
#include <cstring>

SerialHandler::SerialHandler()
    : serial_(nullptr)
    , state_(WAIT_START)
    , buffer_pos_(0)
    , frame_length_(0)
    , command_type_(0)
    , seq_(0)
    , payload_pos_(0)
    , expected_checksum_(0)
    , frame_ready_(false)
    , last_frame_time_(0)
    , last_rx_byte_time_(0)
{
    memset(&completed_frame_, 0, sizeof(completed_frame_));
}

void SerialHandler::init(HardwareSerial& serial) {
    serial_ = &serial;
    serial_->begin(SERIAL_BAUD);
    resetParser();
    log_i("SerialHandler initialized at %u baud", SERIAL_BAUD);
}

void SerialHandler::resetParser() {
    state_ = WAIT_START;
    buffer_pos_ = 0;
    frame_length_ = 0;
    command_type_ = 0;
    seq_ = 0;
    payload_pos_ = 0;
    expected_checksum_ = 0;
}

void SerialHandler::update() {
    if (!serial_) return;

    while (serial_->available()) {
        last_rx_byte_time_ = millis();
        uint8_t byte = serial_->read();

        switch (state_) {
            case WAIT_START:
                if (byte == START_MARKER) {
                    state_ = READ_LENGTH_LOW;
                }
                break;

            case READ_LENGTH_LOW:
                frame_length_ = byte;
                state_ = READ_LENGTH_HIGH;
                break;

            case READ_LENGTH_HIGH:
                frame_length_ |= (static_cast<uint16_t>(byte) << 8);
                if (frame_length_ > MAX_PAYLOAD_SIZE) {
                    log_w("Frame payload too large: %u bytes, max %u", frame_length_, MAX_PAYLOAD_SIZE);
                    resetParser();
                } else {
                    state_ = READ_COMMAND;
                }
                break;

            case READ_COMMAND:
                command_type_ = byte;
                state_ = READ_SEQ_LOW;
                break;

            case READ_SEQ_LOW:
                seq_ = byte;
                state_ = READ_SEQ_HIGH;
                break;

            case READ_SEQ_HIGH:
                seq_ |= (static_cast<uint16_t>(byte) << 8);
                payload_pos_ = 0;
                if (frame_length_ == 0) {
                    // No payload, go straight to checksum
                    state_ = READ_CHECKSUM;
                } else {
                    state_ = READ_PAYLOAD;
                }
                break;

            case READ_PAYLOAD:
                completed_frame_.payload[payload_pos_++] = byte;
                if (payload_pos_ >= frame_length_) {
                    state_ = READ_CHECKSUM;
                }
                break;

            case READ_CHECKSUM:
                expected_checksum_ = byte;
                state_ = VERIFY_END;
                break;

            case VERIFY_END:
                if (byte == END_MARKER) {
                    // Build the data block for checksum verification:
                    // command_type(1) + seq(2) + payload(frame_length_)
                    uint16_t check_len = 1 + 2 + frame_length_;
                    uint8_t check_buf[MAX_SERIAL_BUFFER];
                    check_buf[0] = command_type_;
                    check_buf[1] = static_cast<uint8_t>(seq_ & 0xFF);
                    check_buf[2] = static_cast<uint8_t>((seq_ >> 8) & 0xFF);
                    if (frame_length_ > 0) {
                        memcpy(&check_buf[3], completed_frame_.payload, frame_length_);
                    }

                    uint8_t computed = computeChecksum(check_buf, check_len);
                    if (computed == expected_checksum_) {
                        completed_frame_.command_type = command_type_;
                        completed_frame_.seq = seq_;
                        completed_frame_.payload_length = frame_length_;
                        frame_ready_ = true;
                        last_frame_time_ = millis();
                        log_d("Frame received: cmd=0x%02X seq=%u len=%u",
                              command_type_, seq_, frame_length_);
                    } else {
                        log_w("Checksum mismatch: expected=0x%02X computed=0x%02X",
                              expected_checksum_, computed);
                    }
                } else {
                    log_w("Invalid end marker: 0x%02X (expected 0x%02X)", byte, END_MARKER);
                }
                resetParser();
                break;
        }
    }
}

bool SerialHandler::hasFrame() const {
    return frame_ready_;
}

bool SerialHandler::getFrame(SerialFrame& frame) {
    if (!frame_ready_) return false;
    memcpy(&frame, &completed_frame_, sizeof(SerialFrame));
    frame_ready_ = false;
    return true;
}

uint8_t SerialHandler::computeChecksum(const uint8_t* data, uint16_t length) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

void SerialHandler::buildAndSendFrame(uint8_t command_type, uint16_t seq,
                                       const uint8_t* payload, uint16_t length) {
    if (!serial_) return;

    // Build checksum data: command_type(1) + seq(2) + payload
    uint16_t check_len = 1 + 2 + length;
    uint8_t check_buf[MAX_SERIAL_BUFFER];
    check_buf[0] = command_type;
    check_buf[1] = static_cast<uint8_t>(seq & 0xFF);
    check_buf[2] = static_cast<uint8_t>((seq >> 8) & 0xFF);
    if (length > 0 && payload != nullptr) {
        memcpy(&check_buf[3], payload, length);
    }
    uint8_t checksum = computeChecksum(check_buf, check_len);

    // Write frame to serial
    serial_->write(START_MARKER);

    // frame_length (payload length, little endian)
    serial_->write(static_cast<uint8_t>(length & 0xFF));
    serial_->write(static_cast<uint8_t>((length >> 8) & 0xFF));

    // command_type
    serial_->write(command_type);

    // seq (little endian)
    serial_->write(static_cast<uint8_t>(seq & 0xFF));
    serial_->write(static_cast<uint8_t>((seq >> 8) & 0xFF));

    // payload
    if (length > 0 && payload != nullptr) {
        serial_->write(payload, length);
    }

    // checksum
    serial_->write(checksum);

    // end marker
    serial_->write(END_MARKER);

    serial_->flush();
}

void SerialHandler::sendAck(uint16_t seq, uint8_t status) {
    // ACK payload: seq(2) + status(1) = 3 bytes
    uint8_t payload[3];
    payload[0] = static_cast<uint8_t>(seq & 0xFF);
    payload[1] = static_cast<uint8_t>((seq >> 8) & 0xFF);
    payload[2] = status;

    buildAndSendFrame(RSP_ACK, RESPONSE_SEQ, payload, 3);
    log_d("ACK sent: seq=%u status=%u", seq, status);
}

void SerialHandler::sendDeviceInfo(const DeviceInfo* devices, uint8_t count) {
    // DEVICE_INFO payload: device_count(1) + [mac(6) + group(1) + rssi(1) + battery(1)] * count
    static constexpr uint8_t DEVICE_ENTRY_SIZE = 6 + 1 + 1 + 1; // 9 bytes per device
    uint16_t payload_len = 1 + (count * DEVICE_ENTRY_SIZE);
    uint8_t payload[MAX_PAYLOAD_SIZE];

    if (payload_len > MAX_PAYLOAD_SIZE) {
        log_w("DeviceInfo payload too large: %u devices", count);
        return;
    }

    payload[0] = count;
    for (uint8_t i = 0; i < count; i++) {
        uint16_t offset = 1 + (i * DEVICE_ENTRY_SIZE);
        memcpy(&payload[offset], devices[i].mac, 6);
        payload[offset + 6] = devices[i].group;
        payload[offset + 7] = static_cast<uint8_t>(devices[i].rssi);
        payload[offset + 8] = devices[i].battery;
    }

    buildAndSendFrame(RSP_DEVICE_INFO, RESPONSE_SEQ, payload, payload_len);
    log_d("DeviceInfo sent: %u devices", count);
}

void SerialHandler::sendTxStatus(uint8_t queue_depth, uint8_t last_result, uint32_t uptime_sec) {
    // TX_STATUS payload: queue_depth(1) + last_result(1) + uptime_sec(4) = 6 bytes
    uint8_t payload[6];
    payload[0] = queue_depth;
    payload[1] = last_result;
    payload[2] = static_cast<uint8_t>(uptime_sec & 0xFF);
    payload[3] = static_cast<uint8_t>((uptime_sec >> 8) & 0xFF);
    payload[4] = static_cast<uint8_t>((uptime_sec >> 16) & 0xFF);
    payload[5] = static_cast<uint8_t>((uptime_sec >> 24) & 0xFF);

    buildAndSendFrame(RSP_TX_STATUS, RESPONSE_SEQ, payload, 6);
    log_d("TxStatus sent: queue=%u result=%u uptime=%u", queue_depth, last_result, uptime_sec);
}
