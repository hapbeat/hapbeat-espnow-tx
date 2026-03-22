#include "command_dispatcher.h"
#include "config.h"
#include "crc32.h"
#include "display.h"
#include <Arduino.h>
#include <cstring>

CommandDispatcher::CommandDispatcher()
    : serial_(nullptr)
    , sender_(nullptr)
    , time_sync_(nullptr)
{
}

void CommandDispatcher::init(SerialHandler& serial, EspNowSender& sender, TimeSync& timeSync) {
    serial_    = &serial;
    sender_    = &sender;
    time_sync_ = &timeSync;
    log_i("CommandDispatcher initialized");
}

void CommandDispatcher::processFrame(const SerialFrame& frame) {
    log_d("Dispatching command 0x%02X, seq=%u, payload_len=%u",
          frame.command_type, frame.seq, frame.payload_length);

    switch (frame.command_type) {
        case CMD_DISPATCH_PLAY:
            handlePlay(frame);
            break;
        case CMD_DISPATCH_STOP:
            handleStop(frame);
            break;
        case CMD_DISPATCH_STOP_ALL:
            handleStopAll(frame);
            break;
        case CMD_TIME_SYNC:
            handleTimeSync(frame);
            break;
        case CMD_QUERY_STATUS:
            handleQueryStatus(frame);
            break;
        case CMD_DEVICE_SCAN:
            handleDeviceScan(frame);
            break;
        default:
            log_w("Unknown command: 0x%02X", frame.command_type);
            if (serial_) {
                serial_->sendAck(frame.seq, ACK_ERROR);
            }
            break;
    }
}

uint16_t CommandDispatcher::extractString(const uint8_t* payload, uint16_t payload_length,
                                           uint16_t offset, char* out, uint16_t out_size) {
    uint16_t i = 0;
    while ((offset + i) < payload_length && i < (out_size - 1)) {
        out[i] = static_cast<char>(payload[offset + i]);
        if (out[i] == '\0') {
            return i + 1; // include the null terminator in consumed count
        }
        i++;
    }
    // No null terminator found; null-terminate the output anyway
    out[i] = '\0';
    return 0; // indicate failure
}

void CommandDispatcher::handlePlay(const SerialFrame& frame) {
    // DISPATCH_PLAY payload:
    //   event_id (null-terminated string)
    //   target_time (int64, little endian)
    //   target_group (uint8)
    //   gain (float32, little endian)

    char event_id[128];
    uint16_t consumed = extractString(frame.payload, frame.payload_length, 0, event_id, sizeof(event_id));
    if (consumed == 0) {
        log_e("PLAY: failed to extract event_id");
        if (serial_) serial_->sendAck(frame.seq, ACK_ERROR);
        return;
    }

    uint16_t pos = consumed;

    // Check remaining payload: int64(8) + uint8(1) + float32(4) = 13 bytes
    if (pos + 13 > frame.payload_length) {
        log_e("PLAY: payload too short (need %u more bytes, have %u)",
              13, frame.payload_length - pos);
        if (serial_) serial_->sendAck(frame.seq, ACK_ERROR);
        return;
    }

    // Extract target_time (int64, little endian) - microseconds from Bridge
    int64_t target_time_us = 0;
    memcpy(&target_time_us, &frame.payload[pos], 8);
    pos += 8;

    // Extract target_group (uint8)
    uint8_t target_group = frame.payload[pos];
    pos += 1;

    // Extract gain (float32, little endian)
    float gain_f = 0.0f;
    memcpy(&gain_f, &frame.payload[pos], 4);
    pos += 4;

    // Compute CRC32 hash of the event ID string
    uint32_t event_hash = crc32(event_id);

    // Convert target_time from microseconds to milliseconds for ESP-NOW packet
    uint32_t target_time_ms = static_cast<uint32_t>((target_time_us / 1000) & 0xFFFFFFFF);

    // Convert gain from float [0.0, 1.0] to uint8 [0, 255]
    uint8_t gain_u8 = 0;
    if (gain_f <= 0.0f) {
        gain_u8 = 0;
    } else if (gain_f >= 1.0f) {
        gain_u8 = 255;
    } else {
        gain_u8 = static_cast<uint8_t>(gain_f * 255.0f + 0.5f);
    }

    // Build ESP-NOW packet
    EspNowPacket packet;
    packet.command       = ESPNOW_CMD_PLAY;
    packet.event_id_hash = event_hash;
    packet.target_time   = target_time_ms;
    packet.group         = target_group;
    packet.gain          = gain_u8;

    log_i("PLAY: event_id=\"%s\" hash=0x%08X target_time_ms=%u group=%u gain=%u",
          event_id, event_hash, target_time_ms, target_group, gain_u8);

    bool ok = false;
    if (sender_) {
        if (target_group == 0) {
            ok = sender_->sendBroadcast(packet);
        } else {
            ok = sender_->sendToGroup(packet, target_group);
        }
    }

    displaySendReaction("PLAY", event_id, target_group, ok);

    if (serial_) {
        serial_->sendAck(frame.seq, ok ? ACK_OK : ACK_ERROR);
    }
}

void CommandDispatcher::handleStop(const SerialFrame& frame) {
    // DISPATCH_STOP payload:
    //   event_id (null-terminated string)
    //   target_group (uint8)

    char event_id[128];
    uint16_t consumed = extractString(frame.payload, frame.payload_length, 0, event_id, sizeof(event_id));
    if (consumed == 0) {
        log_e("STOP: failed to extract event_id");
        if (serial_) serial_->sendAck(frame.seq, ACK_ERROR);
        return;
    }

    uint16_t pos = consumed;

    if (pos + 1 > frame.payload_length) {
        log_e("STOP: payload too short");
        if (serial_) serial_->sendAck(frame.seq, ACK_ERROR);
        return;
    }

    uint8_t target_group = frame.payload[pos];

    // Compute CRC32 hash of the event ID string
    uint32_t event_hash = crc32(event_id);

    // Build ESP-NOW stop packet
    EspNowPacket packet;
    packet.command       = ESPNOW_CMD_STOP;
    packet.event_id_hash = event_hash;
    packet.target_time   = 0;
    packet.group         = target_group;
    packet.gain          = 0;

    log_i("STOP: event_id=\"%s\" hash=0x%08X group=%u", event_id, event_hash, target_group);

    bool ok = false;
    if (sender_) {
        if (target_group == 0) {
            ok = sender_->sendBroadcast(packet);
        } else {
            ok = sender_->sendToGroup(packet, target_group);
        }
    }

    displaySendReaction("STOP", event_id, target_group, ok);

    if (serial_) {
        serial_->sendAck(frame.seq, ok ? ACK_OK : ACK_ERROR);
    }
}

void CommandDispatcher::handleStopAll(const SerialFrame& frame) {
    // DISPATCH_STOP_ALL payload:
    //   target_group (uint8)

    if (frame.payload_length < 1) {
        log_e("STOP_ALL: payload too short");
        if (serial_) serial_->sendAck(frame.seq, ACK_ERROR);
        return;
    }

    uint8_t target_group = frame.payload[0];

    // Build ESP-NOW stop_all packet
    EspNowPacket packet;
    packet.command       = ESPNOW_CMD_STOP_ALL;
    packet.event_id_hash = 0;
    packet.target_time   = 0;
    packet.group         = target_group;
    packet.gain          = 0;

    log_i("STOP_ALL: group=%u", target_group);

    bool ok = false;
    if (sender_) {
        if (target_group == 0) {
            ok = sender_->sendBroadcast(packet);
        } else {
            ok = sender_->sendToGroup(packet, target_group);
        }
    }

    displaySendReaction("STOP ALL", nullptr, target_group, ok);

    if (serial_) {
        serial_->sendAck(frame.seq, ok ? ACK_OK : ACK_ERROR);
    }
}

void CommandDispatcher::handleTimeSync(const SerialFrame& frame) {
    // TIME_SYNC payload:
    //   bridge_time (int64, little endian, microseconds)

    if (frame.payload_length < 8) {
        log_e("TIME_SYNC: payload too short (need 8 bytes, have %u)", frame.payload_length);
        if (serial_) serial_->sendAck(frame.seq, ACK_ERROR);
        return;
    }

    int64_t bridge_time_us = 0;
    memcpy(&bridge_time_us, frame.payload, 8);

    if (time_sync_) {
        time_sync_->updateBridgeTime(bridge_time_us);
    }

    log_i("TIME_SYNC: bridge_time_us=%lld", bridge_time_us);

    if (serial_) {
        serial_->sendAck(frame.seq, ACK_OK);
    }
}

void CommandDispatcher::handleQueryStatus(const SerialFrame& frame) {
    // QUERY_STATUS: respond with TX_STATUS

    uint32_t uptime_sec = millis() / 1000;
    uint8_t queue_depth = 0; // No queue implemented yet
    uint8_t last_result = sender_ ? (sender_->getLastResult() ? 0 : 1) : 1;

    log_d("QUERY_STATUS: uptime=%u queue=%u last_result=%u", uptime_sec, queue_depth, last_result);

    if (serial_) {
        serial_->sendTxStatus(queue_depth, last_result, uptime_sec);
    }
}

void CommandDispatcher::handleDeviceScan(const SerialFrame& frame) {
    // DEVICE_SCAN: trigger ESP-NOW scan.
    // Simplified initial implementation: respond with an empty device list.
    // Future: implement active scanning via ESP-NOW probe packets.

    log_i("DEVICE_SCAN: responding with empty device list (not yet implemented)");

    if (serial_) {
        serial_->sendDeviceInfo(nullptr, 0);
    }
}
