#ifndef COMMAND_DISPATCHER_H
#define COMMAND_DISPATCHER_H

#include "types.h"
#include "serial_handler.h"
#include "espnow_sender.h"
#include "time_sync.h"

// Routes incoming serial frames to the appropriate handler.
// Parses command payloads, builds ESP-NOW packets, and sends responses.
class CommandDispatcher {
public:
    CommandDispatcher();

    // Initialize with references to the serial handler, ESP-NOW sender, and time sync.
    void init(SerialHandler& serial, EspNowSender& sender, TimeSync& timeSync);

    // Process a received serial frame and dispatch to the appropriate handler.
    void processFrame(const SerialFrame& frame);

private:
    // Command handlers
    void handlePlay(const SerialFrame& frame);
    void handleStop(const SerialFrame& frame);
    void handleStopAll(const SerialFrame& frame);
    void handleTimeSync(const SerialFrame& frame);
    void handleQueryStatus(const SerialFrame& frame);
    void handleDeviceScan(const SerialFrame& frame);

    // Helper: extract a null-terminated string from payload at given offset.
    // Returns the number of bytes consumed (including the null terminator).
    // Returns 0 if no null terminator is found within the payload bounds.
    static uint16_t extractString(const uint8_t* payload, uint16_t payload_length,
                                   uint16_t offset, char* out, uint16_t out_size);

    SerialHandler* serial_;
    EspNowSender*  sender_;
    TimeSync*      time_sync_;
};

#endif // COMMAND_DISPATCHER_H
