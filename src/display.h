#ifndef DISPLAY_H
#define DISPLAY_H

#include <cstdint>

// Initialize M5Stack LCD display
void displayInit();

// Show boot progress message
void displayBootStatus(const char* msg);

// Update header bar with serial connection status
void displayUpdateHeader(bool connected);

// Update the main status screen (call periodically, e.g. every 500ms)
// Shows uptime, sent/failed counts, WiFi channel, time sync status
void displayUpdateStatus(uint32_t uptime_sec, uint32_t sent, uint32_t failed,
                         uint8_t wifi_ch, bool time_synced);

// Flash a send reaction on screen (call when ESP-NOW packet is sent)
// cmd: "PLAY", "STOP", "STOP_ALL"
// event_id: event name (nullable)
// group: target group
// success: whether send succeeded
void displaySendReaction(const char* cmd, const char* event_id,
                         uint8_t group, bool success);

// Show an error message briefly
void displayError(const char* msg);

// Audio source mode stats display (~4 fps).
// ch: ESP-NOW channel, level: input_level 0-100, pkt: total sent,
// inflight: current in-flight count, drop: dropped (busy), fail: send errors.
void displayUpdateAudioStats(uint8_t ch, int level, uint32_t pkt,
                              int inflight, uint32_t drop, uint32_t fail);

// Repeater mode stats display (~4 fps).
// ch: ESP-NOW channel, relay_mac: source MAC string ("AA:BB:CC:DD:EE:FF" or "not set"),
// relayed: packets relayed, dropped: packets ignored (wrong src or pending slot full).
void displayUpdateRepeaterStats(uint8_t ch, const char* relay_mac,
                                 uint32_t relayed, uint32_t dropped);

#endif // DISPLAY_H
