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

#endif // DISPLAY_H
