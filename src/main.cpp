#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "serial_handler.h"
#include "espnow_sender.h"
#include "command_dispatcher.h"
#include "time_sync.h"

// Global instances
static SerialHandler      serialHandler;
static EspNowSender       espNowSender;
static TimeSync           timeSync;
static CommandDispatcher   dispatcher;

// Periodic status report timing
static uint32_t lastStatusReport = 0;

void setup() {
    // Initialize serial communication with the Bridge
    serialHandler.init(Serial);

    log_i("=== Hapbeat Transmitter Firmware ===");
    log_i("Initializing...");

    // Initialize ESP-NOW sender (sets up WiFi STA mode and ESP-NOW)
    if (!espNowSender.init()) {
        log_e("ESP-NOW initialization failed! Halting.");
        while (true) {
            delay(1000);
        }
    }

    // Initialize command dispatcher with references to all subsystems
    dispatcher.init(serialHandler, espNowSender, timeSync);

    log_i("Transmitter ready. Waiting for commands from Bridge.");
    lastStatusReport = millis();
}

void loop() {
    // Read and parse incoming serial data from the Bridge
    serialHandler.update();

    // Process any completely received frames
    SerialFrame frame;
    if (serialHandler.getFrame(frame)) {
        dispatcher.processFrame(frame);
    }

    // Periodic debug status output
    uint32_t now = millis();
    if (now - lastStatusReport >= STATUS_REPORT_INTERVAL_MS) {
        lastStatusReport = now;
        uint32_t uptime_sec = now / 1000;
        log_i("Status: uptime=%us, sent=%u, failed=%u, time_synced=%s",
              uptime_sec,
              espNowSender.getTotalSent(),
              espNowSender.getTotalFailed(),
              timeSync.isSynced() ? "yes" : "no");
    }
}
