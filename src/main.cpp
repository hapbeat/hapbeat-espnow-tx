#include <Arduino.h>
#include <M5Stack.h>
#include "config.h"
#include "types.h"
#include "serial_handler.h"
#include "espnow_sender.h"
#include "command_dispatcher.h"
#include "time_sync.h"
#include "display.h"

// Global instances
static SerialHandler      serialHandler;
static EspNowSender       espNowSender;
static TimeSync           timeSync;
static CommandDispatcher   dispatcher;

// Periodic status report timing
static uint32_t lastStatusReport = 0;
static constexpr uint32_t DISPLAY_UPDATE_MS = 500;

// Serial connection timeout: if no frame received within this period, consider disconnected
static constexpr uint32_t SERIAL_CONNECT_TIMEOUT_MS = 5000;

void setup() {
    // Initialize M5Stack (LCD, SD, Serial, I2C)
    M5.begin(true, false, true, false);  // LCD=true, SD=false, Serial=true, I2C=false

    // Initialize display
    displayInit();
    displayBootStatus("Serial init...");

    // Initialize serial communication with the Bridge
    serialHandler.init(Serial);

    log_i("=== Hapbeat Transmitter Firmware ===");
    log_i("Initializing...");

    // Initialize ESP-NOW sender (sets up WiFi STA mode and ESP-NOW)
    displayBootStatus("ESP-NOW init...");
    if (!espNowSender.init()) {
        log_e("ESP-NOW initialization failed! Halting.");
        displayError("ESP-NOW INIT FAILED");
        while (true) {
            delay(1000);
        }
    }
    displayBootStatus("ESP-NOW OK");

    // Initialize command dispatcher with references to all subsystems
    dispatcher.init(serialHandler, espNowSender, timeSync);

    displayBootStatus("Ready. Waiting for Bridge.");
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

    // Periodic status + display update
    uint32_t now = millis();
    if (now - lastStatusReport >= DISPLAY_UPDATE_MS) {
        lastStatusReport = now;
        uint32_t uptime_sec = now / 1000;

        // Update header connection status (any byte received = port is open)
        uint32_t lastRx = serialHandler.getLastRxByteTime();
        bool connected = (lastRx > 0) && (now - lastRx < SERIAL_CONNECT_TIMEOUT_MS);
        displayUpdateHeader(connected);

        // Update LCD status
        displayUpdateStatus(uptime_sec,
                            espNowSender.getTotalSent(),
                            espNowSender.getTotalFailed(),
                            ESPNOW_CHANNEL,
                            timeSync.isSynced());

        // Serial log at lower frequency (every STATUS_REPORT_INTERVAL_MS)
        static uint32_t lastSerialLog = 0;
        if (now - lastSerialLog >= STATUS_REPORT_INTERVAL_MS) {
            lastSerialLog = now;
            log_i("Status: uptime=%us, sent=%u, failed=%u, time_synced=%s",
                  uptime_sec,
                  espNowSender.getTotalSent(),
                  espNowSender.getTotalFailed(),
                  timeSync.isSynced() ? "yes" : "no");
        }
    }
}
