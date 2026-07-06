#include <Arduino.h>
#ifdef BOARD_CORES3
#include <M5Unified.h>   // CoreS3: M5Unified (classic M5Stack lib is unsupported)
#else
#include <M5Stack.h>
#endif
#include "config.h"
#include "types.h"
#include "serial_handler.h"
#include "espnow_sender.h"
#include "command_dispatcher.h"
#include "time_sync.h"
#include "display.h"
#include "node_serial_config.h"
#ifdef AUDIO_SOURCE
#include "audio_source.h"
#endif
#ifdef REPEATER
#include "repeater.h"
#endif

// ESP-NOW sender — used by all build variants (Bridge relay / audio source / repeater).
static EspNowSender       espNowSender;

// Bridge-relay-only globals. The AUDIO_SOURCE / REPEATER builds return early and
// never use these; excluding them also keeps the HardwareSerial-based SerialHandler
// out of the CoreS3 build (where Serial is USB-CDC / HWCDC, not HardwareSerial).
#if !defined(AUDIO_SOURCE) && !defined(REPEATER)
#define BRIDGE_RELAY 1
static SerialHandler       serialHandler;
static TimeSync            timeSync;
static CommandDispatcher   dispatcher;
static uint32_t lastStatusReport = 0;
static constexpr uint32_t DISPLAY_UPDATE_MS = 500;
// Serial connection timeout: if no frame received within this period, consider disconnected
static constexpr uint32_t SERIAL_CONNECT_TIMEOUT_MS = 5000;
#endif

void setup() {
    // Initialize the M5 board.
#ifdef BOARD_CORES3
    // CoreS3: M5Unified auto-detects the board and brings up the AXP2101 PMIC
    // (which powers the M-Bus / Module Audio), the display, and internal I2C.
    auto m5cfg = M5.config();
    m5cfg.serial_baudrate = SERIAL_BAUD;
    M5.begin(m5cfg);
#else
    // Classic Core/Basic (M5Stack library).
    M5.begin(true, false, true, false);  // LCD=true, SD=false, Serial=true, I2C=false
#endif

    displayInit();
    displayBootStatus("Serial init...");

#ifdef AUDIO_SOURCE
    // ESP-NOW live audio source (DEC-034): line-in → ADPCM → broadcast.
    // EspNowSender.init() provides WiFi STA + channel (from NVS) + esp_now_init
    // + broadcast peer registration. audioSourceSetup() then overrides the send
    // callback and initialises I2S.
    Serial.begin(SERIAL_BAUD);
    displayBootStatus("ESP-NOW init...");
    if (!espNowSender.init()) {
        displayError("ESP-NOW INIT FAILED");
        while (true) delay(1000);
    }
    audioSourceSetup();
    displayBootStatus("AUDIO SOURCE ready");
    return;
#endif

#ifdef REPEATER
    // ESP-NOW repeater (DEC-033): receives 0xAA from source MAC, re-broadcasts.
    // Same init path as AUDIO_SOURCE: EspNowSender provides the ESP-NOW base,
    // repeaterSetup() registers the receive callback.
    Serial.begin(SERIAL_BAUD);
    displayBootStatus("ESP-NOW init...");
    if (!espNowSender.init()) {
        displayError("ESP-NOW INIT FAILED");
        while (true) delay(1000);
    }
    repeaterSetup();
    displayBootStatus("REPEATER ready");
    return;
#endif

#ifdef BRIDGE_RELAY
    // ---- Bridge command relay path ----------------------------------------
    serialHandler.init(Serial);

    log_i("=== Hapbeat Transmitter Firmware ===");
    log_i("Initializing...");

    displayBootStatus("ESP-NOW init...");
    if (!espNowSender.init()) {
        log_e("ESP-NOW initialization failed! Halting.");
        displayError("ESP-NOW INIT FAILED");
        while (true) {
            delay(1000);
        }
    }
    displayBootStatus("ESP-NOW OK");

    dispatcher.init(serialHandler, espNowSender, timeSync);

    displayBootStatus("Ready. Waiting for Bridge.");
    log_i("Transmitter ready. Waiting for commands from Bridge.");
    lastStatusReport = millis();
#endif // BRIDGE_RELAY
}

void loop() {
    // Studio JSON config (USB Web Serial) — consumes '{'-prefixed lines,
    // leaves binary Bridge frames untouched (DEC-034 serial-config).
    nodeSerialConfigUpdate();

#ifdef AUDIO_SOURCE
    audioSourceLoop();
    return;
#endif

#ifdef REPEATER
    repeaterLoop();
    return;
#endif

#ifdef BRIDGE_RELAY
    // ---- Bridge relay path ------------------------------------------------
    serialHandler.update();

    SerialFrame frame;
    if (serialHandler.getFrame(frame)) {
        dispatcher.processFrame(frame);
    }

    uint32_t now = millis();
    if (now - lastStatusReport >= DISPLAY_UPDATE_MS) {
        lastStatusReport = now;
        uint32_t uptime_sec = now / 1000;

        uint32_t lastRx = serialHandler.getLastRxByteTime();
        bool connected = (lastRx > 0) && (now - lastRx < SERIAL_CONNECT_TIMEOUT_MS);
        displayUpdateHeader(connected);

        displayUpdateStatus(uptime_sec,
                            espNowSender.getTotalSent(),
                            espNowSender.getTotalFailed(),
                            ESPNOW_CHANNEL,
                            timeSync.isSynced());

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
#endif // BRIDGE_RELAY
}
