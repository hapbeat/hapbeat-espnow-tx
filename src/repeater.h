#ifndef REPEATER_H
#define REPEATER_H

// ---------------------------------------------------------------------------
// repeater.h — ESP-NOW repeater mode (DEC-033, -D REPEATER)
//
// Receives 0xAA stream packets from a single configured source MAC and
// re-broadcasts them verbatim over ESP-NOW. Configured from Studio over USB
// Web Serial (set_relay_source / set_espnow_channel).
//
// See contracts/specs/espnow-stream.md §7.2 for the full spec.
//
// main.cpp dispatches setup()/loop() here when REPEATER is defined.
// ---------------------------------------------------------------------------

#include <cstdint>

// Core lifecycle (called from main.cpp).
void repeaterSetup();
void repeaterLoop();

// Live-update the relay source MAC (called from node_serial_config after NVS write).
// mac: 6-byte array, big-endian order (same as WiFi.macAddress bytes).
void repeaterSetRelaySrc(const uint8_t* mac);

// Read back the currently configured relay source MAC.
// Returns false if not yet configured.
bool repeaterGetRelaySrc(uint8_t* out_mac);

#endif // REPEATER_H
