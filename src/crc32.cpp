#include "crc32.h"
#include <cstring>

// Standard CRC32 using reflected polynomial 0xEDB88320.
// Bit-by-bit computation (no lookup table) to minimize RAM usage on ESP32.

static uint32_t crc32_update(uint32_t crc, uint8_t byte) {
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 1) {
            crc = (crc >> 1) ^ 0xEDB88320;
        } else {
            crc >>= 1;
        }
    }
    return crc;
}

uint32_t crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
        crc = crc32_update(crc, data[i]);
    }
    return crc ^ 0xFFFFFFFF;
}

uint32_t crc32(const char* str) {
    return crc32(reinterpret_cast<const uint8_t*>(str), strlen(str));
}
