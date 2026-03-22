#ifndef CRC32_H
#define CRC32_H

#include <cstdint>

// Compute CRC32 of a null-terminated string.
// Uses standard CRC32 polynomial (0xEDB88320, reflected).
// This converts Event ID strings to the 4-byte hash used in ESP-NOW packets.
uint32_t crc32(const char* str);

// Compute CRC32 of a byte buffer with explicit length.
uint32_t crc32(const uint8_t* data, size_t length);

#endif // CRC32_H
