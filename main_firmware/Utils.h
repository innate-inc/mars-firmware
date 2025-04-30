// ===== Utils.h =====
#pragma once
#include <Arduino.h>

// Calculate CRC-8/MAXIM over data buffer
// polynomial 0x8C, initial value 0x00
uint8_t calculateCRC8Maxim(const uint8_t *data, size_t len);

// Combine two bytes into a signed 16-bit integer
inline int16_t combineInt16(uint8_t high, uint8_t low) {
  return (int16_t)((high << 8) | low);
}