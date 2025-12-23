// src/Utils.h
#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

/**
 * Calculate CRC-8/MAXIM checksum over data buffer
 * Polynomial: 0x8C, Initial value: 0x00
 * Used for motor command and I2C packet validation
 */
uint8_t calculateCRC8Maxim(const uint8_t *data, size_t len);

/**
 * Combine two bytes into a signed 16-bit integer (big-endian)
 * @param high Most significant byte
 * @param low Least significant byte
 * @return Combined 16-bit signed integer
 */
inline int16_t combineInt16(uint8_t high, uint8_t low) {
  return (int16_t)((high << 8) | low);
}

/**
 * Split 16-bit signed integer into high and low bytes
 * @param value 16-bit value to split
 * @param high Reference to store high byte
 * @param low Reference to store low byte
 */
inline void splitInt16(int16_t value, uint8_t &high, uint8_t &low) {
  high = (value >> 8) & 0xFF;
  low = value & 0xFF;
}

/**
 * Convert battery voltage reading to scaled integer for transmission
 * @param voltage Voltage in volts (e.g., 12.34)
 * @return Scaled voltage * 100 (e.g., 1234)
 */
inline uint16_t scaleBatteryVoltage(float voltage) {
  return (uint16_t)(voltage * 100.0f);
}

/**
 * Convert scaled integer back to battery voltage
 * @param scaled_voltage Scaled voltage from transmission
 * @return Actual voltage in volts
 */
inline float unscaleBatteryVoltage(uint16_t scaled_voltage) {
  return scaled_voltage / 100.0f;
}

#endif // UTILS_H