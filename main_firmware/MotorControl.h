// src/MotorControl.h
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include "wiring_private.h"    // for pinPeripheral()

class MotorControl {
public:
  /**
   * @param sercomNum  SERCOM index (0 or 4) to bind interrupts and select SERCOM instance
   * @param rxPin      Pin number for RX
   * @param txPin      Pin number for TX
   * @param rxPad      SERCOM RX pad
   * @param txPad      SERCOM TX pad
   */
  MotorControl(uint8_t sercomNum,
               uint8_t rxPin, uint8_t txPin,
               uint8_t rxPad, uint8_t txPad);

  /**
   * Initialize UART, pins, and set motor ID & speed-loop mode.
   */
  void begin(uint32_t baud = 115200);

  /** Send a new drive speed command [-MAX...+MAX] */
  void sendCommand(int16_t speed);

  /** Re-send last drive command to maintain motion */
  void maintain();

  /** Process incoming bytes to update RPM feedback */
  void processFeedback();

  /** True if a fresh RPM reading is available */
  bool available();

  /** Get the most recent RPM reading */
  float getRPM() const;

private:
  Uart*       _serial;       // allocated based on SERCOM number
  uint8_t     _id;           // motor ID (default=1)
  int16_t     _lastSpeed;
  float       _lastRPM;
  bool        _newRPM;

  static const size_t BUFFER_SIZE = 64;
  uint8_t     _buffer[BUFFER_SIZE];
  size_t      _bufIndex;

  uint8_t     _sercomNum;
  static MotorControl* _instances[8];

  uint8_t calculateCRC(const uint8_t* data, int len);

  // Friend declarations for interrupt handlers
  friend void SERCOM0_0_Handler();
  friend void SERCOM0_1_Handler();
  friend void SERCOM0_2_Handler();
  friend void SERCOM0_3_Handler();
  friend void SERCOM4_0_Handler();
  friend void SERCOM4_1_Handler();
  friend void SERCOM4_2_Handler();
  friend void SERCOM4_3_Handler();
};

// C linkage for interrupt handlers
extern "C" {
  void SERCOM0_0_Handler();
  void SERCOM0_1_Handler();
  void SERCOM0_2_Handler();
  void SERCOM0_3_Handler();
  void SERCOM4_0_Handler();
  void SERCOM4_1_Handler();
  void SERCOM4_2_Handler();
  void SERCOM4_3_Handler();
}

#endif // MOTORCONTROL_H

