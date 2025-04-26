// ===== LEDControl.h =====
#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// LED operating modes
enum class LedMode : uint8_t { Off = 0, Solid, Blink, Ring };

class LEDControl {
public:
  // Constructor: specify data pin and number of LEDs
  LEDControl(uint8_t pin, uint16_t numLeds);

  // Initialize the strip (call in setup)
  void begin();

  // Set mode, interval (ms), and RGB color
  void setMode(LedMode mode, uint16_t intervalMs, uint8_t r, uint8_t g, uint8_t b);

  // Call in loop() to update animations
  void update();

private:
  Adafruit_NeoPixel strip;
  LedMode    mode;
  uint16_t   interval;
  uint8_t    red, green, blue;
  unsigned long prevMillis;
  bool       blinkState;
  uint16_t   currentIndex;
};