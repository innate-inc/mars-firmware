// ===== BatteryMonitor.h =====
#pragma once
#include <Arduino.h>

// Simple battery voltage monitor using a resistor divider
class BatteryMonitor {
public:
  // pin: analog input pin (default 19)
  // r1: bottom resistor of divider (default 3.3)
  // r2: top resistor of divider (default 14.7)
  BatteryMonitor(uint8_t pin = 19, float r1 = 4.7f, float r2 = 13.3f);


  // Read and return the battery voltage (V)
  float getVoltage() const;

private:
  uint8_t _pin;
  float   _r1;
  float   _r2;
  float   _scale;   // multiplier ((r1+r2)/r1)
  float   _vref;    // ADC reference voltage
  float   _maxAdc;  // max ADC value
};