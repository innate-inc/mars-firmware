// ===== BatteryMonitor.cpp =====
#include "BatteryMonitor.h"

BatteryMonitor::BatteryMonitor(uint8_t pin, float r1, float r2)
  : _pin(pin), _r1(r1), _r2(r2),
    _vref(3.3f), _maxAdc(1023.0f)  // using 12-bit ADC and 3.3V ref
{
  // Compute scale: Vbat = Vmeas * ((r1 + r2) / r1)
  _scale = (_r2 + _r1) / _r1;
}


float BatteryMonitor::getVoltage() const {
  uint32_t raw = analogRead(_pin);
  float measured = (raw / _maxAdc) * _vref;
  return measured*_scale;
}