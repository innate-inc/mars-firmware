#include <Arduino.h>
#include "BatteryMonitor.h"

BatteryMonitor batt;  // defaults: pin 19, r1=3.3, r2=14.7

void setup() {
  Serial.begin(115200);
}

void loop() {
  float voltage = batt.getVoltage();
  Serial.print("Battery Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");
  delay(1000);
}
