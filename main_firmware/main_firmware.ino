#include <Arduino.h>
#include "LEDControl.h"

#define LED_PIN     2    // change to your data pin
#define NUM_LEDS    3   // change to your strip length

// instantiate your LED controller
LEDControl leds(LED_PIN, NUM_LEDS);

// human-readable names for the modes
const char* modeNames[] = { "Off", "Solid", "Blink", "Ring" };
const uint8_t numModes = 4;

uint8_t currentMode   = 0;
unsigned long lastChange  = 0;
const unsigned long changeInterval = 5000; // 2 seconds

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(A0));   // seed RNG
  
  leds.begin();
  // initialize first mode/color immediately
  changeMode();
}

void loop() {
  leds.update();

  unsigned long now = millis();
  if (now - lastChange >= changeInterval) {
    changeMode();
    lastChange = now;
  }
}

void changeMode() {
  // cycle through modes
  currentMode = (currentMode + 1) % numModes;

  // pick a random color
  uint8_t r = random(0, 256);
  uint8_t g = random(0, 256);
  uint8_t b = random(0, 256);

  // apply new mode & color, use 500ms interval for blink/ring
  leds.setMode(
    static_cast<LedMode>(currentMode),
    500,
    r, g, b
  );

  // print to serial
  Serial.print("Mode: ");
  Serial.print(modeNames[currentMode]);
  Serial.print("  Color: R=");
  Serial.print(r);
  Serial.print(" G=");
  Serial.print(g);
  Serial.print(" B=");
  Serial.println(b);
}
