#include "LEDControl.h"

LEDControl::LEDControl(uint8_t pin = 2, uint16_t numLeds = 39)
  : strip(numLeds, pin, NEO_GRB + NEO_KHZ800),
    mode(LedMode::Off), interval(300),
    red(0), green(0), blue(0),
    prevMillis(0), blinkState(false), currentIndex(0)
{}

void LEDControl::begin() {
  strip.begin();
  strip.clear();
  strip.show();
  prevMillis = millis();
  blinkState = false;
  currentIndex = 0;
}

void LEDControl::setMode(LedMode m, uint16_t intervalMs, uint8_t r, uint8_t g, uint8_t b) {
  mode = m;
  interval = intervalMs;
  red = r; green = g; blue = b;
  prevMillis = millis();
  blinkState = false;
  currentIndex = 0;

  // Immediately apply Off or Solid
  if (mode == LedMode::Off) {
    strip.clear();
    strip.show();
  } else if (mode == LedMode::Solid) {
    for (uint16_t i = 0; i < strip.numPixels(); ++i) {
      strip.setPixelColor(i, strip.Color(red, green, blue));
    }
    strip.show();
  }
}

void LEDControl::update() {
  unsigned long now = millis();
  switch (mode) {
    case LedMode::Blink:
      if (now - prevMillis >= interval) {
        prevMillis = now;
        blinkState = !blinkState;
        if (blinkState) {
          for (uint16_t i = 0; i < strip.numPixels(); ++i)
            strip.setPixelColor(i, strip.Color(red, green, blue));
        } else {
          strip.clear();
        }
        strip.show();
      }
      break;

    case LedMode::Ring:
      if (now - prevMillis >= interval) {
        prevMillis = now;
        currentIndex = (currentIndex + 1) % strip.numPixels();
        strip.clear();
        strip.setPixelColor(currentIndex, strip.Color(red, green, blue));
        strip.show();
      }
      break;

    default:
      // Off and Solid are static
      break;
  }
}