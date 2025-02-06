#include <Adafruit_NeoPixel.h>

#define LED_PIN     2  // Pin where LED strip is connected
#define NUM_LEDS    7  // Total number of LEDs in the ring
#define DELAY_MS    300 // Delay time for animation

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.clear(); // Turn off all LEDs
    strip.setPixelColor(i, strip.Color(0, 139, 139)); // Set current LED to specified color
    strip.show();
    delay(DELAY_MS);
  }
}
