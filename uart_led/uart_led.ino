#include <Adafruit_NeoPixel.h>

// ---------- NeoPixel Configuration ----------
#define LED_PIN     2       // Pin where the NeoPixel strip is connected
#define NUM_LEDS    7       // Total number of LEDs in the strip

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------- Global Variables for LED Command ----------
uint8_t ledMode   = 0;        // LED mode: 0 = Off, 1 = Solid, 2 = Blink, 3 = Ring
uint16_t timeParam = 300;     // Time parameter (milliseconds)
uint8_t redVal    = 0;        // Red intensity (0–255)
uint8_t greenVal  = 0;        // Green intensity (0–255)
uint8_t blueVal   = 0;        // Blue intensity (0–255)

// Variables used for animations in Blink and Ring modes:
unsigned long previousMillis = 0; // For timing animations
int currentLEDIndex = 0;            // For Ring mode: currently lit LED index
bool blinkState = false;            // For Blink mode: current on/off state

// ---------- UART Message Parsing Variables ----------
/*
  Each complete message is 10 bytes long:
    - Bytes 0-1: Start-of-Message (SOM) marker: 0x69 0x69
    - Bytes 2-9: Protocol message:
         * Byte 2: Command ID (for LED command, 0x02)
         * Byte 3: LED Mode
         * Bytes 4-5: Time Parameter (16-bit unsigned; high byte, then low byte)
         * Byte 6: Red intensity
         * Byte 7: Green intensity
         * Byte 8: Blue intensity
         * Byte 9: CRC (CRC‑8/MAXIM computed over bytes 2–8)
  
  When reading from Serial1, we ignore the SOM and then store the next 8 bytes.
*/
enum ParseState {
  WAIT_FOR_SOM1,
  WAIT_FOR_SOM2,
  READ_PAYLOAD
};

ParseState parseState = WAIT_FOR_SOM1;
uint8_t payloadBuffer[8];  // Will hold bytes: [CmdID, LEDMode, TimeHi, TimeLo, Red, Green, Blue, CRC]
int payloadIndex = 0;

// ---------- CRC‑8/MAXIM Calculation Function ----------
/*
  Computes CRC‑8/MAXIM using:
    - Polynomial (reflected): 0x8C
    - Initial value: 0x00
  The CRC is computed over the 7-byte payload (bytes 2–8 of the full message).
*/
uint8_t calculateCRC8Maxim(const uint8_t *data, int len) {
  uint8_t crc = 0x00;
  for (int j = 0; j < len; j++) {
    crc ^= data[j];
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0x8C;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// ---------- Function to Send LED Response (Acknowledgment) ----------
/*
  Constructs and transmits a 10-byte LED Response message over Serial1.
  
  Response Message Format:
    - Bytes 0-1: SOM marker (0x69, 0x69)
    - Byte 2: Response ID (0x82, since LED command is 0x02 and response = command + 0x80)
    - Byte 3: LED Mode (current)
    - Byte 4: Red intensity (current)
    - Byte 5: Green intensity (current)
    - Byte 6: Blue intensity (current)
    - Byte 7: Time Parameter high byte
    - Byte 8: Time Parameter low byte
    - Byte 9: CRC computed over bytes 2–8
*/
void sendLEDResponse() {
  uint8_t response[10];
  
  // Start-of-Message marker:
  response[0] = 0x69;
  response[1] = 0x69;
  
  // Build the protocol message:
  response[2] = 0x82;                 // Response ID = 0x02 + 0x80
  response[3] = ledMode;              // Current LED Mode
  response[4] = redVal;               // Current Red intensity
  response[5] = greenVal;             // Current Green intensity
  response[6] = blueVal;              // Current Blue intensity
  response[7] = (timeParam >> 8) & 0xFF; // Time parameter high byte
  response[8] = timeParam & 0xFF;        // Time parameter low byte
  
  // Compute CRC over bytes 2 through 8 (7 bytes)
  response[9] = calculateCRC8Maxim(response + 2, 7);
  
  // Transmit the complete response message over Serial1:
  for (int i = 0; i < 10; i++) {
    Serial1.write(response[i]);
  }
}

// ---------- Function to Process Incoming Serial1 Data ----------
void processSerial() {
  while (Serial1.available() > 0) {
    uint8_t byteRead = Serial1.read();
    
    switch (parseState) {
      case WAIT_FOR_SOM1:
        if (byteRead == 0x69) {
          parseState = WAIT_FOR_SOM2;
        }
        break;
        
      case WAIT_FOR_SOM2:
        if (byteRead == 0x69) {
          // SOM marker confirmed; start reading the payload
          parseState = READ_PAYLOAD;
          payloadIndex = 0;
        } else {
          // Not a valid SOM sequence; reset state
          parseState = WAIT_FOR_SOM1;
        }
        break;
        
      case READ_PAYLOAD:
        payloadBuffer[payloadIndex++] = byteRead;
        if (payloadIndex == 8) {
          // We have received the 8 payload bytes.
          // Compute CRC over the first 7 bytes.
          uint8_t computedCRC = calculateCRC8Maxim(payloadBuffer, 7);
          
          if (computedCRC == payloadBuffer[7]) {
            // Valid message received.
            // Process LED Command (only if command ID is 0x02)
            if (payloadBuffer[0] == 0x02) {  // LED Command
              // Extract LED command parameters:
              ledMode   = payloadBuffer[1];
              timeParam = ((uint16_t)payloadBuffer[2] << 8) | payloadBuffer[3];
              redVal    = payloadBuffer[4];
              greenVal  = payloadBuffer[5];
              blueVal   = payloadBuffer[6];
              
              // Reset animation timing variables:
              previousMillis = millis();
              blinkState = false;
              currentLEDIndex = 0;
              
              // Immediately update the LED strip for Off or Solid modes:
              if (ledMode == 0) {       // Off mode: clear the strip.
                strip.clear();
                strip.show();
              }
              else if (ledMode == 1) {  // Solid mode: fill all LEDs.
                for (int i = 0; i < NUM_LEDS; i++) {
                  strip.setPixelColor(i, strip.Color(redVal, greenVal, blueVal));
                }
                strip.show();
              }
              // For Blink (2) and Ring (3) modes, the update will be done in updateLEDs().
              
              // Send back an acknowledgment LED Response message:
              sendLEDResponse();
            }
          }
          // Reset parser state to wait for the next message
          parseState = WAIT_FOR_SOM1;
        }
        break;
    }
  }
}

// ---------- Function to Update LED Animations ----------
void updateLEDs() {
  unsigned long currentMillis = millis();
  
  switch (ledMode) {
    case 0: // Off mode – nothing to animate.
      break;
      
    case 1: // Solid mode – static color already applied.
      break;
      
    case 2: // Blink mode – alternate between on and off.
      if (currentMillis - previousMillis >= timeParam) {
        previousMillis = currentMillis;
        blinkState = !blinkState;
        
        if (blinkState) {
          // Turn all LEDs on with the specified color.
          for (int i = 0; i < NUM_LEDS; i++) {
            strip.setPixelColor(i, strip.Color(redVal, greenVal, blueVal));
          }
        } else {
          // Turn off all LEDs.
          strip.clear();
        }
        strip.show();
      }
      break;
      
    case 3: // Ring mode – one LED lit at a time moving around the strip.
      if (currentMillis - previousMillis >= timeParam) {
        previousMillis = currentMillis;
        // Move to the next LED (wrap around using modulo)
        currentLEDIndex = (currentLEDIndex + 1) % NUM_LEDS;
        strip.clear();
        strip.setPixelColor(currentLEDIndex, strip.Color(redVal, greenVal, blueVal));
        strip.show();
      }
      break;
      
    default:
      // Unrecognized mode; do nothing.
      break;
  }
}

// ---------- Arduino Standard Functions ----------
void setup() {
  // Initialize the LED strip:
  strip.begin();
  strip.clear();
  strip.show();
  
  // Start Serial1 with UART settings: 115200 baud, 8 data bits, no parity, 1 stop bit.
  Serial1.begin(115200);
}

void loop() {
  processSerial();  // Process any incoming messages on Serial1.
  updateLEDs();     // Update LED animations (if in Blink or Ring mode).
}

