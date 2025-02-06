/*
  Joystick to Serial1 Movement Command Publisher with LED Command on Button Press
  --------------------------------------------------------------------------------
  - Joystick:
      * A2: forward/backward speed axis
      * A3: turning axis
  - Deadband: 0.1 V around 1.65 V (neutral)
  - Mapping:
      * Speed: ±1.28 (scaled to ±128 after multiplying by 100)
      * Turn:  ±2.56 (scaled to ±256 after multiplying by 100)
  - Movement Message Format (8 bytes):
      Byte 0: Command ID (0x01)
      Bytes 1-2: Forward Speed (16-bit signed fixed-point, scaled by 100; big-endian)
      Bytes 3-4: Turn Rate (16-bit signed fixed-point, scaled by 100; big-endian)
      Bytes 5-6: Reserved (0x00)
      Byte 7: CRC‑8/MAXIM computed over bytes 0–6.
  - LED Command Message Format (8 bytes):
      Byte 0: Command ID (0x02)
      Byte 1: LED Mode (3 = Ring mode)
      Bytes 2-3: Time Parameter (16-bit unsigned, in ms; 100 ms → 0x00 0x64)
      Byte 4: Red intensity (0–255, here 0xFF for red)
      Byte 5: Green intensity (0–255, here 0x00)
      Byte 6: Blue intensity (0–255, here 0x00)
      Byte 7: CRC‑8/MAXIM computed over bytes 0–6.
  - The messages are sent over Serial1 (hardware UART on RX/TX pins).
  - Debug output is printed to Serial (USB CDC).
*/

#include <Arduino.h>

// ADC configuration (using a 10-bit ADC: 0-1023)
const int ADC_MAX = 1023;
const float VREF = 3.3;  // in volts

// Joystick calibration values
const float centerVoltage = 1.65;  // Neutral center voltage
const float deadband = 0.1;        // Deadband in volts
const float effectiveRange = centerVoltage - deadband;  // = 1.55 V

// Maximum command values
const float maxSpeed = 1.28;   // Maximum forward/backward speed
const float maxTurn  = 2.56;   // Maximum turn rate

// Pin definitions
// A2 and A3 are used for the joystick analog inputs.
// A4 is used for the button input.
const int buttonPin = A4;

// Function Prototypes
float mapJoystick(float voltage, float center, float deadband, float effective, float maxCommand);
uint8_t calculateCRC8Maxim(uint8_t *data, int len);
void sendMovementCommand(float speed, float turn);
void sendLEDCommand();

void setup() {
  // Initialize USB Serial for debugging
  Serial.begin(115200);
  while (!Serial) { ; } // Wait for USB serial connection

  // Initialize hardware UART (Serial1) for command transmission on RX/TX pins (#0 and #1)
  Serial1.begin(115200);

  // Configure the button pin with internal pull-up.
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.println("Starting Joystick and LED Command Publisher...");
}

void loop() {
  // --- 1. Read Joystick Values and Send Movement Command ---
  int rawA2 = analogRead(A2);   // Speed axis
  int rawA3 = analogRead(A3);   // Turn axis

  float voltageA2 = (rawA2 * VREF) / ADC_MAX;
  float voltageA3 = (rawA3 * VREF) / ADC_MAX;

  // Map the joystick voltages to command values
  float speedCommand = mapJoystick(voltageA2, centerVoltage, deadband, effectiveRange, maxSpeed);
  float turnCommand  = mapJoystick(voltageA3, centerVoltage, deadband, effectiveRange, maxTurn);

  // Send the movement command over Serial1
  sendMovementCommand(speedCommand, turnCommand);

  // --- 2. Check Button State and Send LED Command on Press ---
  // We'll detect a transition from unpressed (HIGH) to pressed (LOW).
  static int lastButtonState = HIGH;
  int currentButtonState = digitalRead(buttonPin);
  if (currentButtonState != lastButtonState) {
    if (currentButtonState == LOW) {
      // Button pressed: send LED command to set light to ring mode (100ms, red)
      sendLEDCommand();
    }
    lastButtonState = currentButtonState;
  }

  // --- 3. Debug Output ---
  Serial.print("A2 Voltage: ");
  Serial.print(voltageA2, 3);
  Serial.print(" V  =>  Speed: ");
  Serial.print(speedCommand, 3);
  Serial.print("\tA3 Voltage: ");
  Serial.print(voltageA3, 3);
  Serial.print(" V  =>  Turn: ");
  Serial.println(turnCommand, 3);

  delay(500);  // 500 ms loop delay
}

/*
 * mapJoystick()
 *
 * Converts a joystick voltage reading into a command value.
 * If the reading is within the deadband (±deadband around center), returns 0.
 * Otherwise, scales the deviation (beyond deadband) linearly to ±maxCommand.
 */
float mapJoystick(float voltage, float center, float deadband, float effective, float maxCommand) {
  float delta = voltage - center;
  if (abs(delta) < deadband) {
    return 0.0;
  }
  if (delta > deadband) {
    return ((delta - deadband) / effective) * maxCommand;
  } else { // delta < -deadband
    return ((delta + deadband) / effective) * maxCommand;
  }
}

/*
 * calculateCRC8Maxim()
 *
 * Computes the CRC‑8/MAXIM checksum over a block of data.
 * Uses a reflected polynomial (0x8C) and an initial value of 0x00.
 * The CRC is computed over the first 'len' bytes of the data array.
 */
uint8_t calculateCRC8Maxim(uint8_t *data, int len) {
  uint8_t crc = 0x00;
  for (int j = 0; j < len; j++) {
    crc ^= data[j];
    for (int i = 0; i < 8; i++) {
      if (crc & 0x01) {
        crc = (crc >> 1) ^ 0x8C;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

/*
 * sendMovementCommand()
 *
 * Prepares and sends an 8-byte movement command over Serial1.
 * The message structure is:
 *   Byte 0:  Command ID (0x01)
 *   Bytes 1-2: Forward Speed (16-bit signed, fixed-point scaled by 100; big-endian)
 *   Bytes 3-4: Turn Rate (16-bit signed, fixed-point scaled by 100; big-endian)
 *   Bytes 5-6: Reserved (0x00)
 *   Byte 7:  CRC‑8/MAXIM computed over bytes 0–6.
 */
void sendMovementCommand(float speed, float turn) {
  uint8_t message[8];

  message[0] = 0x01;  // Movement command ID

  // Convert speed to fixed-point (scale by 100)
  int16_t speedVal = (int16_t)(speed * 100);
  message[1] = (uint8_t)((speedVal >> 8) & 0xFF);  // High byte
  message[2] = (uint8_t)(speedVal & 0xFF);           // Low byte

  // Convert turn to fixed-point (scale by 100)
  int16_t turnVal = (int16_t)(turn * 100);
  message[3] = (uint8_t)((turnVal >> 8) & 0xFF);     // High byte
  message[4] = (uint8_t)(turnVal & 0xFF);              // Low byte

  // Reserved bytes
  message[5] = 0x00;
  message[6] = 0x00;

  // Compute the CRC over the first 7 bytes
  message[7] = calculateCRC8Maxim(message, 7);

  // Send the message over Serial1 (UART)
  Serial1.write(message, 8);

  // Debug: Print the message bytes in hexadecimal
  Serial.print("Sent Movement Message: ");
  for (int i = 0; i < 8; i++) {
    if (message[i] < 0x10) Serial.print("0");
    Serial.print(message[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

/*
 * sendLEDCommand()
 *
 * Prepares and sends an 8-byte LED command over Serial1.
 * The message structure is:
 *   Byte 0: Command ID (0x02)
 *   Byte 1: LED Mode (3 for Ring mode)
 *   Bytes 2-3: Time Parameter (16-bit unsigned, in ms; 100 ms → 0x00, 0x64)
 *   Byte 4: Red intensity (0xFF for red)
 *   Byte 5: Green intensity (0x00)
 *   Byte 6: Blue intensity (0x00)
 *   Byte 7: CRC‑8/MAXIM computed over bytes 0–6.
 */
void sendLEDCommand() {
  uint8_t message[8];

  message[0] = 0x02;  // LED command ID
  message[1] = 0x03;  // LED Mode: 3 = Ring mode
  message[2] = 0x00;  // Time Parameter high byte (100 ms)
  message[3] = 0x64;  // Time Parameter low byte (100 ms)
  message[4] = 0xFF;  // Red intensity (255)
  message[5] = 0x00;  // Green intensity
  message[6] = 0x00;  // Blue intensity

  // Compute the CRC over the first 7 bytes
  message[7] = calculateCRC8Maxim(message, 7);

  // Send the LED command over Serial1
  Serial1.write(message, 8);

  // Debug: Print the LED command message in hexadecimal
  Serial.print("Sent LED Command: ");
  for (int i = 0; i < 8; i++) {
    if (message[i] < 0x10) Serial.print("0");
    Serial.print(message[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
