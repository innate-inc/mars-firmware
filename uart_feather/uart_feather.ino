/*
  Joystick to Serial1 Command Publisher with Updated Protocol (10-byte messages)
  ------------------------------------------------------------------------------
  Protocol Summary:
    - Total Message Length: 10 bytes
      • Bytes 0-1: SOM marker (0x69, 0x69)
      • Bytes 2-8: Protocol message (7 bytes payload)
      • Byte 9: CRC‑8/MAXIM over bytes 2–8
  Movement Command (0x01):
    • Bytes 3-4: Forward Speed (16-bit signed fixed-point, scale×100; big-endian)
    • Bytes 5-6: Turn Rate (16-bit signed fixed-point, scale×100; big-endian)
    • Bytes 7-8: Reserved (0x00)
  LED Command (0x02):
    • Byte 3: LED Mode (3 for Ring mode)
    • Bytes 4-5: Time Parameter (16-bit unsigned, 100 ms; high byte then low byte)
    • Byte 6: Red intensity (0xFF)
    • Byte 7: Green intensity (0x00)
    • Byte 8: Blue intensity (0x00)
  
  The messages are sent over Serial1 (hardware UART on RX/TX), and debug info is printed to Serial.
*/

#include <Arduino.h>

// ADC configuration (assuming a 10-bit ADC: 0-1023)
const int ADC_MAX = 1023;
const float VREF = 3.3;  // Reference voltage in volts

// Joystick calibration
const float centerVoltage = 1.65;  // Neutral voltage
const float deadband = 0.1;        // Deadband around center in volts
const float effectiveRange = centerVoltage - deadband;  // = 1.55 V

// Maximum command values (in volts)
const float maxSpeed = 1.28;   // Forward/backward speed range ±1.28
const float maxTurn  = 2.56;   // Turn rate range ±2.56

// Pin definition for the button (used for triggering the LED command)
const int buttonPin = A4;

// Function Prototypes
float mapJoystick(float voltage, float center, float deadband, float effective, float maxCommand);
uint8_t calculateCRC8Maxim(uint8_t *data, int len);
void sendMovementCommand(float speed, float turn);
void sendLEDCommand();

void setup() {
  // Initialize USB serial for debugging
  Serial.begin(115200);
  while (!Serial) { ; } // Wait for USB serial connection

  // Initialize hardware UART (Serial1) on RX/TX pins
  Serial1.begin(115200);

  // Configure the button pin with an internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.println("Starting Joystick and LED Command Publisher with Updated Protocol...");
}

void loop() {
  // --- 1. Read Joystick Values and Send Movement Command ---
  int rawA2 = analogRead(A2);  // Speed axis
  int rawA3 = analogRead(A3);  // Turn axis

  float voltageA2 = (rawA2 * VREF) / ADC_MAX;
  float voltageA3 = (rawA3 * VREF) / ADC_MAX;

  float speedCommand = mapJoystick(voltageA2, centerVoltage, deadband, effectiveRange, maxSpeed);
  float turnCommand  = mapJoystick(voltageA3, centerVoltage, deadband, effectiveRange, maxTurn);

  sendMovementCommand(speedCommand, turnCommand);

  // --- 2. Check Button State and Send LED Command on Press ---
  static int lastButtonState = HIGH;
  int currentButtonState = digitalRead(buttonPin);
  if (currentButtonState != lastButtonState) {
    if (currentButtonState == LOW) {
      // Button pressed: send LED command (Ring mode, 100 ms, red)
      sendLEDCommand();
    }
    lastButtonState = currentButtonState;
  }

  // --- 3. Debug Output ---
  Serial.print("A2 Voltage: ");
  Serial.print(voltageA2, 3);
  Serial.print(" V => Speed: ");
  Serial.print(speedCommand, 3);
  Serial.print("\tA3 Voltage: ");
  Serial.print(voltageA3, 3);
  Serial.print(" V => Turn: ");
  Serial.println(turnCommand, 3);

  delay(5);  // Loop every 500 ms
}

/*
 * mapJoystick()
 *
 * Converts a joystick voltage reading to a command value.
 * If within the deadband (±deadband around center), returns 0.
 * Otherwise, scales the deviation (beyond deadband) linearly to ±maxCommand.
 */
float mapJoystick(float voltage, float center, float deadband, float effective, float maxCommand) {
  float delta = voltage - center;
  if (abs(delta) < deadband) {
    return 0.0;
  }
  if (delta > deadband) {
    return ((delta - deadband) / effective) * maxCommand;
  } else {  // delta < -deadband
    return ((delta + deadband) / effective) * maxCommand;
  }
}

/*
 * calculateCRC8Maxim()
 *
 * Computes the CRC‑8/MAXIM over a block of data.
 * Uses a reflected polynomial (0x8C) with an initial value of 0x00.
 * The CRC is computed over the provided data array (of length 'len').
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
 * Constructs and sends a 10-byte movement command message over Serial1.
 * Message structure:
 *   Bytes 0-1: SOM marker (0x69, 0x69)
 *   Byte 2:    Command ID (0x01)
 *   Bytes 3-4: Forward Speed (16-bit signed fixed-point scaled by 100; big-endian)
 *   Bytes 5-6: Turn Rate (16-bit signed fixed-point scaled by 100; big-endian)
 *   Bytes 7-8: Reserved (0x00)
 *   Byte 9:    CRC‑8/MAXIM computed over bytes 2–8.
 */
void sendMovementCommand(float speed, float turn) {
  uint8_t message[10];

  // Set SOM marker
  message[0] = 0x69;
  message[1] = 0x69;

  // Protocol payload
  message[2] = 0x01;  // Movement command ID

  // Forward speed conversion (16-bit signed, scale by 100)
  int16_t speedVal = (int16_t)(speed * 100);
  message[3] = (uint8_t)((speedVal >> 8) & 0xFF);
  message[4] = (uint8_t)(speedVal & 0xFF);

  // Turn rate conversion (16-bit signed, scale by 100)
  int16_t turnVal = (int16_t)(turn * 100);
  message[5] = (uint8_t)((turnVal >> 8) & 0xFF);
  message[6] = (uint8_t)(turnVal & 0xFF);

  // Reserved bytes
  message[7] = 0x00;
  message[8] = 0x00;

  // CRC computed over payload bytes (bytes 2-8)
  message[9] = calculateCRC8Maxim(&message[2], 7);

  // Send the message over Serial1 (UART)
  Serial1.write(message, 10);

  // Debug: Print the message in hexadecimal format
  Serial.print("Sent Movement Message: ");
  for (int i = 0; i < 10; i++) {
    if (message[i] < 0x10) Serial.print("0");
    Serial.print(message[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

/*
 * sendLEDCommand()
 *
 * Constructs and sends a 10-byte LED command message over Serial1.
 * Message structure:
 *   Bytes 0-1: SOM marker (0x69, 0x69)
 *   Byte 2:    Command ID (0x02)
 *   Byte 3:    LED Mode (3 for Ring mode)
 *   Bytes 4-5: Time Parameter (16-bit unsigned; 100 ms → high byte then low byte)
 *   Byte 6:    Red intensity (0xFF)
 *   Byte 7:    Green intensity (0x00)
 *   Byte 8:    Blue intensity (0x00)
 *   Byte 9:    CRC‑8/MAXIM computed over bytes 2–8.
 */
void sendLEDCommand() {
  uint8_t message[10];

  // Set SOM marker
  message[0] = 0x69;
  message[1] = 0x69;

  // Protocol payload
  message[2] = 0x02;  // LED command ID
  message[3] = 0x03;  // LED Mode: 3 (Ring mode)

  // Time parameter: 100 ms (16-bit unsigned, high byte then low)
  uint16_t timeParam = 100;
  message[4] = (uint8_t)((timeParam >> 8) & 0xFF);
  message[5] = (uint8_t)(timeParam & 0xFF);

  // LED color: Red = 0xFF, Green = 0x00, Blue = 0x00
  message[6] = 0xFF;
  message[7] = 0x00;
  message[8] = 0x00;

  // CRC computed over payload bytes (bytes 2-8)
  message[9] = calculateCRC8Maxim(&message[2], 7);

  // Send the message over Serial1 (UART)
  Serial1.write(message, 10);

  // Debug: Print the LED command message in hexadecimal format
  Serial.print("Sent LED Command: ");
  for (int i = 0; i < 10; i++) {
    if (message[i] < 0x10) Serial.print("0");
    Serial.print(message[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}
