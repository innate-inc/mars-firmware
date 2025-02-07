#include <Arduino.h>

// ADC configuration (assuming a 10-bit ADC: 0–1023)
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

// -----------------------------------------------------------------------------
// Forward Declarations
float mapJoystick(float voltage, float center, float deadband, float effective, float maxCommand);
uint8_t calculateCRC8Maxim(uint8_t *data, int len);
void sendMovementCommand(float speed, float turn);
void sendLEDCommand();

// --- Functions for Receiving and Processing Incoming Messages ---

// Define a simple state machine to parse incoming messages
enum ParseState { WAIT_FOR_SOM1, WAIT_FOR_SOM2, READ_PAYLOAD };
ParseState parseState = WAIT_FOR_SOM1;
uint8_t rxBuffer[10];
uint8_t rxIndex = 0;

/*
 * processSerialMessages()
 *
 * This function reads incoming bytes from Serial1, looks for the two-byte SOM
 * marker (0x69, 0x69), collects the subsequent 8 bytes (payload and CRC), then
 * validates the message by computing the CRC‑8/MAXIM on bytes 2–8.
 *
 * If the message is valid, print its contents according to the response type.
 */
void processSerialMessages() {
  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();
    switch (parseState) {
      case WAIT_FOR_SOM1:
        if (b == 0x69) {
          parseState = WAIT_FOR_SOM2;
        }
        break;

      case WAIT_FOR_SOM2:
        if (b == 0x69) {
          // Found complete SOM marker. Save it and start reading the payload.
          rxBuffer[0] = 0x69;
          rxBuffer[1] = 0x69;
          rxIndex = 2; // Next byte will be payload byte 0 (command/response ID)
          parseState = READ_PAYLOAD;
        } else {
          // Incorrect sequence: start over.
          parseState = WAIT_FOR_SOM1;
        }
        break;

      case READ_PAYLOAD:
        rxBuffer[rxIndex++] = b;
        if (rxIndex >= 10) { // We have all 10 bytes
          // Validate the message (compute CRC over bytes 2–8 and compare with byte 9)
          if (validateMessage(rxBuffer)) {
            printMessage(rxBuffer);
          } else {
            Serial.println("Invalid message received (CRC mismatch).");
          }
          // Reset the state machine to search for the next message.
          parseState = WAIT_FOR_SOM1;
          rxIndex = 0;
        }
        break;
    }
  }
}

/*
 * validateMessage()
 *
 * Computes the CRC‑8/MAXIM over bytes 2–8 of the 10-byte message and compares
 * it to the received CRC (byte 9). Returns true if they match.
 */
bool validateMessage(uint8_t *msg) {
  uint8_t computedCRC = calculateCRC8Maxim(&msg[2], 7);
  return (computedCRC == msg[9]);
}

/*
 * printMessage()
 *
 * Decodes and prints the contents of a valid response message.
 * Supported response IDs are:
 *   • 0x81: Movement (Position Update) Response
 *   • 0x82: LED Response
 *   • 0x83: Status Response
 */
void printMessage(uint8_t *msg) {
  uint8_t responseID = msg[2];
  switch (responseID) {
    case 0x81: {  // Movement (Position Update) Response
      int16_t x = (msg[3] << 8) | msg[4];
      int16_t y = (msg[5] << 8) | msg[6];
      int16_t theta = (msg[7] << 8) | msg[8];
      Serial.print("Movement Response Received: X=");
      Serial.print(x);
      Serial.print(" Y=");
      Serial.print(y);
      Serial.print(" Theta=");
      Serial.println(theta);
      break;
    }
    case 0x82: {  // LED Response
      uint8_t ledMode = msg[3];
      uint8_t red = msg[4];
      uint8_t green = msg[5];
      uint8_t blue = msg[6];
      uint16_t timeParam = (msg[7] << 8) | msg[8];
      Serial.print("LED Response Received: Mode=");
      Serial.print(ledMode);
      Serial.print(" Red=");
      Serial.print(red);
      Serial.print(" Green=");
      Serial.print(green);
      Serial.print(" Blue=");
      Serial.print(blue);
      Serial.print(" Time Parameter=");
      Serial.println(timeParam);
      break;
    }
    case 0x83: {  // Status Response
      uint16_t batteryVoltage = (msg[3] << 8) | msg[4];
      uint16_t motorTemp = (msg[5] << 8) | msg[6];
      uint8_t faultCode = msg[7];
      Serial.print("Status Response Received: Battery Voltage=");
      Serial.print(batteryVoltage);
      Serial.print(" Motor Temp=");
      Serial.print(motorTemp);
      Serial.print(" Fault Code=");
      Serial.println(faultCode);
      break;
    }
    default:
      Serial.print("Unknown Response ID: ");
      Serial.println(responseID, HEX);
      break;
  }
}

// -----------------------------------------------------------------------------
// Setup & Loop
// -----------------------------------------------------------------------------

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
  // --- 1. Process any incoming messages ---
  processSerialMessages();

  // --- 2. Read Joystick Values and Send Movement Command ---
  int rawA2 = analogRead(A2);  // Speed axis
  int rawA3 = analogRead(A3);  // Turn axis

  float voltageA2 = (rawA2 * VREF) / ADC_MAX;
  float voltageA3 = (rawA3 * VREF) / ADC_MAX;

  float speedCommand = mapJoystick(voltageA2, centerVoltage, deadband, effectiveRange, maxSpeed);
  float turnCommand  = mapJoystick(voltageA3, centerVoltage, deadband, effectiveRange, maxTurn);

  sendMovementCommand(speedCommand, turnCommand);

  // --- 3. Check Button State and Send LED Command on Press ---
  static int lastButtonState = HIGH;
  int currentButtonState = digitalRead(buttonPin);
  if (currentButtonState != lastButtonState) {
    if (currentButtonState == LOW) {
      // Button pressed: send LED command (Ring mode, 100 ms, red)
      sendLEDCommand();
    }
    lastButtonState = currentButtonState;
  }

  // --- 4. Debug Output ---
  /*Serial.print("A2 Voltage: ");
  Serial.print(voltageA2, 3);
  Serial.print(" V => Speed: ");
  Serial.print(speedCommand, 3);
  Serial.print("\tA3 Voltage: ");
  Serial.print(voltageA3, 3);
  Serial.print(" V => Turn: ");
  Serial.println(turnCommand, 3);*/

  delay(5);  // Short delay; adjust as needed
}

// -----------------------------------------------------------------------------
// Utility Functions (Sending & CRC Calculation)
// -----------------------------------------------------------------------------

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
      if (crc & 0x01)
        crc = (crc >> 1) ^ 0x8C;
      else
        crc >>= 1;
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
  /*Serial.print("Sent Movement Message: ");
  for (int i = 0; i < 10; i++) {
    if (message[i] < 0x10) Serial.print("0");
    Serial.print(message[i], HEX);
    Serial.print(" ");
  }
  Serial.println();*/
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

