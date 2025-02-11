#include <Arduino.h>
#include <math.h>  // For PI and math operations

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

// Global state for current position feedback.
// X and Y are in millimeters.
// Theta is stored in degrees (normalized between -180 and 180).
struct Position {
  int16_t x;
  int16_t y;
  float theta;
};

Position currentPosition = {0, 0, 0.0};
unsigned long lastPositionPrintTime = 0; // For printing every 1 second

// --- Function Declarations ---
float mapJoystick(float voltage, float center, float deadband, float effective, float maxCommand);
uint8_t calculateCRC8Maxim(uint8_t *data, int len);
void sendMovementCommand(float speed, float turn);
void sendLEDCommand();
bool validateMessage(uint8_t *msg);
void printMessage(uint8_t *msg);
void processSerialMessages();

// --- Functions for Receiving and Processing Incoming Messages ---

enum ParseState { WAIT_FOR_SOM1, WAIT_FOR_SOM2, READ_PAYLOAD };
ParseState parseState = WAIT_FOR_SOM1;
uint8_t rxBuffer[10];
uint8_t rxIndex = 0;

void processSerialMessages() {
  while (Serial1.available() > 0) {
    uint8_t b = Serial1.read();
    // (No per-byte printing here.)

    // State machine for packet assembly.
    switch (parseState) {
      case WAIT_FOR_SOM1:
        if (b == 0x69) {
          parseState = WAIT_FOR_SOM2;
        }
        break;
      case WAIT_FOR_SOM2:
        if (b == 0x69) {
          // Found complete SOM marker.
          rxBuffer[0] = 0x69;
          rxBuffer[1] = 0x69;
          rxIndex = 2;  // Next byte will be payload (command/response ID)
          parseState = READ_PAYLOAD;
        } else {
          parseState = WAIT_FOR_SOM1;
        }
        break;
      case READ_PAYLOAD:
        rxBuffer[rxIndex++] = b;
        if (rxIndex >= 10) { // We have all 10 bytes.
          // Validate the message (CRC computed over bytes 2–8).
          if (validateMessage(rxBuffer)) {
            printMessage(rxBuffer);
          } else {
            Serial.println("Invalid message received (CRC mismatch).");
          }
          // Reset state machine for next message.
          parseState = WAIT_FOR_SOM1;
          rxIndex = 0;
        }
        break;
    }
  }
}

bool validateMessage(uint8_t *msg) {
  uint8_t computedCRC = calculateCRC8Maxim(&msg[2], 7);
  return (computedCRC == msg[9]);
}

void printMessage(uint8_t *msg) {
  // First, print the entire 10-byte message as a hex dump.
  Serial.print("Valid message received: ");
  for (int i = 0; i < 10; i++) {
    if (msg[i] < 0x10) Serial.print("0");
    Serial.print(msg[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  // Then interpret the message based on the response ID.
  uint8_t responseID = msg[2];
  switch (responseID) {
    case 0x81: {  // Movement (Position Update) Response
      // Extract raw data.
      int16_t x = (msg[3] << 8) | msg[4];
      int16_t y = (msg[5] << 8) | msg[6];
      int16_t theta_raw = (msg[7] << 8) | msg[8];
      
      // Convert theta:
      // - The raw theta is in hundredths of a radian.
      // - Divide by 100 to get radians.
      // - Multiply by (180/PI) to convert to degrees.
      float theta_rad = theta_raw / 100.0;
      float theta_deg = theta_rad * (180.0 / PI);
      
      // Normalize theta to be between -180 and 180 degrees.
      while (theta_deg > 180) theta_deg -= 360;
      while (theta_deg < -180) theta_deg += 360;
      
      // Update global state.
      currentPosition.x = x;
      currentPosition.y = y;
      currentPosition.theta = theta_deg;
      
      // Print the interpreted values.
      Serial.print("Movement Response Received: X=");
      Serial.print(x);
      Serial.print(" mm, Y=");
      Serial.print(y);
      Serial.print(" mm, Theta=");
      Serial.print(theta_deg, 2);
      Serial.println(" deg");
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

// --- Setup & Loop ---
void setup() {
  // Initialize USB serial for debugging.
  Serial.begin(115200);
  while (!Serial) { ; }
  
  // Initialize hardware UART (Serial1).
  Serial1.begin(115200);

  // Configure the button pin with an internal pull-up resistor.
  pinMode(buttonPin, INPUT_PULLUP);

  // Seed the random number generator using an unconnected analog pin.
  randomSeed(analogRead(A0));

  Serial.println("Starting Joystick and LED Command Publisher...");
}

void loop() {
  unsigned long currentMillis = millis();

  // 1. Process any incoming messages from Serial1.
  processSerialMessages();

  // 2. Read Joystick Values and Send Movement Command.
  int rawA2 = analogRead(A2);  // Speed axis.
  int rawA3 = analogRead(A3);  // Turn axis.

  float voltageA2 = (rawA2 * VREF) / ADC_MAX;
  float voltageA3 = (rawA3 * VREF) / ADC_MAX;

  float speedCommand = mapJoystick(voltageA2, centerVoltage, deadband, effectiveRange, maxSpeed);
  float turnCommand  = mapJoystick(voltageA3, centerVoltage, deadband, effectiveRange, maxTurn);

  sendMovementCommand(speedCommand, turnCommand);

  // 3. Check Button State and Send LED Command on Press.
  static int lastButtonState = HIGH;
  int currentButtonState = digitalRead(buttonPin);
  if (currentButtonState != lastButtonState) {
    if (currentButtonState == LOW) {
      // Button pressed: send an LED command with random parameters.
      sendLEDCommand();
    }
    lastButtonState = currentButtonState;
  }

  // 4. Every 1 second, print the current position from state.
  if (currentMillis - lastPositionPrintTime >= 1000) {
    Serial.print("Current Position - X: ");
    Serial.print(currentPosition.x);
    Serial.print(" mm, Y: ");
    Serial.print(currentPosition.y);
    Serial.print(" mm, Theta: ");
    Serial.print(currentPosition.theta, 2);
    Serial.println(" deg");
    lastPositionPrintTime = currentMillis;
  }

  delay(20);  // Maintain a 50 Hz loop (20 ms per iteration).
}

// --- Utility Functions ---

/*
 * mapJoystick()
 *
 * Converts a joystick voltage reading to a command value.
 * Returns 0 if the reading is within the deadband.
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
 * Computes the CRC‑8/MAXIM checksum over the provided data.
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

  // Set SOM marker.
  message[0] = 0x69;
  message[1] = 0x69;
  
  // Command ID for movement.
  message[2] = 0x01;

  // Convert forward speed (scale by 100).
  int16_t speedVal = (int16_t)(speed * 100);
  message[3] = (uint8_t)((speedVal >> 8) & 0xFF);
  message[4] = (uint8_t)(speedVal & 0xFF);

  // Convert turn rate (scale by 100).
  int16_t turnVal = (int16_t)(turn * 100);
  message[5] = (uint8_t)((turnVal >> 8) & 0xFF);
  message[6] = (uint8_t)(turnVal & 0xFF);

  // Reserved bytes.
  message[7] = 0x00;
  message[8] = 0x00;

  // CRC calculated over payload bytes (2–8).
  message[9] = calculateCRC8Maxim(&message[2], 7);

  // Send the message.
  Serial1.write(message, 10);
}

/*
 * sendLEDCommand()
 *
 * Constructs and sends a 10-byte LED command message over Serial1.
 * The LED command now uses randomized parameters:
 *   - LED Mode: Random value (0–3)
 *   - Time Parameter: Random value between 100 and 600 ms.
 *   - LED Color: Each of R, G, B is random (0–255)
 */
void sendLEDCommand() {
  uint8_t message[10];

  // Set SOM marker.
  message[0] = 0x69;
  message[1] = 0x69;
  
  // Command ID for LED.
  message[2] = 0x02;

  // Randomize LED mode (0–3).
  uint8_t ledMode = random(0, 4);
  message[3] = ledMode;

  // Randomize time parameter between 100 and 600 ms.
  uint16_t timeParam = random(100, 600);
  message[4] = (uint8_t)((timeParam >> 8) & 0xFF);
  message[5] = (uint8_t)(timeParam & 0xFF);

  // Randomize LED color values (Red, Green, Blue; each 0–255).
  uint8_t red   = random(0, 256);
  uint8_t green = random(0, 256);
  uint8_t blue  = random(0, 256);
  message[6] = red;
  message[7] = green;
  message[8] = blue;

  // Compute CRC over payload bytes (2–8).
  message[9] = calculateCRC8Maxim(&message[2], 7);

  // Send the LED command.
  Serial1.write(message, 10);

  // Optionally, print the LED command that was sent.
  Serial.print("Sent LED Command: ");
  for (int i = 0; i < 10; i++) {
    if (message[i] < 0x10) Serial.print("0");
    Serial.print(message[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Mode: ");
  Serial.print(ledMode);
  Serial.print(" Time: ");
  Serial.print(timeParam);
  Serial.print(" ms  Color: (");
  Serial.print(red);
  Serial.print(", ");
  Serial.print(green);
  Serial.print(", ");
  Serial.print(blue);
  Serial.println(")");
}


