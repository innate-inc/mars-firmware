#include <Arduino.h>
#include "wiring_private.h"  // Needed for pinPeripheral()

// ---------------------------------------------------------------------
// Motor and drive parameters
#define MOTOR_ID        0x01           // Motor ID to set on both motors
#define WHEEL_DIAMETER  73.0           // Wheel diameter in mm
#define MAX_SPEED       2100           // Motor command range: -2100 to +2100
#define TRACK_WIDTH     140.0          // Distance between the two motors in mm

// ---------------------------------------------------------------------
// For main robot communications on Serial1:
// Movement command protocol: 10 bytes total = 2-byte SOM + 8-byte payload.
// The payload for a movement command (command ID 0x01) is:
//   Byte0: Command ID (0x01)
//   Byte1-2: Forward speed (16-bit signed, scaled by 100; e.g., 64 means 0.64 m/s)
//   Byte3-4: Turn rate (16-bit signed, scaled by 100; e.g., -128 means -1.28 rad/s)
//   Bytes5-6: Reserved (0x00)
//   Byte7: CRC (CRC-8/MAXIM computed over bytes 0–6)
const uint8_t SOM[2] = {0x69, 0x69};

// ---------------------------------------------------------------------
// Create Uart objects for each motor controller

// Left Motor: Using SERCOM4 with TX on PIN_A2 (Pad 0) and RX on PIN_A3 (Pad 1)
Uart SerialLeft(&sercom4, PIN_A3, PIN_A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Right Motor: Using SERCOM0 with TX on PIN_A4 (Pad 0) and RX on PIN_A1 (Pad 1)
Uart SerialRight(&sercom0, PIN_A1, PIN_A4, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// ---------------------------------------------------------------------
// Required SERCOM interrupt handlers for SERCOM4 (Left Motor)
void SERCOM4_0_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_1_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_2_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_3_Handler() { SerialLeft.IrqHandler(); }

// Required SERCOM interrupt handlers for SERCOM0 (Right Motor)
void SERCOM0_0_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_1_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_2_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_3_Handler() { SerialRight.IrqHandler(); }

// ---------------------------------------------------------------------
// CRC-8/MAXIM function (Polynomial 0x8C, initial value 0x00)
uint8_t calculate_crc8_maxim(uint8_t *data, int len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0x8C;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// ---------------------------------------------------------------------
// Global feedback buffers and variables
#define MOTOR_BUFFER_SIZE 64
uint8_t motorFeedbackBufferLeft[MOTOR_BUFFER_SIZE];
uint8_t motorFeedbackIndexLeft = 0;
uint8_t motorFeedbackBufferRight[MOTOR_BUFFER_SIZE];
uint8_t motorFeedbackIndexRight = 0;

String leftFeedbackHex = "";
float leftFeedbackRpm = 0.0;
bool leftFeedbackReady = false;

String rightFeedbackHex = "";
float rightFeedbackRpm = 0.0;
bool rightFeedbackReady = false;

// ---------------------------------------------------------------------
// Motor initialization functions
// Set motor ID command (send 5 times)
void setMotorID(Uart &motorSerial) {
  uint8_t cmd[10] = {0};
  cmd[0] = 0xAA;
  cmd[1] = 0x55;
  cmd[2] = 0x53;
  cmd[3] = MOTOR_ID;  // Set ID to 1
  cmd[4] = 0x00;
  cmd[5] = 0x00;
  cmd[6] = 0x00;
  cmd[7] = 0x00;
  cmd[8] = 0x00;
  cmd[9] = calculate_crc8_maxim(cmd, 9);
  for (int i = 0; i < 5; i++) {
    motorSerial.write(cmd, 10);
    delay(20);
  }
}

// Set motor mode to speed loop (mode value 0x02)
void setMotorMode(Uart &motorSerial) {
  uint8_t cmd[10] = {0};
  cmd[0] = MOTOR_ID;
  cmd[1] = 0xA0;
  cmd[2] = 0x02;  // 0x02: speed loop mode
  cmd[3] = 0x00;
  cmd[4] = 0x00;
  cmd[5] = 0x00;
  cmd[6] = 0x00;
  cmd[7] = 0x00;
  cmd[8] = 0x00;
  cmd[9] = calculate_crc8_maxim(cmd, 9);
  motorSerial.write(cmd, 10);
  delay(20);
}

// ---------------------------------------------------------------------
// sendMotorCommand()
// Constructs and sends a 10-byte drive command via the specified UART.
// Command format:
//   Byte0: Motor ID (0x01)
//   Byte1: Command (0x64 for drive)
//   Bytes 2-3: Speed (16-bit signed; high byte then low byte)
//   Bytes 4-5: Reserved (0x00)
//   Byte6: Acceleration time (e.g., 0x01 means 1ms per 1rpm)
//   Byte7: Brake (0x00 for no braking; 0xFF for brake)
//   Byte8: Reserved (0x00)
//   Byte9: CRC-8/MAXIM over bytes 0-8
void sendMotorCommand(Uart &motorSerial, int16_t speed) {
  uint8_t cmd[10] = {0};
  cmd[0] = MOTOR_ID;
  cmd[1] = 0x64;  // Drive command
  cmd[2] = (speed >> 8) & 0xFF;
  cmd[3] = speed & 0xFF;
  cmd[4] = 0x00;
  cmd[5] = 0x00;
  cmd[6] = 0x01;
  cmd[7] = 0x00;
  cmd[8] = 0x00;
  cmd[9] = calculate_crc8_maxim(cmd, 9);
  motorSerial.write(cmd, 10);
}

// ---------------------------------------------------------------------
// processMovementCommand()
// Processes an 8-byte payload (received from Serial1 after the SOM) to compute
// the left and right motor drive commands using a differential-drive formula.
void processMovementCommand(uint8_t *msg) {
  // Expected payload:
  // Byte0: Command ID (0x01)
  // Byte1-2: Forward speed (16-bit signed, scaled by 100)
  // Byte3-4: Turn rate (16-bit signed, scaled by 100)
  // Bytes5-6: Reserved
  // Byte7: CRC
  if (msg[0] != 0x01) return;
  int16_t forwardSpeed = ((int16_t)msg[1] << 8) | msg[2];
  int16_t turnRate = ((int16_t)msg[3] << 8) | msg[4];

  // Convert forwardSpeed (scale=100) to mm/s:
  float v_mmps = (forwardSpeed / 100.0) * 1000.0;
  float wheelCirc = WHEEL_DIAMETER * 3.14159;
  float target_rpm = (v_mmps / wheelCirc) * 60.0;

  // turnRate (scale=100) is assumed to be angular velocity in rad/s
  float omega = turnRate / 100.0;
  float delta = (omega * TRACK_WIDTH * 60.0) / (2.0 * wheelCirc);
  float left_rpm = target_rpm - delta;
  float right_rpm = target_rpm + delta;

  // Convert rpm to command value (multiply by 10)
  int16_t left_cmd = (int16_t)(left_rpm * 10);
  int16_t right_cmd = (int16_t)(right_rpm * 10);

  // Clamp to allowed range
  if (left_cmd > MAX_SPEED) left_cmd = MAX_SPEED;
  if (left_cmd < -MAX_SPEED) left_cmd = -MAX_SPEED;
  if (right_cmd > MAX_SPEED) right_cmd = MAX_SPEED;
  if (right_cmd < -MAX_SPEED) right_cmd = -MAX_SPEED;

  sendMotorCommand(SerialLeft, left_cmd);
  sendMotorCommand(SerialRight, right_cmd);
}

// ---------------------------------------------------------------------
// Feedback processing functions for each motor

// Process feedback from left motor (SerialLeft) using a sliding-window parser.
void processLeftFeedback() {
  while (SerialLeft.available() > 0) {
    if (motorFeedbackIndexLeft < MOTOR_BUFFER_SIZE)
      motorFeedbackBufferLeft[motorFeedbackIndexLeft++] = SerialLeft.read();
    else {
      for (uint8_t i = 1; i < MOTOR_BUFFER_SIZE; i++)
        motorFeedbackBufferLeft[i - 1] = motorFeedbackBufferLeft[i];
      motorFeedbackBufferLeft[MOTOR_BUFFER_SIZE - 1] = SerialLeft.read();
    }
    while (motorFeedbackIndexLeft >= 10) {
      if (calculate_crc8_maxim(motorFeedbackBufferLeft, 9) == motorFeedbackBufferLeft[9]) {
        String hexStr = "";
        for (int i = 0; i < 10; i++)
          hexStr += String(motorFeedbackBufferLeft[i], HEX) + " ";
        int16_t speedFeedback = ((int16_t)motorFeedbackBufferLeft[2] << 8) | motorFeedbackBufferLeft[3];
        leftFeedbackRpm = speedFeedback / 10.0;
        leftFeedbackHex = hexStr;
        leftFeedbackReady = true;
        for (uint8_t i = 10; i < motorFeedbackIndexLeft; i++)
          motorFeedbackBufferLeft[i - 10] = motorFeedbackBufferLeft[i];
        motorFeedbackIndexLeft -= 10;
      } else {
        for (uint8_t i = 1; i < motorFeedbackIndexLeft; i++)
          motorFeedbackBufferLeft[i - 1] = motorFeedbackBufferLeft[i];
        motorFeedbackIndexLeft--;
      }
    }
  }
}

// Process feedback from right motor (SerialRight) using a sliding-window parser.
void processRightFeedback() {
  while (SerialRight.available() > 0) {
    if (motorFeedbackIndexRight < MOTOR_BUFFER_SIZE)
      motorFeedbackBufferRight[motorFeedbackIndexRight++] = SerialRight.read();
    else {
      for (uint8_t i = 1; i < MOTOR_BUFFER_SIZE; i++)
        motorFeedbackBufferRight[i - 1] = motorFeedbackBufferRight[i];
      motorFeedbackBufferRight[MOTOR_BUFFER_SIZE - 1] = SerialRight.read();
    }
    while (motorFeedbackIndexRight >= 10) {
      if (calculate_crc8_maxim(motorFeedbackBufferRight, 9) == motorFeedbackBufferRight[9]) {
        String hexStr = "";
        for (int i = 0; i < 10; i++)
          hexStr += String(motorFeedbackBufferRight[i], HEX) + " ";
        int16_t speedFeedback = ((int16_t)motorFeedbackBufferRight[2] << 8) | motorFeedbackBufferRight[3];
        rightFeedbackRpm = speedFeedback / 10.0;
        rightFeedbackHex = hexStr;
        rightFeedbackReady = true;
        for (uint8_t i = 10; i < motorFeedbackIndexRight; i++)
          motorFeedbackBufferRight[i - 10] = motorFeedbackBufferRight[i];
        motorFeedbackIndexRight -= 10;
      } else {
        for (uint8_t i = 1; i < motorFeedbackIndexRight; i++)
          motorFeedbackBufferRight[i - 1] = motorFeedbackBufferRight[i];
        motorFeedbackIndexRight--;
      }
    }
  }
}

// Process feedback from both motors and print on the same line when both are available.
void processFeedback() {
  processLeftFeedback();
  processRightFeedback();
  if (leftFeedbackReady && rightFeedbackReady) {
    Serial.print("Left Feedback: ");
    Serial.print(leftFeedbackHex);
    Serial.print(" | RPM: ");
    Serial.print(leftFeedbackRpm, 1);
    Serial.print("    Right Feedback: ");
    Serial.print(rightFeedbackHex);
    Serial.print(" | RPM: ");
    Serial.println(rightFeedbackRpm, 1);
    leftFeedbackReady = false;
    rightFeedbackReady = false;
  }
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Initializing Dual Motor Controller...");

  // Initialize Serial1 (movement commands)
  Serial1.begin(115200);

  // Initialize left motor UART on SERCOM4
  SerialLeft.begin(115200);
  pinPeripheral(PIN_A2, PIO_SERCOM_ALT);  // A2 as TX (SERCOM4 Pad 0)
  pinPeripheral(PIN_A3, PIO_SERCOM_ALT);  // A3 as RX (SERCOM4 Pad 1)

  // Initialize right motor UART on SERCOM0
  SerialRight.begin(115200);
  pinPeripheral(PIN_A4, PIO_SERCOM_ALT);  // A4 as TX (SERCOM0 Pad 0)
  pinPeripheral(PIN_A1, PIO_SERCOM_ALT);  // A1 as RX (SERCOM0 Pad 1)

  Serial.println("Serial1, SerialLeft (SERCOM4) & SerialRight (SERCOM0) Ready.");

  // Motor Initialization Phase:
  // 1. Set motor IDs to 1 (send ID command 5 times)
  setMotorID(SerialLeft);
  setMotorID(SerialRight);
  delay(100);
  // 2. Set motor mode to speed loop (mode value 0x02)
  setMotorMode(SerialLeft);
  setMotorMode(SerialRight);
  Serial.println("Motor initialization complete. Now waiting for movement commands on Serial1.");
}

// ---------------------------------------------------------------------
void loop() {
  // Process movement commands from Serial1.
  if (Serial1.available() >= 10) {
    uint8_t firstByte = Serial1.read();
    if (firstByte == SOM[0] && Serial1.peek() == SOM[1]) {
      Serial1.read();  // Consume second SOM byte
      uint8_t payload[8];
      Serial1.readBytes(payload, 8);
      uint8_t crc_recv = payload[7];
      uint8_t crc_calc = calculate_crc8_maxim(payload, 7);
      if (crc_recv == crc_calc) {
        processMovementCommand(payload);
      } else {
        Serial.println("Movement command CRC error.");
      }
    }
  }
  
  // Process feedback from both motors.
  processFeedback();
}

