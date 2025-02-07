#include <Arduino.h>
#include "wiring_private.h"  // Needed for pinPeripheral()

// -----------------------
// Motor and wheel parameters
#define MOTOR_ID 0x01           // Motor ID (adjust if needed)
#define WHEEL_DIAMETER_MM 73.0    // Wheel diameter in mm (not used for feedback decoding)
#define MAX_RPM 210             // Motor's RPM limit (for command conversion)
#define MAX_SPEED 2100          // Command range corresponds to -2100 ~ 2100

// -----------------------
// For main robot communications on Serial1:
// Message protocol: 10 bytes total = 2 bytes SOM + 8 bytes protocol message
const uint8_t SOM[2] = {0x69, 0x69};

// -----------------------
// Setup Serial2 on SERCOM4 (motor control and feedback)
// Using PIN_A2 (TX) and PIN_A3 (RX)
// IMPORTANT: For SAMD51, TX must be on Pad 0.
Uart Serial2(&sercom4, PIN_A3, PIN_A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Required Interrupt Handlers for SERCOM4 (SAMD51 requires four)
void SERCOM4_0_Handler() { Serial2.IrqHandler(); }
void SERCOM4_1_Handler() { Serial2.IrqHandler(); }
void SERCOM4_2_Handler() { Serial2.IrqHandler(); }
void SERCOM4_3_Handler() { Serial2.IrqHandler(); }

// -----------------------
// CRC-8/MAXIM function (Polynomial 0x8C, Initial Value 0x00)
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

// -----------------------
// Buffer for motor feedback from Serial2 (motor responses)
// We'll accumulate bytes and use a sliding window to find valid 10-byte messages.
#define MOTOR_BUFFER_SIZE 64
uint8_t motorFeedbackBuffer[MOTOR_BUFFER_SIZE];
uint8_t motorFeedbackIndex = 0;

// -----------------------
void setup() {
  Serial.begin(115200);   // USB Serial for debugging
  while (!Serial);        // Wait for the Serial Monitor to open
  Serial.println("Initializing...");

  // Setup Serial1 (main robot communication)
  Serial1.begin(115200);

  // Setup Serial2 (SERCOM4) for motor control and feedback
  Serial2.begin(115200);  // Start SERCOM UART first
  pinPeripheral(PIN_A3, PIO_SERCOM_ALT);  // Configure PIN_A3 for SERCOM4 RX (Pad 1)
  pinPeripheral(PIN_A2, PIO_SERCOM_ALT);  // Configure PIN_A2 for SERCOM4 TX (Pad 0)

  Serial.println("UART1 (Serial1) & SERCOM4 (Serial2) Ready.");
}

// -----------------------
void loop() {
  // === Process incoming commands from Serial1 (main robot communication) ===
  while (Serial1.available() >= 10) {
    uint8_t firstByte = Serial1.read();
    if (firstByte == SOM[0] && Serial1.peek() == SOM[1]) {  // Check for SOM marker
      Serial1.read(); // Consume the second SOM byte
      uint8_t message[8];
      Serial1.readBytes(message, 8);

      uint8_t received_crc = message[7];  // Last byte is the CRC of the protocol message
      uint8_t calculated_crc = calculate_crc8_maxim(message, 7);

      if (received_crc == calculated_crc) {
        processCommand(message);
      }
    }
  }

  // === Process motor feedback from Serial2 using a sliding window parser ===
  while (Serial2.available() > 0) {
    // Append one byte from Serial2 to the feedback buffer
    if (motorFeedbackIndex < MOTOR_BUFFER_SIZE) {
      motorFeedbackBuffer[motorFeedbackIndex++] = Serial2.read();
    } else {
      // If the buffer is full, shift everything left by one and add the new byte
      for (uint8_t i = 1; i < MOTOR_BUFFER_SIZE; i++) {
        motorFeedbackBuffer[i - 1] = motorFeedbackBuffer[i];
      }
      motorFeedbackBuffer[MOTOR_BUFFER_SIZE - 1] = Serial2.read();
    }
    
    // Check if the buffer has at least 10 bytes (a full message)
    while (motorFeedbackIndex >= 10) {
      // Compute CRC over the first 9 bytes; compare with the 10th byte
      if (calculate_crc8_maxim(motorFeedbackBuffer, 9) == motorFeedbackBuffer[9]) {
        // Valid message found: decode and print it
        Serial.print("[Motor Feedback] ");
        for (int i = 0; i < 10; i++) {
          Serial.print(motorFeedbackBuffer[i], HEX);
          Serial.print(" ");
        }
        
        // Decode motor feedback if it is a drive feedback message (ID: any, then 0x64)
        if (motorFeedbackBuffer[1] == 0x64) {
          // Bytes 2 and 3 represent the speed feedback (signed 16-bit)
          int16_t speedFeedback = ((int16_t)motorFeedbackBuffer[2] << 8) | motorFeedbackBuffer[3];
          // According to the protocol, -2100 ~ 2100 corresponds to -210 ~ 210 rpm.
          // Thus, convert the 16-bit signed value to RPM by dividing by 10.
          float motorRpm = speedFeedback / 10.0;
          Serial.print(" | RPM: ");
          Serial.print(motorRpm, 1);
        }
        Serial.println();

        // Remove the 10-byte message from the buffer by shifting remaining bytes
        for (uint8_t i = 10; i < motorFeedbackIndex; i++) {
          motorFeedbackBuffer[i - 10] = motorFeedbackBuffer[i];
        }
        motorFeedbackIndex -= 10;
      } else {
        // The message at the beginning is not valid; shift the buffer by one byte and try again.
        for (uint8_t i = 1; i < motorFeedbackIndex; i++) {
          motorFeedbackBuffer[i - 1] = motorFeedbackBuffer[i];
        }
        motorFeedbackIndex--;
      }
    }
  }
}

// -----------------------
// Process commands coming in on Serial1 (main robot commands)
// For this example, we only handle movement commands (commandID == 0x01)
void processCommand(uint8_t *msg) {
  uint8_t commandID = msg[0];
  if (commandID == 0x01) {  // Movement Command
    int16_t forwardSpeed = (msg[1] << 8) | msg[2];
    
    // Convert fixed-point forwardSpeed (scaled by 100, representing m/s) to mm/s:
    float speed_mm_per_sec = (forwardSpeed / 100.0) * 1000.0;
    // Calculate the wheel circumference in mm (not used in feedback decoding):
    float wheel_circumference = WHEEL_DIAMETER_MM * 3.14159;
    // Compute target RPM from the desired speed (for command conversion):
    int16_t target_rpm = (speed_mm_per_sec / wheel_circumference) * 60.0;
    
    // Clamp to motor's RPM limits
    if (target_rpm > MAX_RPM) target_rpm = MAX_RPM;
    if (target_rpm < -MAX_RPM) target_rpm = -MAX_RPM;
    
    // Convert target RPM to the motor command range (-2100 ~ 2100)
    int16_t motor_command = (target_rpm * MAX_SPEED) / MAX_RPM;
    
    sendMotorCommand(motor_command);
  }
}

// -----------------------
// Send a motor command via Serial2 (SERCOM4)
// This function sends a 10-byte command; we do not print the command.
void sendMotorCommand(int16_t speed) {
  uint8_t command[10] = {0};
  command[0] = MOTOR_ID;
  command[1] = 0x64;  // Command for "Drive the Motor"
  command[2] = (speed >> 8) & 0xFF;  // Speed high byte
  command[3] = speed & 0xFF;         // Speed low byte
  command[4] = 0x00;  // Reserved
  command[5] = 0x00;  // Reserved
  command[6] = 0x01;  // Acceleration time (1ms per 1RPM)
  command[7] = 0x00;  // Brake (0xFF would brake; 0 means no braking)
  command[8] = 0x00;  // Reserved
  command[9] = calculate_crc8_maxim(command, 9);  // Compute CRC8 over first 9 bytes

  // Send the command to the motor via Serial2 (do not print the command)
  Serial2.write(command, 10);
}

