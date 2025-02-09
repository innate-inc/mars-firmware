#include <Arduino.h>
#include "wiring_private.h"    // Needed for pinPeripheral()
#include <Adafruit_NeoPixel.h>

// ---------------------------------------------------------------------
// Constants and Definitions
// ---------------------------------------------------------------------
#define MOTOR_ID        0x01           // Both motors set to ID 1
#define WHEEL_DIAMETER  73.0           // Wheel diameter in mm
#define MAX_SPEED       2100           // Motor command range: -2100 to 2100
#define TRACK_WIDTH     140.0          // Distance between the two motors in mm

const uint8_t SOM[2] = {0x69, 0x69};   // Start-of-Message marker

// LED definitions
#define LED_PIN     2       // Pin for NeoPixel strip
#define NUM_LEDS    39       // Number of LEDs

// Motor feedback buffer size
#define MOTOR_BUFFER_SIZE 64

// ---------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------

// Robot state (position and heading)
float posX = 0.0, posY = 0.0, theta = 0.0; // Position (in mm) and heading (in radians)
int16_t latestForwardSpeed = 0;           // Latest forward speed command (scaled by 100)
int16_t latestTurnRate = 0;               // Latest turn rate command (scaled by 100)

// LED state variables
uint8_t ledMode = 0;      // 0 = Off, 1 = Solid, 2 = Blink, 3 = Ring
uint16_t ledTime = 300;   // Time parameter in ms
uint8_t ledRed = 0, ledGreen = 0, ledBlue = 0;

// LED animation timing
unsigned long ledPreviousMillis = 0;
bool blinkState = false;
int currentLEDIndex = 0;

// Integration timing
unsigned long lastIntegrationTime = 0;

// Motor feedback buffers and state
uint8_t motorFeedbackBufferLeft[MOTOR_BUFFER_SIZE];
uint8_t motorFeedbackIndexLeft = 0;
float leftFeedbackRpm = 0.0;
bool leftFeedbackReady = false;

uint8_t motorFeedbackBufferRight[MOTOR_BUFFER_SIZE];
uint8_t motorFeedbackIndexRight = 0;
float rightFeedbackRpm = 0.0;
bool rightFeedbackReady = false;

// ---------------------------------------------------------------------
// Serial1 (host) message parser variables
// ---------------------------------------------------------------------
enum Serial1ParseState { WAIT_FOR_SOM1, WAIT_FOR_SOM2, READ_PAYLOAD };
Serial1ParseState serial1State = WAIT_FOR_SOM1;
uint8_t serial1PayloadBuffer[8];  // Holds the 8-byte payload (7 data + 1 CRC)
int serial1PayloadIndex = 0;

// ---------------------------------------------------------------------
// Motor UART Setup (as per your sample code)
// ---------------------------------------------------------------------

// Left Motor: Using SERCOM4 with TX on PIN_A2 (Pad 0) and RX on PIN_A3 (Pad 1)
Uart SerialLeft(&sercom4, PIN_A3, PIN_A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);
// Right Motor: Using SERCOM0 with TX on PIN_A4 (Pad 0) and RX on PIN_A1 (Pad 1)
Uart SerialRight(&sercom0, PIN_A1, PIN_A4, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Required SERCOM interrupt handlers for SerialLeft (SERCOM4)
void SERCOM4_0_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_1_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_2_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_3_Handler() { SerialLeft.IrqHandler(); }

// Required SERCOM interrupt handlers for SerialRight (SERCOM0)
void SERCOM0_0_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_1_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_2_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_3_Handler() { SerialRight.IrqHandler(); }

// ---------------------------------------------------------------------
// LED Setup
// ---------------------------------------------------------------------
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------------------------------------------------------
// Function Prototypes
// ---------------------------------------------------------------------
uint8_t calculate_crc8_maxim(uint8_t *data, int len);
void setMotorID(Uart &motorSerial);
void setMotorMode(Uart &motorSerial);
void sendMotorCommand(Uart &motorSerial, int16_t speed);
void processLeftFeedback();
void processRightFeedback();
void processMotorFeedback();
void computeMotorCommands(int16_t forward, int16_t turn, int16_t *leftCmd, int16_t *rightCmd);
void integratePosition();
void updateLEDs();
void resetLEDAnimation();
void updateLEDStripImmediate();
void processSerial1Messages();
void processSerial1Payload(uint8_t *payload);
void sendMovementResponse();
void sendLEDResponse();
void sendStatusResponse();

// ---------------------------------------------------------------------
// CRC-8/MAXIM Calculation Function (Polynomial 0x8C, initial value 0x00)
// ---------------------------------------------------------------------
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
// Motor Initialization Functions
// ---------------------------------------------------------------------
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

void setMotorMode(Uart &motorSerial) {
  uint8_t cmd[10] = {0};
  cmd[0] = MOTOR_ID;
  cmd[1] = 0xA0;
  cmd[2] = 0x02;  // Speed loop mode
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
// Send Motor Command (Drive Command)
// ---------------------------------------------------------------------
void sendMotorCommand(Uart &motorSerial, int16_t speed) {
  uint8_t cmd[10] = {0};
  cmd[0] = MOTOR_ID;
  cmd[1] = 0x64;  // Drive command
  cmd[2] = (speed >> 8) & 0xFF;
  cmd[3] = speed & 0xFF;
  cmd[4] = 0x00;
  cmd[5] = 0x00;
  cmd[6] = 0x01;  // Acceleration time = 1ms per 1rpm
  cmd[7] = 0x00;  // No braking (0xFF for brake)
  cmd[8] = 0x00;
  cmd[9] = calculate_crc8_maxim(cmd, 9);
  motorSerial.write(cmd, 10);
}

// ---------------------------------------------------------------------
// Process Motor Feedback for Left Motor
// ---------------------------------------------------------------------
void processLeftFeedback() {
  while (SerialLeft.available() > 0) {
    uint8_t byteRead = SerialLeft.read();
    if (motorFeedbackIndexLeft < MOTOR_BUFFER_SIZE)
      motorFeedbackBufferLeft[motorFeedbackIndexLeft++] = byteRead;
    else {
      // If buffer full, shift left to make room.
      for (uint8_t i = 1; i < MOTOR_BUFFER_SIZE; i++)
        motorFeedbackBufferLeft[i - 1] = motorFeedbackBufferLeft[i];
      motorFeedbackBufferLeft[MOTOR_BUFFER_SIZE - 1] = byteRead;
    }
    
    // Process complete 10-byte messages
    while (motorFeedbackIndexLeft >= 10) {
      if (calculate_crc8_maxim(motorFeedbackBufferLeft, 9) == motorFeedbackBufferLeft[9]) {
        int16_t speedFeedback = ((int16_t)motorFeedbackBufferLeft[2] << 8) | motorFeedbackBufferLeft[3];
        leftFeedbackRpm = speedFeedback / 10.0;  // Scale: motor feedback is multiplied by 10
        leftFeedbackReady = true;
        // Remove processed 10 bytes from buffer
        for (uint8_t i = 10; i < motorFeedbackIndexLeft; i++)
          motorFeedbackBufferLeft[i - 10] = motorFeedbackBufferLeft[i];
        motorFeedbackIndexLeft -= 10;
      } else {
        // If CRC fails, discard one byte and try to re-sync.
        for (uint8_t i = 1; i < motorFeedbackIndexLeft; i++)
          motorFeedbackBufferLeft[i - 1] = motorFeedbackBufferLeft[i];
        motorFeedbackIndexLeft--;
      }
    }
  }
}

// ---------------------------------------------------------------------
// Process Motor Feedback for Right Motor
// ---------------------------------------------------------------------
void processRightFeedback() {
  while (SerialRight.available() > 0) {
    uint8_t byteRead = SerialRight.read();
    if (motorFeedbackIndexRight < MOTOR_BUFFER_SIZE)
      motorFeedbackBufferRight[motorFeedbackIndexRight++] = byteRead;
    else {
      for (uint8_t i = 1; i < MOTOR_BUFFER_SIZE; i++)
        motorFeedbackBufferRight[i - 1] = motorFeedbackBufferRight[i];
      motorFeedbackBufferRight[MOTOR_BUFFER_SIZE - 1] = byteRead;
    }
    
    while (motorFeedbackIndexRight >= 10) {
      if (calculate_crc8_maxim(motorFeedbackBufferRight, 9) == motorFeedbackBufferRight[9]) {
        int16_t speedFeedback = ((int16_t)motorFeedbackBufferRight[2] << 8) | motorFeedbackBufferRight[3];
        rightFeedbackRpm = speedFeedback / 10.0;
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

// ---------------------------------------------------------------------
// Process Motor Feedback (Both Motors)
// ---------------------------------------------------------------------
void processMotorFeedback() {
  processLeftFeedback();
  processRightFeedback();
  // (For integration, we simply use the latest RPM values.)
}

// ---------------------------------------------------------------------
// Compute Motor Commands from the Latest Velocity Command
// ---------------------------------------------------------------------
void computeMotorCommands(int16_t forward, int16_t turn, int16_t *leftCmd, int16_t *rightCmd) {
  // forward: 16-bit signed, scaled by 100 (e.g., 64 means 0.64 m/s)
  // turn: 16-bit signed, scaled by 100 (e.g., -128 means -1.28 rad/s)
  
  // Convert forward speed to mm/s:
  float forwardSpeed = (forward / 100.0) * 1000.0;
  // Compute wheel circumference (mm)
  float wheelCircumference = WHEEL_DIAMETER * 3.14159;
  // Compute target RPM from forward speed:
  float targetRPM = (forwardSpeed / wheelCircumference) * 60.0;
  
  // Convert turn rate (in rad/s) and compute the differential term:
  float omega = turn / 100.0;
  float delta = (omega * TRACK_WIDTH * 60.0) / (2.0 * wheelCircumference);
  
  float leftRPM = targetRPM - delta;
  float rightRPM = targetRPM + delta;
  
  // Convert RPM to command value (multiply by 10)
  *leftCmd = (int16_t)(leftRPM * 10);
  *rightCmd = (int16_t)(rightRPM * 10);
  
  // Clamp to allowed motor range:
  if (*leftCmd > MAX_SPEED) *leftCmd = MAX_SPEED;
  if (*leftCmd < -MAX_SPEED) *leftCmd = -MAX_SPEED;
  if (*rightCmd > MAX_SPEED) *rightCmd = MAX_SPEED;
  if (*rightCmd < -MAX_SPEED) *rightCmd = -MAX_SPEED;
}

// ---------------------------------------------------------------------
// Integrate Robot Position Using Motor Feedback
// ---------------------------------------------------------------------
void integratePosition() {
  unsigned long currentTime = millis();
  if (lastIntegrationTime == 0) {
    lastIntegrationTime = currentTime;
    return;
  }
  float dt = (currentTime - lastIntegrationTime) / 1000.0; // dt in seconds
  lastIntegrationTime = currentTime;
  
  // Use the most recent RPM feedback values
  float leftRPM = leftFeedbackRpm;
  float rightRPM = rightFeedbackRpm;
  
  // Wheel circumference (mm)
  float wheelCircumference = WHEEL_DIAMETER * 3.14159;
  
  // Convert RPM to linear speed (mm/s)
  float vLeft = (leftRPM / 60.0) * wheelCircumference;
  float vRight = (rightRPM / 60.0) * wheelCircumference;
  
  // Average linear speed (mm/s) and angular velocity (rad/s)
  float v = (vLeft + vRight) / 2.0;
  float omega = (vRight - vLeft) / TRACK_WIDTH;
  
  // Update position (posX and posY in mm, theta in radians)
  posX += v * cos(theta) * dt;
  posY += v * sin(theta) * dt;
  theta += omega * dt;
}

// ---------------------------------------------------------------------
// LED Animation Functions
// ---------------------------------------------------------------------
void resetLEDAnimation() {
  ledPreviousMillis = millis();
  blinkState = false;
  currentLEDIndex = 0;
}

void updateLEDStripImmediate() {
  if (ledMode == 0) { // Off
    strip.clear();
    strip.show();
  } else if (ledMode == 1) { // Solid
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(ledRed, ledGreen, ledBlue));
    }
    strip.show();
  }
}

void updateLEDs() {
  unsigned long currentMillis = millis();
  
  switch (ledMode) {
    case 0: // Off – no animation
      break;
    case 1: // Solid – static color already set
      break;
    case 2: // Blink – toggle on/off every ledTime ms
      if (currentMillis - ledPreviousMillis >= ledTime) {
        ledPreviousMillis = currentMillis;
        blinkState = !blinkState;
        if (blinkState) {
          for (int i = 0; i < NUM_LEDS; i++) {
            strip.setPixelColor(i, strip.Color(ledRed, ledGreen, ledBlue));
          }
        } else {
          strip.clear();
        }
        strip.show();
      }
      break;
    case 3: // Ring – one LED on at a time, moving around the strip
      if (currentMillis - ledPreviousMillis >= ledTime) {
        ledPreviousMillis = currentMillis;
        currentLEDIndex = (currentLEDIndex + 1) % NUM_LEDS;
        strip.clear();
        strip.setPixelColor(currentLEDIndex, strip.Color(ledRed, ledGreen, ledBlue));
        strip.show();
      }
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------
// Serial1 (Host) Message Processing
// ---------------------------------------------------------------------
void processSerial1Messages() {
  while (Serial1.available() > 0) {
    uint8_t byteRead = Serial1.read();
    switch (serial1State) {
      case WAIT_FOR_SOM1:
        if (byteRead == 0x69)
          serial1State = WAIT_FOR_SOM2;
        break;
      case WAIT_FOR_SOM2:
        if (byteRead == 0x69) {
          serial1State = READ_PAYLOAD;
          serial1PayloadIndex = 0;
        } else {
          serial1State = WAIT_FOR_SOM1;
        }
        break;
      case READ_PAYLOAD:
        serial1PayloadBuffer[serial1PayloadIndex++] = byteRead;
        if (serial1PayloadIndex == 8) {
          // Check CRC over first 7 bytes of payload
          uint8_t crcCalc = calculate_crc8_maxim(serial1PayloadBuffer, 7);
          if (crcCalc == serial1PayloadBuffer[7]) {
            // Process valid payload
            processSerial1Payload(serial1PayloadBuffer);
          } else {
            // CRC error; optionally print debug info
            // Serial.println("Serial1 message CRC error.");
          }
          serial1State = WAIT_FOR_SOM1;
        }
        break;
    }
  }
}

void processSerial1Payload(uint8_t *payload) {
  uint8_t cmdID = payload[0];
  if (cmdID == 0x01) { // Movement Command
    // Bytes 1-2: forward speed, Bytes 3-4: turn rate
    int16_t forwardSpeed = ((int16_t)payload[1] << 8) | payload[2];
    int16_t turnRate = ((int16_t)payload[3] << 8) | payload[4];
    latestForwardSpeed = forwardSpeed;
    latestTurnRate = turnRate;
    
    // Compute and send motor commands immediately
    int16_t leftCmd, rightCmd;
    computeMotorCommands(latestForwardSpeed, latestTurnRate, &leftCmd, &rightCmd);
    sendMotorCommand(SerialLeft, leftCmd);
    sendMotorCommand(SerialRight, rightCmd);
    
    // Send back a Movement Response with the current position
    sendMovementResponse();
    
  } else if (cmdID == 0x02) { // LED Command
    // Byte 1: LED mode, Bytes 2-3: time parameter, Byte 4: Red, Byte 5: Green, Byte 6: Blue
    ledMode = payload[1];
    ledTime = ((uint16_t)payload[2] << 8) | payload[3];
    ledRed = payload[4];
    ledGreen = payload[5];
    ledBlue = payload[6];
    resetLEDAnimation();
    
    // For Off or Solid modes, update the strip immediately.
    if (ledMode == 0 || ledMode == 1)
      updateLEDStripImmediate();
      
    // Send LED Response
    sendLEDResponse();
    
  } else if (cmdID == 0x03) { // Status Request Command
    // For now, send a status response with zero values.
    sendStatusResponse();
  }
}

// ---------------------------------------------------------------------
// Send Movement Response (Response ID 0x81)
// ---------------------------------------------------------------------
void sendMovementResponse() {
  uint8_t response[10];
  response[0] = 0x69;
  response[1] = 0x69;
  response[2] = 0x81;  // Movement Response ID
  
  // Pack x, y, theta into 6 bytes.
  // For simplicity, posX and posY (in mm) are cast to int16_t.
  // Theta is scaled by 100 (i.e. in hundredths of a radian).
  int16_t x_fixed = (int16_t) posX;
  int16_t y_fixed = (int16_t) posY;
  int16_t theta_fixed = (int16_t)(theta * 100);
  
  response[3] = (x_fixed >> 8) & 0xFF;
  response[4] = x_fixed & 0xFF;
  response[5] = (y_fixed >> 8) & 0xFF;
  response[6] = y_fixed & 0xFF;
  response[7] = (theta_fixed >> 8) & 0xFF;
  response[8] = theta_fixed & 0xFF;
  
  // Compute CRC over payload bytes 2-8 (7 bytes)
  response[9] = calculate_crc8_maxim(response + 2, 7);
  Serial1.write(response, 10);
}

// ---------------------------------------------------------------------
// Send LED Response (Response ID 0x82)
// ---------------------------------------------------------------------
void sendLEDResponse() {
  uint8_t response[10];
  response[0] = 0x69;
  response[1] = 0x69;
  response[2] = 0x82;  // LED Response ID
  
  response[3] = ledMode;
  response[4] = ledRed;
  response[5] = ledGreen;
  response[6] = ledBlue;
  response[7] = (ledTime >> 8) & 0xFF;
  response[8] = ledTime & 0xFF;
  
  response[9] = calculate_crc8_maxim(response + 2, 7);
  Serial1.write(response, 10);
}

// ---------------------------------------------------------------------
// Send Status Response (Response ID 0x83)
// ---------------------------------------------------------------------
void sendStatusResponse() {
  uint8_t response[10];
  response[0] = 0x69;
  response[1] = 0x69;
  response[2] = 0x83;  // Status Response ID
  
  // Battery Voltage (2 bytes), Motor Temperature (2 bytes), Fault Code (1 byte), Reserved (1 byte)
  uint16_t batteryVoltage = 0;  // Not implemented
  uint16_t motorTemp = 0;       // Not implemented
  uint8_t faultCode = 0;
  
  response[3] = (batteryVoltage >> 8) & 0xFF;
  response[4] = batteryVoltage & 0xFF;
  response[5] = (motorTemp >> 8) & 0xFF;
  response[6] = motorTemp & 0xFF;
  response[7] = faultCode;
  response[8] = 0x00; // Reserved
  
  response[9] = calculate_crc8_maxim(response + 2, 7);
  Serial1.write(response, 10);
}

// ---------------------------------------------------------------------
// Setup Function
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);     // For debugging
  while (!Serial);          // Wait for Serial monitor
  Serial.println("Initializing Robot Controller...");
  
  // Initialize Serial1 for host communications
  Serial1.begin(115200);
  delay(200);
  
  // Initialize motor UARTs
  SerialLeft.begin(115200);
  delay(200);
  pinPeripheral(PIN_A2, PIO_SERCOM_ALT);  // Set A2 as TX for SERCOM4
  pinPeripheral(PIN_A3, PIO_SERCOM_ALT);  // Set A3 as RX for SERCOM4
  
  SerialRight.begin(115200);
  delay(200);
  pinPeripheral(PIN_A4, PIO_SERCOM_ALT);  // Set A4 as TX for SERCOM0
  pinPeripheral(PIN_A1, PIO_SERCOM_ALT);  // Set A1 as RX for SERCOM0
  
  Serial.println("Serial1, SerialLeft, and SerialRight initialized.");
  delay(200);
  // Initialize the LED strip
  strip.begin();
  strip.clear();
  strip.show();
  
  // Motor initialization: set motor IDs and mode.
  setMotorID(SerialLeft);
  setMotorID(SerialRight);
  delay(100);
  setMotorMode(SerialLeft);
  setMotorMode(SerialRight);
  Serial.println("Motors initialized to ID 1 and speed loop mode.");
  
  // Initialize robot state variables
  posX = 0.0;
  posY = 0.0;
  theta = 0.0;
  latestForwardSpeed = 0;
  latestTurnRate = 0;
  
  // Initialize LED state variables
  ledMode = 0;
  ledTime = 300;
  ledRed = 0;
  ledGreen = 0;
  ledBlue = 0;
  resetLEDAnimation();
  
  lastIntegrationTime = millis();
}

// ---------------------------------------------------------------------
// Main Loop Function
// ---------------------------------------------------------------------
void loop() {
  // Process all incoming host commands on Serial1
  processSerial1Messages();
  
  // Re-send motor commands based on the latest velocity command (to maintain motion)
  int16_t leftCmd, rightCmd;
  computeMotorCommands(latestForwardSpeed, latestTurnRate, &leftCmd, &rightCmd);
  sendMotorCommand(SerialLeft, leftCmd);
  sendMotorCommand(SerialRight, rightCmd);
  
  // Process motor feedback from both motors
  processMotorFeedback();
  
  // Integrate the motor feedback to update the robot's position
  integratePosition();
  
  // Update LED animations (for Blink and Ring modes)
  updateLEDs();
  
  delay(5);  // Small delay to yield time to other tasks
}
