#include <Arduino.h>
#include <Wire.h>
#include "wiring_private.h"    // Needed for pinPeripheral()
#include <Adafruit_NeoPixel.h>

// ---------------------------------------------------------------------
// Constants and Definitions
// ---------------------------------------------------------------------
#define MOTOR_ID        0x01           // Motor ID for both motors
#define WHEEL_DIAMETER  73.0           // Wheel diameter in mm
#define MAX_SPEED       2100           // Motor command range: -2100 to 2100
#define TRACK_WIDTH     140.0          // Distance between the two motors (mm)

#define LED_PIN         2              // NeoPixel strip pin
#define NUM_LEDS        39             // Number of LEDs

#define MOTOR_BUFFER_SIZE 64           // Size of motor feedback buffer

// ------------------ IMU Definitions ------------------
#define IMU_GYRO_ADDRESS       0x6A         // IMU I2C address
#define IMU_WHO_AM_I           0x0F         // WHO_AM_I register
#define IMU_CTRL_REG1_G        0x10         // Gyro control register 1
#define IMU_OUT_Z_L_G          0x1C         // Gyro Z-axis (low byte) used for heading (yaw)
#define IMU_GYRO_SENSITIVITY   0.00875      // dps per LSB for ±245 dps
#define IMU_SAMPLE_COUNT       12000        // Number of samples for bias measurement

// ---------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------
// Robot state: position (mm) and heading (theta in radians)
float posX = 0.0, posY = 0.0, theta = 0.0;

// Latest commands received from host (scaled by 100)
int16_t latestForwardSpeed = 0;
int16_t latestTurnRate = 0;

// LED state variables
uint8_t ledMode = 0;      // 0 = Off, 1 = Solid, 2 = Blink, 3 = Ring
uint16_t ledTime = 300;   // LED time parameter in ms
uint8_t ledRed = 0, ledGreen = 0, ledBlue = 0;
unsigned long ledPreviousMillis = 0;
bool blinkState = false;
int currentLEDIndex = 0;

// Timing for position integration
unsigned long lastIntegrationTime = 0;

// Motor feedback buffers & variables
uint8_t motorFeedbackBufferLeft[MOTOR_BUFFER_SIZE];
uint8_t motorFeedbackIndexLeft = 0;
float leftFeedbackRpm = 0.0;
bool leftFeedbackReady = false;

uint8_t motorFeedbackBufferRight[MOTOR_BUFFER_SIZE];
uint8_t motorFeedbackIndexRight = 0;
float rightFeedbackRpm = 0.0;
bool rightFeedbackReady = false;

// Serial1 (host) message parser variables
enum Serial1ParseState { WAIT_FOR_SOM1, WAIT_FOR_SOM2, READ_PAYLOAD };
Serial1ParseState serial1State = WAIT_FOR_SOM1;
uint8_t serial1PayloadBuffer[8];  // 7 data + 1 CRC
int serial1PayloadIndex = 0;

// ------------------ IMU Global Variables ------------------
float imuYaw = 0;       // Integrated yaw in degrees (from IMU)
float imuBiasZ = 0;     // Gyro Z-axis bias (dps)
unsigned long previousIMUTime = 0;

// ---------------------------------------------------------------------
// Motor UART Setup (SERCOM/UART objects)
// ---------------------------------------------------------------------
// Left Motor: Using SERCOM4 with TX on PIN_A2 and RX on PIN_A3
Uart SerialLeft(&sercom4, PIN_A3, PIN_A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);
// Right Motor: Using SERCOM0 with TX on PIN_A4 and RX on PIN_A1
Uart SerialRight(&sercom0, PIN_A1, PIN_A4, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// SERCOM interrupt handlers for SerialLeft (SERCOM4)
void SERCOM4_0_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_1_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_2_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_3_Handler() { SerialLeft.IrqHandler(); }
// SERCOM interrupt handlers for SerialRight (SERCOM0)
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

void imuWriteRegister(uint8_t reg, uint8_t value);
uint8_t imuReadRegister(uint8_t reg);
int16_t imuRead16(uint8_t reg);
void imuMeasureBias();
void updateIMUHeading();

// ---------------------------------------------------------------------
// CRC-8/MAXIM Calculation Function (Polynomial 0x8C, init 0x00)
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
  cmd[3] = MOTOR_ID;
  cmd[4] = 0x00; cmd[5] = 0x00; cmd[6] = 0x00; cmd[7] = 0x00; cmd[8] = 0x00;
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
  cmd[3] = 0x00; cmd[4] = 0x00; cmd[5] = 0x00; cmd[6] = 0x00; cmd[7] = 0x00; cmd[8] = 0x00;
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
  cmd[4] = 0x00; cmd[5] = 0x00;
  cmd[6] = 0x01;  // Acceleration time
  cmd[7] = 0x00;  // No braking
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
      for (uint8_t i = 1; i < MOTOR_BUFFER_SIZE; i++)
        motorFeedbackBufferLeft[i - 1] = motorFeedbackBufferLeft[i];
      motorFeedbackBufferLeft[MOTOR_BUFFER_SIZE - 1] = byteRead;
    }
    while (motorFeedbackIndexLeft >= 10) {
      if (calculate_crc8_maxim(motorFeedbackBufferLeft, 9) == motorFeedbackBufferLeft[9]) {
        int16_t speedFeedback = ((int16_t)motorFeedbackBufferLeft[2] << 8) | motorFeedbackBufferLeft[3];
        leftFeedbackRpm = speedFeedback / 10.0;
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
}

// ---------------------------------------------------------------------
// Compute Motor Commands from Latest Velocity Command
// ---------------------------------------------------------------------
void computeMotorCommands(int16_t forward, int16_t turn, int16_t *leftCmd, int16_t *rightCmd) {
  float forwardSpeed = (forward / 100.0) * 1000.0; // Convert to mm/s
  float wheelCircumference = WHEEL_DIAMETER * 3.14159;
  
  float targetRPM = (forwardSpeed / wheelCircumference) * 60.0;
  
  float omega = turn / 100.0;
  float delta = (omega * TRACK_WIDTH * 60.0) / (2.0 * wheelCircumference);
  
  float leftRPM = -(targetRPM - delta);
  float rightRPM = targetRPM + delta;
  
  *leftCmd = (int16_t)(leftRPM * 10);
  *rightCmd = (int16_t)(rightRPM * 10);
  
  if (*leftCmd > MAX_SPEED) *leftCmd = MAX_SPEED;
  if (*leftCmd < -MAX_SPEED) *leftCmd = -MAX_SPEED;
  if (*rightCmd > MAX_SPEED) *rightCmd = MAX_SPEED;
  if (*rightCmd < -MAX_SPEED) *rightCmd = -MAX_SPEED;
}

// ---------------------------------------------------------------------
// Integrate Robot Position Using Motor Feedback and IMU Heading
// ---------------------------------------------------------------------
void integratePosition() {
  unsigned long currentTime = millis();
  if (lastIntegrationTime == 0) {
    lastIntegrationTime = currentTime;
    return;
  }
  float dt = (currentTime - lastIntegrationTime) / 1000.0;
  lastIntegrationTime = currentTime;
  
  float wheelCircumference = WHEEL_DIAMETER * 3.14159;
  float vLeft = (-leftFeedbackRpm / 60.0) * wheelCircumference;
  float vRight = (rightFeedbackRpm / 60.0) * wheelCircumference;
  float v = (vLeft + vRight) / 2.0;
  
  // Use the IMU’s integrated yaw (converted to radians) for heading.
  theta = imuYaw * (PI / 180.0);
  
  posX += v * cos(theta) * dt;
  posY += v * sin(theta) * dt;
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
  if (ledMode == 0) {
    strip.clear();
    strip.show();
  } else if (ledMode == 1) {
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(ledRed, ledGreen, ledBlue));
    }
    strip.show();
  }
}

void updateLEDs() {
  unsigned long currentMillis = millis();
  switch (ledMode) {
    case 0:
      break;
    case 1:
      break;
    case 2:
      if (currentMillis - ledPreviousMillis >= ledTime) {
        ledPreviousMillis = currentMillis;
        blinkState = !blinkState;
        if (blinkState) {
          for (int i = 0; i < NUM_LEDS; i++)
            strip.setPixelColor(i, strip.Color(ledRed, ledGreen, ledBlue));
        } else {
          strip.clear();
        }
        strip.show();
      }
      break;
    case 3:
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
          uint8_t crcCalc = calculate_crc8_maxim(serial1PayloadBuffer, 7);
          if (crcCalc == serial1PayloadBuffer[7])
            processSerial1Payload(serial1PayloadBuffer);
          serial1State = WAIT_FOR_SOM1;
        }
        break;
    }
  }
}

void processSerial1Payload(uint8_t *payload) {
  uint8_t cmdID = payload[0];
  if (cmdID == 0x01) { // Movement Command
    int16_t forwardSpeed = ((int16_t)payload[1] << 8) | payload[2];
    int16_t turnRate = ((int16_t)payload[3] << 8) | payload[4];
    latestForwardSpeed = forwardSpeed;
    latestTurnRate = turnRate;
    
    int16_t leftCmd, rightCmd;
    computeMotorCommands(latestForwardSpeed, latestTurnRate, &leftCmd, &rightCmd);
    sendMotorCommand(SerialLeft, leftCmd);
    sendMotorCommand(SerialRight, rightCmd);
    
    Serial.println("Sending Movement Response");
    sendMovementResponse();
    
  } else if (cmdID == 0x02) { // LED Command
    ledMode = payload[1];
    ledTime = ((uint16_t)payload[2] << 8) | payload[3];
    ledRed = payload[4];
    ledGreen = payload[5];
    ledBlue = payload[6];
    resetLEDAnimation();
    
    if (ledMode == 0 || ledMode == 1)
      updateLEDStripImmediate();
      
    sendLEDResponse();
    
  } else if (cmdID == 0x03) { // Status Request Command
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
  response[2] = 0x81;
  
  int16_t x_fixed = (int16_t) posX;
  int16_t y_fixed = (int16_t) posY;
  int16_t theta_fixed = (int16_t)(theta * 100);
  
  response[3] = (x_fixed >> 8) & 0xFF;
  response[4] = x_fixed & 0xFF;
  response[5] = (y_fixed >> 8) & 0xFF;
  response[6] = y_fixed & 0xFF;
  response[7] = (theta_fixed >> 8) & 0xFF;
  response[8] = theta_fixed & 0xFF;
  
  response[9] = calculate_crc8_maxim(response + 2, 7);
  Serial.println("Sending Movement Response");
  Serial1.write(response, 10);
}

// ---------------------------------------------------------------------
// Send LED Response (Response ID 0x82)
// ---------------------------------------------------------------------
void sendLEDResponse() {
  uint8_t response[10];
  response[0] = 0x69;
  response[1] = 0x69;
  response[2] = 0x82;
  
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
  response[2] = 0x83;
  
  uint16_t batteryVoltage = 0;  // Not implemented
  uint16_t motorTemp = 0;       // Not implemented
  uint8_t faultCode = 0;
  
  response[3] = (batteryVoltage >> 8) & 0xFF;
  response[4] = batteryVoltage & 0xFF;
  response[5] = (motorTemp >> 8) & 0xFF;
  response[6] = motorTemp & 0xFF;
  response[7] = faultCode;
  response[8] = 0x00;
  
  response[9] = calculate_crc8_maxim(response + 2, 7);
  Serial1.write(response, 10);
}

// ---------------------------------------------------------------------
// IMU Functions for Heading Integration
// ---------------------------------------------------------------------
void imuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_GYRO_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t imuReadRegister(uint8_t reg) {
  Wire.beginTransmission(IMU_GYRO_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(IMU_GYRO_ADDRESS, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

int16_t imuRead16(uint8_t reg) {
  uint8_t low = 0, high = 0;
  Wire.beginTransmission(IMU_GYRO_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(IMU_GYRO_ADDRESS, (uint8_t)2);
  if (Wire.available() >= 2) {
    low = Wire.read();
    high = Wire.read();
  }
  return (int16_t)(high << 8 | low);
}

void imuMeasureBias() {
  long sumZ = 0;
  for (int i = 0; i < IMU_SAMPLE_COUNT; i++) {
    int16_t rawZ = imuRead16(IMU_OUT_Z_L_G);
    sumZ += rawZ;
    delay(10);
  }
  imuBiasZ = (sumZ / (float)IMU_SAMPLE_COUNT) * IMU_GYRO_SENSITIVITY;
  Serial.print("IMU Bias Z (dps): ");
  Serial.println(imuBiasZ);
}

void updateIMUHeading() {
  unsigned long currentTime = millis();
  float dt = (currentTime - previousIMUTime) / 1000.0;
  previousIMUTime = currentTime;
  
  int16_t rawZ = imuRead16(IMU_OUT_Z_L_G);
  float gyroZ = (rawZ * IMU_GYRO_SENSITIVITY) - imuBiasZ;
  
  imuYaw += gyroZ * dt; // Update integrated yaw (degrees)
}

// ---------------------------------------------------------------------
// Setup Function
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Robot Controller with IMU Heading...");
  
  Wire.begin();
  delay(100);
  
  // Initialize IMU and check WHO_AM_I
  uint8_t imuWhoAmI = imuReadRegister(IMU_WHO_AM_I);
  Serial.print("IMU WHO_AM_I: 0x");
  Serial.println(imuWhoAmI, HEX);
  if (imuWhoAmI != 0x68) {
    Serial.println("IMU sensor not detected!");
    while(1);
  }
  
  imuWriteRegister(IMU_CTRL_REG1_G, 0x60);  // Configure gyro (normal mode, 119Hz, ±245 dps)
  delay(100);
  imuMeasureBias();
  previousIMUTime = millis();
  
  // Initialize robot state
  posX = 0.0; posY = 0.0; theta = 0.0;
  latestForwardSpeed = 0; latestTurnRate = 0;
  
  Serial1.begin(115200);
  delay(200);
  
  // Initialize motor UARTs
  SerialLeft.begin(115200);
  delay(200);
  pinPeripheral(PIN_A2, PIO_SERCOM_ALT);
  pinPeripheral(PIN_A3, PIO_SERCOM_ALT);
  
  SerialRight.begin(115200);
  delay(200);
  pinPeripheral(PIN_A4, PIO_SERCOM_ALT);
  pinPeripheral(PIN_A1, PIO_SERCOM_ALT);
  
  // Initialize LED strip
  strip.begin();
  strip.clear();
  strip.show();
  
  // Motor initialization
  setMotorID(SerialLeft);
  setMotorID(SerialRight);
  delay(100);
  setMotorMode(SerialLeft);
  setMotorMode(SerialRight);
  Serial.println("Motors initialized.");
  
  lastIntegrationTime = millis();
}

// ---------------------------------------------------------------------
// Main Loop
// ---------------------------------------------------------------------
void loop() {
  processSerial1Messages();
  
  int16_t leftCmd, rightCmd;
  computeMotorCommands(latestForwardSpeed, latestTurnRate, &leftCmd, &rightCmd);
  sendMotorCommand(SerialLeft, leftCmd);
  sendMotorCommand(SerialRight, rightCmd);
  
  processMotorFeedback();
  
  updateIMUHeading();
  integratePosition();
  
  updateLEDs();
  
  delay(5);
}
