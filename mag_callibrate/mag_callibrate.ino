#include <Arduino.h>
#include "wiring_private.h"    // Needed for pinPeripheral()
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>    // IMU library

// ---------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------
#define WHEEL_DIAMETER      69      // Wheel diameter in mm
#define TRACK_WIDTH         189     // Distance between the two motors in mm
#define MAX_SPEED           2100    // Motor command range: -2100 to 2100
#define MOTOR_ID            0x01    // Both motors set to ID 1

#define SAMPLE_INTERVAL_MS  20      // Record IMU every 20ms (~50Hz)
#define TEST_DURATION_MS    40000   // 40 seconds
#define TURN_RATE_RADS      1.0     // Desired angular velocity (rad/s)

// Max samples buffer
const int MAX_SAMPLES = TEST_DURATION_MS / SAMPLE_INTERVAL_MS + 10;
int16_t magX[MAX_SAMPLES], magY[MAX_SAMPLES];
int sampleCount = 0;

// IMU object
LSM9DS1 imu;

// Serial UARTs for motors
Uart SerialLeft(&sercom4, PIN_A3, PIN_A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);
Uart SerialRight(&sercom0, PIN_A1, PIN_A4, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// ISR handlers
void SERCOM4_0_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_1_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_2_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_3_Handler() { SerialLeft.IrqHandler(); }
void SERCOM0_0_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_1_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_2_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_3_Handler() { SerialRight.IrqHandler(); }

// ---------------------------------------------------------------------
// CRC-8/MAXIM Calculation Function (Polynomial 0x8C)
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
// Motor Initialization and Commands
// ---------------------------------------------------------------------
void setMotorID(Uart &motorSerial) {
  uint8_t cmd[10] = {0xAA, 0x55, 0x53, MOTOR_ID, 0,0,0,0,0,0};
  cmd[9] = calculate_crc8_maxim(cmd, 9);
  for (int i = 0; i < 5; i++) {
    motorSerial.write(cmd, 10);
    delay(20);
  }
}

void setMotorMode(Uart &motorSerial) {
  uint8_t cmd[10] = {MOTOR_ID, 0xA0, 0x02, 0,0,0,0,0,0,0};
  cmd[9] = calculate_crc8_maxim(cmd, 9);
  motorSerial.write(cmd, 10);
  delay(20);
}

void sendMotorCommand(Uart &motorSerial, int16_t speed) {
  uint8_t cmd[10] = {0};
  cmd[0] = MOTOR_ID;
  cmd[1] = 0x64;             // Drive command
  cmd[2] = (speed >> 8) & 0xFF;
  cmd[3] = speed & 0xFF;
  cmd[6] = 0x01;             // Acceleration time = 1ms per 1rpm
  cmd[9] = calculate_crc8_maxim(cmd, 9);
  motorSerial.write(cmd, 10);
}

void computeTurnCommands(float omega, int16_t &leftCmd, int16_t &rightCmd) {
  float v = (omega * TRACK_WIDTH / 2.0);         // mm/s at each wheel
  float wheelCirc = WHEEL_DIAMETER * PI;
  float rpm = (v / wheelCirc) * 60.0;
  leftCmd  = (int16_t)(rpm * 10);
  rightCmd = (int16_t)( rpm * 10);
  leftCmd  = constrain(leftCmd,  -MAX_SPEED, MAX_SPEED);
  rightCmd = constrain(rightCmd, -MAX_SPEED, MAX_SPEED);
}

// ---------------------------------------------------------------------
// Setup: initialize Serial, IMU, motors, then run test
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== Magnetometer Calibration & Spin Test ===");
  Wire.begin();
  // IMU init
  if (!imu.begin()) {
    Serial.println("Failed to detect LSM9DS1 IMU");
    while (1);
  }
  delay(10000);

  // Motor UART init
  SerialLeft.begin(115200);
  delay(200);
  pinPeripheral(PIN_A2, PIO_SERCOM_ALT);
  pinPeripheral(PIN_A3, PIO_SERCOM_ALT);

  SerialRight.begin(115200);
  delay(200);
  pinPeripheral(PIN_A4, PIO_SERCOM_ALT);
  pinPeripheral(PIN_A1, PIO_SERCOM_ALT);

  Serial.println("SerialLeft/Right initialized.");

  // Motor setup: ID and speed mode
  setMotorID(SerialLeft);
  setMotorID(SerialRight);
  delay(100);
  setMotorMode(SerialLeft);
  setMotorMode(SerialRight);
  Serial.println("Motors set to ID and speed loop mode.");

  // Compute turn commands
  int16_t leftCmd, rightCmd;
  computeTurnCommands(TURN_RATE_RADS, leftCmd, rightCmd);

  // Run spin and record mag data
  unsigned long startTime = millis();
  while (millis() - startTime < TEST_DURATION_MS) {
    sendMotorCommand(SerialLeft, leftCmd);
    sendMotorCommand(SerialRight, rightCmd);

    imu.readMag();
    if (sampleCount < MAX_SAMPLES) {
      magX[sampleCount] = imu.mx;
      magY[sampleCount] = imu.my;
      sampleCount++;
    }
    delay(SAMPLE_INTERVAL_MS);
  }

  // Stop motors
  sendMotorCommand(SerialLeft, 0);
  sendMotorCommand(SerialRight, 0);

  // Print raw data
  Serial.println("---BEGIN DATA---");
  for (int i = 0; i < sampleCount; i++) {
    Serial.print(magX[i]);
    Serial.print(',');
    Serial.println(magY[i]);
  }
  Serial.println("---END DATA---");

  // Compute calibration
  int16_t minX = INT16_MAX, maxX = INT16_MIN;
  int16_t minY = INT16_MAX, maxY = INT16_MIN;
  for (int i = 0; i < sampleCount; i++) {
    minX = min(minX, magX[i]); maxX = max(maxX, magX[i]);
    minY = min(minY, magY[i]); maxY = max(maxY, magY[i]);
  }
  float hardOffsetX = (minX + maxX) / 2.0;
  float hardOffsetY = (minY + maxY) / 2.0;
  float scaleX = (maxX - minX) / 2.0;
  float scaleY = (maxY - minY) / 2.0;
  float avgScale = (scaleX + scaleY) / 2.0;
  float softScaleX = avgScale / scaleX;
  float softScaleY = avgScale / scaleY;

  // Output
  Serial.println("Calibration Results:");
  Serial.print(" Hard Offsets -> X: "); Serial.print(hardOffsetX);
  Serial.print("  Y: "); Serial.println(hardOffsetY);
  Serial.print(" Soft Scales -> X: "); Serial.print(softScaleX);
  Serial.print("  Y: "); Serial.println(softScaleY);
  Serial.println("Calibration complete.");
}

void loop() {
  // Nothing
}
