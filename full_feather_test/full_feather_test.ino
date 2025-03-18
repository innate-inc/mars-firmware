#include <Wire.h>
#include <Arduino.h>
#include "wiring_private.h"    // Needed for pinPeripheral()
#include <Adafruit_NeoPixel.h>

// ------------------------------
// IMU DEFINITIONS & VARIABLES
// ------------------------------
#define GYRO_ADDRESS       0x6A         // IMU I2C address
#define WHO_AM_I           0x0F
#define CTRL_REG1_G        0x10
#define OUT_Z_L_G          0x1C         // Only use Z-axis for heading (yaw)
#define GYRO_SENSITIVITY   0.00875      // dps per LSB for ±245 dps
#define SAMPLE_COUNT       12000

// Global IMU variables (we only need yaw for heading)
float biasZ = 0;
float yaw = 0;                 // IMU integrated yaw (in degrees)
unsigned long previousIMUTime = 0;

// IMU function prototypes
void writeRegister(uint8_t device, uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t device, uint8_t reg);
int16_t read16(uint8_t device, uint8_t reg);
void measureBias();
void updateIMU();

// ------------------------------
// ROBOT (WHEEL ODOMETRY) DEFINITIONS & VARIABLES
// ------------------------------
#define WHEEL_DIAMETER  73.0   // in mm
#define TRACK_WIDTH     140.0  // in mm

// Robot state (position in mm, heading in radians)
float posX = 0.0, posY = 0.0;
float theta = 0.0;             // robot heading; will be updated from the IMU’s yaw

// Integration timing for position updates
unsigned long lastIntegrationTime = 0;

// (Other definitions for motor control, LED control, Serial1, etc. remain unchanged.)
// …
 
// ------------------------------
// SETUP FUNCTION (Combined Initialization)
// ------------------------------
void setup() {
  Serial.begin(115200);
  // Initialize Serial1, motors, LEDs, etc.
  // … (existing robot initialization code)

  // ----------- IMU Initialization -----------
  Wire.begin();
  delay(100);
  uint8_t whoami = readRegister(GYRO_ADDRESS, WHO_AM_I);
  if (whoami != 0x68) {  // Expected WHO_AM_I for this sensor
    Serial.println("IMU sensor not detected!");
    while(1); // halt if sensor is not found
  }
  // Set up gyroscope: normal mode, 119 Hz, ±245 dps
  writeRegister(GYRO_ADDRESS, CTRL_REG1_G, 0x60);
  delay(100);
  // Measure the Z-axis bias from many samples
  measureBias();
  previousIMUTime = millis();

  // Initialize the position integration timer
  lastIntegrationTime = millis();
}

// ------------------------------
// MAIN LOOP
// ------------------------------
void loop() {
  // Process host commands, send motor commands, update LEDs, etc.
  processSerial1Messages();

  // Send motor commands to maintain motion
  int16_t leftCmd, rightCmd;
  computeMotorCommands(latestForwardSpeed, latestTurnRate, &leftCmd, &rightCmd);
  sendMotorCommand(SerialLeft, leftCmd);
  sendMotorCommand(SerialRight, rightCmd);
  
  // Process motor feedback from both motors (for forward speed)
  processMotorFeedback();
  
  // ----- NEW: Update IMU heading before position integration -----
  updateIMU();
  
  // ----- MODIFIED: Integrate Position Using Wheel Forward Speed and IMU Heading -----
  integratePosition();
  
  // Update LED animations, etc.
  updateLEDs();
  
  delay(5);  // Small delay for task scheduling
}

// ------------------------------
// NEW FUNCTION: updateIMU()
// This function reads the gyro’s Z-axis, removes bias, and integrates to update yaw.
// ------------------------------
void updateIMU() {
  unsigned long currentTime = millis();
  float dt = (currentTime - previousIMUTime) / 1000.0;  // dt in seconds
  previousIMUTime = currentTime;
  
  int16_t rawZ = read16(GYRO_ADDRESS, OUT_Z_L_G);
  float gyroZ = (rawZ * GYRO_SENSITIVITY) - biasZ;
  
  // Integrate gyro Z-axis to update yaw
  yaw += gyroZ * dt;
}

// ------------------------------
// MODIFIED FUNCTION: integratePosition()
// Instead of computing angular velocity (omega) from wheel speeds to update theta,
// we now use the IMU’s yaw (converted to radians) for heading.
// The forward speed is still computed from wheel (motor) feedback.
// ------------------------------
void integratePosition() {
  unsigned long currentTime = millis();
  if (lastIntegrationTime == 0) {
    lastIntegrationTime = currentTime;
    return;
  }
  float dt = (currentTime - lastIntegrationTime) / 1000.0;
  lastIntegrationTime = currentTime;
  
  float wheelCircumference = WHEEL_DIAMETER * 3.14159;
  
  // Compute forward speed (v) using motor feedback (in mm/s)
  float vLeft = (-leftFeedbackRpm / 60.0) * wheelCircumference;
  float vRight = (rightFeedbackRpm / 60.0) * wheelCircumference;
  float v = (vLeft + vRight) / 2.0;
  
  // ---- Key Change: Use IMU heading ----
  // Convert IMU yaw (which is in degrees) to radians if needed.
  // (If you prefer to keep yaw in degrees, adjust your trig functions accordingly.)
  theta = yaw * (PI / 180.0);
  
  // Update the robot's position using the forward speed and IMU heading
  posX += v * cos(theta) * dt;
  posY += v * sin(theta) * dt;
}

// ------------------------------
// IMU HELPER FUNCTIONS
// ------------------------------
void writeRegister(uint8_t device, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t device, uint8_t reg) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(device, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

int16_t read16(uint8_t device, uint8_t reg) {
  uint8_t low = 0, high = 0;
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(device, (uint8_t)2);
  if (Wire.available() >= 2) {
    low = Wire.read();
    high = Wire.read();
  }
  return (int16_t)(high << 8 | low);
}

// Measure the Z-axis bias over many samples
void measureBias() {
  long sumZ = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int16_t rawZ = read16(GYRO_ADDRESS, OUT_Z_L_G);
    sumZ += rawZ;
    delay(10); // Delay between samples
  }
  biasZ = (sumZ / (float)SAMPLE_COUNT) * GYRO_SENSITIVITY;
  Serial.print("Measured Bias Z (dps): ");
  Serial.println(biasZ);
}
