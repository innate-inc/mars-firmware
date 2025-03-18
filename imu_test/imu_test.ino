#include <Wire.h>
#include <math.h>

// Gyroscope definitions
#define GYRO_ADDRESS     0x6A         // I2C address for LSM9DS1 gyro
#define WHO_AM_I         0x0F         // WHO_AM_I register for accelerometer/gyro
#define CTRL_REG1_G      0x10         // Gyroscope control register 1
#define OUT_X_L_G        0x18         // Gyro X-axis low byte
#define OUT_X_H_G        0x19         // Gyro X-axis high byte
#define OUT_Y_L_G        0x1A         // Gyro Y-axis low byte
#define OUT_Y_H_G        0x1B         // Gyro Y-axis high byte
#define OUT_Z_L_G        0x1C         // Gyro Z-axis low byte
#define OUT_Z_H_G        0x1D         // Gyro Z-axis high byte

// Accelerometer definitions (LSM9DS1 accel/mag part)
#define ACCEL_ADDRESS    0x1E         // I2C address for LSM9DS1 accelerometer (verify for your breakout)
#define CTRL_REG6_XL     0x20         // Accelerometer control register (for ODR & scale)
#define OUT_X_L_XL       0x28         // Accelerometer X-axis low byte
#define OUT_X_H_XL       0x29         // Accelerometer X-axis high byte
#define OUT_Y_L_XL       0x2A         // Accelerometer Y-axis low byte
#define OUT_Y_H_XL       0x2B         // Accelerometer Y-axis high byte
#define OUT_Z_L_XL       0x2C         // Accelerometer Z-axis low byte
#define OUT_Z_H_XL       0x2D         // Accelerometer Z-axis high byte

// Sensitivity constants
// For Gyro: ±500 dps --> 17.5 mdps/LSB (0.0175 dps per LSB)
#define GYRO_SENSITIVITY 0.0175
// For Accelerometer: assuming ±2g full-scale, sensitivity ~0.000061 g/LSB
#define ACCEL_SENSITIVITY 0.000061

#define SAMPLE_COUNT     2000

// Global bias variables (in dps)
float biasX = 0, biasY = 0, biasZ = 0;

// Global angle variables (in radians)
float roll = 0, pitch = 0, yaw = 0;
unsigned long previousTime = 0;

// Function prototypes
void writeRegister(uint8_t device, uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t device, uint8_t reg);
int16_t read16(uint8_t device, uint8_t reg);
void measureBias();
void initializeOrientation();
float normalizeAngle(float angleDeg);  // Normalize angle to [-180, 180]

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(100);

  // Check sensor communication by reading WHO_AM_I register from gyro
  uint8_t whoami = readRegister(GYRO_ADDRESS, WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);
  
  if (whoami != 0x68) {
    Serial.println("Gyro not detected or wrong I2C address!");
    while(1); // Halt execution if gyro is not found
  } else {
    Serial.println("Gyro detected.");
  }
  
  // Configure the gyroscope:
  // Write 0x68 to CTRL_REG1_G to enable normal mode, set output data rate to 119 Hz,
  // and use ±500 dps range.
  writeRegister(GYRO_ADDRESS, CTRL_REG1_G, 0x68);
  delay(100);  // Allow gyro to settle
  
  // Configure the accelerometer:
  // Write 0x60 to CTRL_REG6_XL to enable the accelerometer with a 119 Hz ODR and ±2g range.
  writeRegister(ACCEL_ADDRESS, CTRL_REG6_XL, 0x60);
  delay(100);  // Allow accelerometer to settle
  
  // Measure gyro bias using multiple samples
  measureBias();
  Serial.println("Gyro bias measured from 2000 samples:");
  Serial.print("Bias X (dps): ");
  Serial.println(biasX);
  Serial.print("Bias Y (dps): ");
  Serial.println(biasY);
  Serial.print("Bias Z (dps): ");
  Serial.println(biasZ);
  
  // Use accelerometer readings to set the initial orientation
  initializeOrientation();
  
  // Initialize timer for integration
  previousTime = millis();
}

void loop() {
  // Calculate the elapsed time (in seconds)
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;  // Convert ms to seconds
  
  // Read raw gyro data for each axis (low byte first)
  int16_t rawX = read16(GYRO_ADDRESS, OUT_X_L_G);
  int16_t rawY = read16(GYRO_ADDRESS, OUT_Y_L_G);
  int16_t rawZ = read16(GYRO_ADDRESS, OUT_Z_L_G);
  
  // Convert raw values to degrees per second (dps) and subtract bias
  float p = (rawX * GYRO_SENSITIVITY) - biasX;  // Angular velocity about X (roll rate) in dps
  float q = (rawY * GYRO_SENSITIVITY) - biasY;  // Angular velocity about Y (pitch rate) in dps
  float r = (rawZ * GYRO_SENSITIVITY) - biasZ;  // Angular velocity about Z (yaw rate) in dps
  
  // Convert from degrees per second to radians per second
  float radPerSecFactor = PI / 180.0;
  float p_rad = p * radPerSecFactor;
  float q_rad = q * radPerSecFactor;
  float r_rad = r * radPerSecFactor;
  
  // Tait-Bryan angle derivatives for ZYX (yaw-pitch-roll) sequence:
  // roll (φ), pitch (θ), yaw (ψ)
  float phi_dot   = p_rad + sin(roll) * tan(pitch) * q_rad + cos(roll) * tan(pitch) * r_rad;
  float theta_dot = cos(roll) * q_rad - sin(roll) * r_rad;
  float psi_dot   = (sin(roll) / cos(pitch)) * q_rad + (cos(roll) / cos(pitch)) * r_rad;
  
  // Integrate to update angles (in radians)
  roll  += phi_dot * dt;
  pitch += theta_dot * dt;
  yaw   += psi_dot * dt;
  
  // Convert angles to degrees
  float roll_deg  = roll * (180.0 / PI);
  float pitch_deg = pitch * (180.0 / PI);
  float yaw_deg   = yaw * (180.0 / PI);
  
  // Normalize angles to the range [-180, 180]
  roll_deg  = normalizeAngle(roll_deg);
  pitch_deg = normalizeAngle(pitch_deg);
  yaw_deg   = normalizeAngle(yaw_deg);
  
  // Print the updated angles to the serial monitor
  Serial.print("Roll: ");
  Serial.print(roll_deg);
  Serial.print(" | Pitch: ");
  Serial.print(pitch_deg);
  Serial.print(" | Yaw: ");
  Serial.println(yaw_deg);
  
  // Update previous time and wait a short interval before the next loop
  previousTime = currentTime;
  delay(10);
}

// Normalize an angle (in degrees) to the range [-180, 180]
float normalizeAngle(float angleDeg) {
  while(angleDeg > 180.0) angleDeg -= 360.0;
  while(angleDeg < -180.0) angleDeg += 360.0;
  return angleDeg;
}

// Use accelerometer readings to initialize the orientation (roll & pitch)
void initializeOrientation() {
  // Read accelerometer data (low bytes first)
  int16_t rawAX = read16(ACCEL_ADDRESS, OUT_X_L_XL);
  int16_t rawAY = read16(ACCEL_ADDRESS, OUT_Y_L_XL);
  int16_t rawAZ = read16(ACCEL_ADDRESS, OUT_Z_L_XL);
  
  // Convert raw readings to g's
  float accX = rawAX * ACCEL_SENSITIVITY;
  float accY = rawAY * ACCEL_SENSITIVITY;
  float accZ = rawAZ * ACCEL_SENSITIVITY;
  
  // Compute initial roll and pitch from accelerometer data.
  // These formulas assume that the sensor is at rest and gravity is the only acceleration.
  roll  = atan2(accY, accZ);
  pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ));
  yaw   = 0;  // Yaw cannot be determined from the accelerometer alone.
  
  Serial.println("Initial orientation set using accelerometer:");
  Serial.print("Initial Roll (deg): ");
  Serial.println(roll * 180.0 / PI);
  Serial.print("Initial Pitch (deg): ");
  Serial.println(pitch * 180.0 / PI);
  Serial.println("Initial Yaw (deg): 0");
}

// Measure gyro bias over a number of samples
void measureBias() {
  long sumX = 0, sumY = 0, sumZ = 0;
  
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int16_t rawX = read16(GYRO_ADDRESS, OUT_X_L_G);
    int16_t rawY = read16(GYRO_ADDRESS, OUT_Y_L_G);
    int16_t rawZ = read16(GYRO_ADDRESS, OUT_Z_L_G);
    
    sumX += rawX;
    sumY += rawY;
    sumZ += rawZ;
    
    delay(10);
  }
  
  // Compute average bias and convert to dps
  biasX = (sumX / (float)SAMPLE_COUNT) * GYRO_SENSITIVITY;
  biasY = (sumY / (float)SAMPLE_COUNT) * GYRO_SENSITIVITY;
  biasZ = (sumZ / (float)SAMPLE_COUNT) * GYRO_SENSITIVITY;
}

// Write a value to a specific register on a given device
void writeRegister(uint8_t device, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read a single byte from a register on a given device
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

// Read a 16-bit value (low byte first) from a given device and register
int16_t read16(uint8_t device, uint8_t reg) {
  uint8_t low = 0, high = 0;
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.endTransmission();
  
  // Request 2 bytes from the device
  Wire.requestFrom(device, (uint8_t)2);
  if (Wire.available() >= 2) {
    low = Wire.read();
    high = Wire.read();
  }
  
  // Combine the two bytes (assuming little-endian order)
  return (int16_t)(high << 8 | low);
}

