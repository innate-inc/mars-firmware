#include <Wire.h>

#define GYRO_ADDRESS 0x6A         // I2C address for LSM9DS1 (adjust if needed)
#define WHO_AM_I     0x0F         // WHO_AM_I register for accelerometer/gyro
#define CTRL_REG1_G  0x10         // Gyroscope control register 1
#define OUT_X_L_G    0x18         // Gyro X-axis low byte
#define OUT_X_H_G    0x19         // Gyro X-axis high byte
#define OUT_Y_L_G    0x1A         // Gyro Y-axis low byte
#define OUT_Y_H_G    0x1B         // Gyro Y-axis high byte
#define OUT_Z_L_G    0x1C         // Gyro Z-axis low byte
#define OUT_Z_H_G    0x1D         // Gyro Z-axis high byte

// Sensitivity for ±245 dps full-scale: 8.75 mdps/LSB (0.00875 dps per LSB)
#define GYRO_SENSITIVITY 0.00875
#define SAMPLE_COUNT 12000

// Global bias variables (in dps)
float biasX = 0, biasY = 0, biasZ = 0;

// Global angle variables (in degrees)
float roll = 0, pitch = 0, yaw = 0;
unsigned long previousTime = 0;

// Function prototypes
void writeRegister(uint8_t device, uint8_t reg, uint8_t value);
uint8_t readRegister(uint8_t device, uint8_t reg);
int16_t read16(uint8_t device, uint8_t reg);
void measureBias();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(100);

  // Check sensor communication by reading WHO_AM_I register
  uint8_t whoami = readRegister(GYRO_ADDRESS, WHO_AM_I);
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoami, HEX);
  
  if (whoami != 0x68) {
    Serial.println("Sensor not detected or wrong I2C address!");
    while(1); // Halt execution if sensor is not found
  } else {
    Serial.println("Sensor detected.");
  }
  
  // Configure the gyroscope:
  // Write 0x60 to CTRL_REG1_G to enable normal mode, set output data rate to 119 Hz,
  // and use ±245 dps range.
  writeRegister(GYRO_ADDRESS, CTRL_REG1_G, 0x60);
  delay(100);  // Allow sensor to settle
  
  // Measure bias using multiple samples
  measureBias();
  Serial.println("Bias measured from 500 samples:");
  Serial.print("Bias X (dps): ");
  Serial.println(biasX);
  Serial.print("Bias Y (dps): ");
  Serial.println(biasY);
  Serial.print("Bias Z (dps): ");
  Serial.println(biasZ);
  
  // Initialize timer for integration
  previousTime = millis();
}

void loop() {
  // Calculate the elapsed time (in seconds)
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;  // Convert milliseconds to seconds
  
  // Read raw gyro data for each axis (using low byte first)
  int16_t rawX = read16(GYRO_ADDRESS, OUT_X_L_G);
  int16_t rawY = read16(GYRO_ADDRESS, OUT_Y_L_G);
  int16_t rawZ = read16(GYRO_ADDRESS, OUT_Z_L_G);
  
  // Convert raw values to degrees per second (dps) and remove bias
  float gyroX = (rawX * GYRO_SENSITIVITY) - biasX;
  float gyroY = (rawY * GYRO_SENSITIVITY) - biasY;
  float gyroZ = (rawZ * GYRO_SENSITIVITY) - biasZ;
  
  // Integrate angular velocity over time to update angles
  roll  += gyroX * dt;
  pitch += gyroY * dt;
  yaw   += gyroZ * dt;
  
  // Print the updated angles to the serial monitor
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Yaw: ");
  Serial.println(yaw);
  
  // Update previous time and wait a short interval before the next loop
  previousTime = currentTime;
  delay(10);
}

// Function to measure bias (average reading) over SAMPLE_COUNT samples
void measureBias() {
  long sumX = 0, sumY = 0, sumZ = 0;
  
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int16_t rawX = read16(GYRO_ADDRESS, OUT_X_L_G);
    int16_t rawY = read16(GYRO_ADDRESS, OUT_Y_L_G);
    int16_t rawZ = read16(GYRO_ADDRESS, OUT_Z_L_G);
    
    sumX += rawX;
    sumY += rawY;
    sumZ += rawZ;
    
    delay(10); // Delay between samples
  }
  
  // Calculate average bias for each axis and convert to dps
  biasX = (sumX / (float)SAMPLE_COUNT) * GYRO_SENSITIVITY;
  biasY = (sumY / (float)SAMPLE_COUNT) * GYRO_SENSITIVITY;
  biasZ = (sumZ / (float)SAMPLE_COUNT) * GYRO_SENSITIVITY;
  Serial.println("Bias measured from 12000 samples:");
  Serial.print("Bias X (dps): ");
  Serial.println(biasX);
  Serial.print("Bias Y (dps): ");
  Serial.println(biasY);
  Serial.print("Bias Z (dps): ");
  Serial.println(biasZ);
}

// Write a value to a specific register of a device
void writeRegister(uint8_t device, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Read a single byte from a register
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

// Read a 16-bit value from consecutive registers (low byte first)
int16_t read16(uint8_t device, uint8_t reg) {
  uint8_t low = 0, high = 0;
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.endTransmission();
  
  // Request 2 bytes from the sensor
  Wire.requestFrom(device, (uint8_t)2);
  if (Wire.available() >= 2) {
    low = Wire.read();
    high = Wire.read();
  }
  
  // Combine the two bytes (assuming little-endian order)
  return (int16_t)(high << 8 | low);
}



