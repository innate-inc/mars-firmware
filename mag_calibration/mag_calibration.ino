#include <Wire.h>
#include <SparkFunLSM9DS1.h> // Use the SparkFun LSM9DS1 library

LSM9DS1 imu; // Create instance of the LSM9DS1 library

// Define the I2C addresses if needed (usually defaults are fine)
#define LSM9DS1_M	0x1E // Mag address
#define LSM9DS1_AG	0x6B // Accel/Gyro address

// --- Magnetometer Output Data Rate (ODR) Settings ---
// Choose a lower rate for calibration
// 0 = 0.625 Hz  4 = 10 Hz
// 1 = 1.25 Hz   5 = 20 Hz
// 2 = 2.5 Hz    6 = 40 Hz
// 3 = 5 Hz      7 = 80 Hz (Default)
const uint8_t CALIBRATION_MAG_ODR = 4; // Use 10 Hz for calibration (takes ~13 seconds)
// Set this to the ODR you want for your main application
const uint8_t APPLICATION_MAG_ODR = 7; // Restore to 80 Hz after calibration

// Helper function to estimate Hz from Mag ODR setting
float getMagRateHz(uint8_t magRate)
{
  switch (magRate) {
      case 0: return 0.625;
      case 1: return 1.25;
      case 2: return 2.5;
      case 3: return 5.0;
      case 4: return 10.0;
      case 5: return 20.0;
      case 6: return 40.0;
      case 7: return 80.0;
      default: return 0.0; // Should not happen
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection

  Wire.begin();

  // Initialize the IMU, using default I2C addresses
  // Make sure begin() is called before setMagODR
  if (!imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire)) {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    while (1); // Halt execution
  }
  Serial.println("LSM9DS1 Initialized.");
  Serial.println();

  // --- Slow down Magnetometer for Calibration ---
  Serial.print("Setting magnetometer ODR to ");
  Serial.print(getMagRateHz(CALIBRATION_MAG_ODR));
  Serial.println(" Hz for calibration.");
  imu.setMagODR(CALIBRATION_MAG_ODR);
  delay(100); // Short delay to allow sensor to adjust

  // Start Magnetometer Calibration
  Serial.println("Starting Magnetometer Calibration...");
  Serial.println("Slowly rotate the sensor in all directions (figure-eight motion works well).");
  Serial.print("Keep rotating until calibration completes (approx ");
  Serial.print(128.0f / getMagRateHz(CALIBRATION_MAG_ODR), 1); // Estimate time
  Serial.println(" seconds)...");
  Serial.println();

  // Calibrate the magnetometer.
  // The 'true' argument automatically calculates biases and loads
  // them into the magnetometer's OFFSET_X/Y/Z_REG_M registers.
  imu.calibrateMag(true);

  Serial.println();
  Serial.println("Magnetometer Calibration Complete!");
  Serial.println("Offsets have been written to the sensor's registers.");
  Serial.println();

  // --- Restore Magnetometer ODR for Application ---
  Serial.print("Restoring magnetometer ODR to ");
  Serial.print(getMagRateHz(APPLICATION_MAG_ODR));
  Serial.println(" Hz.");
  imu.setMagODR(APPLICATION_MAG_ODR);
  delay(100);

  // Print the raw bias values that were calculated and loaded.
  // These are stored in the imu.mBiasRaw array by the calibrateMag function.
  Serial.println("Calculated Raw Offsets (written to registers):");
  Serial.print("X: "); Serial.println(imu.mBiasRaw[0]);
  Serial.print("Y: "); Serial.println(imu.mBiasRaw[1]);
  Serial.print("Z: "); Serial.println(imu.mBiasRaw[2]);
  Serial.println();
  Serial.println("Calibration finished. You can now use the sensor.");
  Serial.println("The sensor will automatically apply these offsets internally.");

  // Optional: Print the biases in Gauss for reference
  // Serial.println("Calculated Biases (Gauss):");
  // Serial.print("X: "); Serial.println(imu.mBias[0], 4);
  // Serial.print("Y: "); Serial.println(imu.mBias[1], 4);
  // Serial.print("Z: "); Serial.println(imu.mBias[2], 4);
}


void loop() {
  // Nothing to do in the loop for calibration.
  // The offsets are now stored in the sensor's hardware registers.
  delay(100);
} 