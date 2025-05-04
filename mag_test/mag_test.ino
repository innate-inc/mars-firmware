#include <Wire.h>
#include <SparkFunLSM9DS1.h> // Use the SparkFun LSM9DS1 library
#include <math.h> // For sqrt()

LSM9DS1 imu; // Create instance of the LSM9DS1 library

// Define the I2C addresses if needed (usually defaults are fine)
#define LSM9DS1_M	0x1E // Mag address
#define LSM9DS1_AG	0x6B // Accel/Gyro address

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection

  Wire.begin();

  // Initialize the IMU, using default I2C addresses
  // *** DO NOT CALL imu.calibrateMag() HERE! ***
  // We want to use the offsets previously written to the registers.
  if (!imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire)) {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    while (1); // Halt execution
  }
  Serial.println("LSM9DS1 Initialized for Testing.");
  Serial.println();
  Serial.println("Ensure the sensor was previously calibrated using the");
  Serial.println("mag_calibration sketch and HAS NOT BEEN POWER CYCLED since.");
  Serial.println();
  Serial.println("Rotate the sensor slowly and observe the output.");
  Serial.println("Raw X/Y/Z should change, but Raw Magnitude should stay relatively constant.");
  Serial.println("--------------------------------------------------");
  Serial.println("Raw X\tRaw Y\tRaw Z\t| Raw Magnitude");
  Serial.println("--------------------------------------------------");

  // Optional: Set the magnetometer ODR to match your application setting if needed
  // imu.setMagODR(7); // Example: Set to 80 Hz
  // delay(100);
}

void loop() {
  // Check if new magnetometer data is available
  if (imu.magAvailable()) {
    // Read the magnetometer values.
    // imu.readMag() reads the sensor values, which should have the
    // hardware offset (written during calibration) automatically subtracted.
    imu.readMag();

    // Calculate the magnitude of the raw vector (after internal offset subtraction)
    // Note: imu.mx, imu.my, imu.mz are int16_t, cast to float for calculation
    float mx_f = (float)imu.mx;
    float my_f = (float)imu.my;
    float mz_f = (float)imu.mz;
    float magnitude = sqrt(mx_f * mx_f + my_f * my_f + mz_f * mz_f);

    // Print the raw values and the magnitude
    Serial.print(imu.mx); Serial.print("\t");
    Serial.print(imu.my); Serial.print("\t");
    Serial.print(imu.mz); Serial.print("\t| ");
    Serial.println(magnitude, 2); // Print magnitude with 2 decimal places

    /*
    // --- Optional: Print values in Gauss ---
    float mGx = imu.calcMag(imu.mx);
    float mGy = imu.calcMag(imu.my);
    float mGz = imu.calcMag(imu.mz);
    float magnitudeGauss = sqrt(mGx*mGx + mGy*mGy + mGz*mGz);

    Serial.print("Gauss X: "); Serial.print(mGx, 4);
    Serial.print("\t Gauss Y: "); Serial.print(mGy, 4);
    Serial.print("\t Gauss Z: "); Serial.print(mGz, 4);
    Serial.print("\t | Gauss Mag: "); Serial.println(magnitudeGauss, 4);
    */
  }

  delay(100); // Adjust delay as needed for readability
} 