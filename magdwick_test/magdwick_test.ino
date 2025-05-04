#include <Wire.h>
#include <SparkFunLSM9DS1.h> // Use the SparkFun LSM9DS1 library
#include <MadgwickAHRS.h>   // MadgwickAHRS library

LSM9DS1 imu; // Create instance of the LSM9DS1 library
Madgwick filter; // Create instance of the Madgwick filter

// Gyro calibration variables
float gx_bias = 0.0, gy_bias = 0.0, gz_bias = 0.0;
const unsigned long calibration_time = 10000; // 10 seconds for calibration
unsigned long loop_micros; // Timer for filter update rate
const unsigned long loop_period_micros = 10000; // Target loop period for 100 Hz (1 / 100 * 1,000,000)

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection

  Wire.begin();

  // Initialize the IMU
  if (!imu.begin()) {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    while (1); // Halt execution
  }
  Serial.println("LSM9DS1 Initialized.");

  // Calibrate the gyroscope
  calibrateGyro();

  // loop_micros = micros(); // Initialize timer for filter update
}

void calibrateGyro() {
  Serial.println("Calibrating gyroscope... Keep the sensor stationary.");
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int count = 0;
  unsigned long start_time = millis();

  while (millis() - start_time < calibration_time) {
    if (imu.gyroAvailable()) {
      imu.readGyro();
      gx_sum += imu.gx;
      gy_sum += imu.gy;
      gz_sum += imu.gz;
      count++;
    }
  }

  if (count > 0) {
    gx_bias = (float)gx_sum / count;
    gy_bias = (float)gy_sum / count;
    gz_bias = (float)gz_sum / count;
  }

  Serial.println("Gyro calibration complete.");
  Serial.print("Bias Gx: "); Serial.println(imu.calcGyro(gx_bias), 4);
  Serial.print("Bias Gy: "); Serial.println(imu.calcGyro(gy_bias), 4);
  Serial.print("Bias Gz: "); Serial.println(imu.calcGyro(gz_bias), 4);
}

void loop() {
  unsigned long loop_start_micros = micros(); // Record loop start time

  // Check if new data is available
  if (imu.accelAvailable() && imu.gyroAvailable() && imu.magAvailable()) {
    // Read sensor data
    imu.readAccel();
    imu.readGyro();
    imu.readMag();

    // Apply gyro calibration
    float gx_cal = imu.gx - gx_bias;
    float gy_cal = imu.gy - gy_bias;
    float gz_cal = imu.gz - gz_bias;

    // Convert raw data to physical units (degrees/second for gyro, g's for accel)
    // Magnetometer units don't strictly matter for Madgwick if normalized internally,
    // but use calculated Gauss values for consistency.
    float ax = imu.calcAccel(imu.ax);
    float ay = imu.calcAccel(imu.ay);
    float az = imu.calcAccel(imu.az);
    float gx = imu.calcGyro(gx_cal);
    float gy = imu.calcGyro(gy_cal);
    float gz = imu.calcGyro(gz_cal);
    float mx = imu.calcMag(imu.mx);
    float my = imu.calcMag(imu.my);
    float mz = imu.calcMag(imu.mz);

    // Calculate delta t
    unsigned long current_micros = micros();
    float delta_t = (current_micros - loop_micros) / 1000000.0f; // Time difference in seconds
    loop_micros = current_micros;


    // Update the filter. Note the order: gx, gy, gz, ax, ay, az, mx, my, mz
    // Assumes filter uses frequency set in begin() to handle time delta internally.
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz); // 9 arguments
    // filter.updateIMU(gx, gy, gz, ax, ay, az); // Use this if no magnetometer

    // Get the orientation angles
    // float roll = filter.getRoll();
    // float pitch = filter.getPitch();
    float yaw = filter.getYaw(); // Yaw is typically the heading

    // Print the heading
    Serial.print("Heading (Yaw): ");
    Serial.println(yaw, 2); // Print with 2 decimal places

  } // End of sensor data available check

  // Enforce loop rate
  unsigned long loop_end_micros = micros();
  long delay_micros = loop_period_micros - (loop_end_micros - loop_start_micros);

  if (delay_micros > 0) {
    delayMicroseconds(delay_micros);
  }
}
