#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

// Create an instance of the sensor using the correct I2C addresses:
// 0x6A for the accel/gyro and 0x1C for the magnetometer.
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(0x6A, 0x1C);

void setup() {
  Serial.begin(115200);
  // Wait for the serial monitor to open (only needed for some boards)
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("LSM9DS1 Sensor Test");

  // Initialize the sensor
  if (!lsm.begin()) {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }

  // Optionally configure sensor ranges
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);       // Accelerometer range: +/-2g (other options available)
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);          // Magnetometer gain: 4 gauss (other options available)
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);       // Gyro scale: 245 degrees per second (other options available)

  Serial.println("LSM9DS1 initialized successfully!");
}

void loop() {
  // Create sensor event structures for each sensor
  sensors_event_t a, m, g, temp;
  
  // Read the sensor data
  lsm.getEvent(&a, &m, &g, &temp);

  // Print Accelerometer data
  Serial.print("Accel (m/s^2): X=");
  Serial.print(a.acceleration.x);
  Serial.print(" Y=");
  Serial.print(a.acceleration.y);
  Serial.print(" Z=");
  Serial.println(a.acceleration.z);

  // Print Magnetometer data
  Serial.print("Mag (uT): X=");
  Serial.print(m.magnetic.x);
  Serial.print(" Y=");
  Serial.print(m.magnetic.y);
  Serial.print(" Z=");
  Serial.println(m.magnetic.z);

  // Print Gyroscope data
  Serial.print("Gyro (dps): X=");
  Serial.print(g.gyro.x);
  Serial.print(" Y=");
  Serial.print(g.gyro.y);
  Serial.print(" Z=");
  Serial.println(g.gyro.z);

  // Print Temperature data
  Serial.print("Temp (°C): ");
  Serial.println(temp.temperature);

  Serial.println(""); // Blank line for readability

  delay(1000); // Wait 1 second before the next reading
}
