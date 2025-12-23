#ifndef IMUCONTROL_H
#define IMUCONTROL_H

#include <Arduino.h>
#include <Wire.h>
#include "wiring_private.h"
#include "ICM_20948.h"

// Forward declaration for interrupt handler
void SERCOM0_Handler();

class IMUControl {
public:
  /**
   * Constructor for ICM-20948 IMU control
   * Uses custom I2C on A1 (SCL) and A4 (SDA) via SERCOM0
   */
  IMUControl();

  /**
   * Initialize IMU, I2C bus, and DMP functionality
   */
  bool begin();

  /**
   * Update IMU readings and process DMP data
   * Should be called regularly in main loop
   */
  void update();

  /**
   * Get current heading in radians
   * Returns theta value for position tracking
   */
  float getHeading() const;

  /**
   * Check if new heading data is available
   */
  bool headingAvailable();

  /**
   * Reset heading to zero (calibration)
   */
  void resetHeading();

  /**
   * Get IMU status for diagnostics
   */
  bool isConnected() const;

  /**
   * Get raw accelerometer and gyro data for debugging
   */
  void getAccelGyro(float &ax, float &ay, float &az, float &gx, float &gy, float &gz);

private:
  // Custom I2C for ICM-20948
  TwoWire _customI2C;
  
  // ICM-20948 object
  ICM_20948_I2C _imu;
  
  // State variables
  float _currentHeading;
  float _rawHeading;      // Raw IMU heading (no offset applied)
  float _headingOffset;   // Offset to make current heading = 0 at calibration
  bool _newHeadingData;
  bool _initialized;
  
  // DMP data structure
  icm_20948_DMP_data_t _dmpData;
  
  // Timing
  unsigned long _lastUpdateTime;
  static const unsigned long UPDATE_INTERVAL_MS = 10; // 100Hz update rate
  
  // Helper methods
  bool initializeCustomI2C();
  bool initializeIMU();
  bool initializeDMP();
  void processQuaternionData();
  
  // Static instance for SERCOM0 interrupt
  static IMUControl* _instance;
  
  // Friend declaration for interrupt handler
  friend void SERCOM0_Handler();
};

#endif // IMUCONTROL_H