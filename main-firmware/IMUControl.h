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
   * Put the ICM-20948 into hardware sleep (enable=true), or resume it to
   * full active/DMP operation (enable=false).
   *
   * setSleep(false) is the single "resume" entry point: it brings the
   * sensor back regardless of whether it was in hardware sleep or in
   * motion-watch, re-enabling the DMP and resetting the DMP/FIFO so no
   * stale samples are consumed. update() is a no-op while asleep.
   */
  void setSleep(bool enable);

  /**
   * Enter motion-watch: keep the accelerometer running (DMP disabled) so
   * pollMotion() can detect a physical disturbance. Used instead of
   * hardware sleep when the caller wants a physical wake path. Idempotent -
   * calling it again while already watching just re-arms the baseline.
   *
   * @param threshold_mg Per-sample |accel| change (in mg) that counts as
   *                     motion.
   * @return true if motion-watch is armed; false if the IMU never
   *         initialized (caller must not advertise a motion wake path).
   */
  bool beginMotionWatch(uint16_t threshold_mg);

  /**
   * Sample the accelerometer once and report whether motion is confirmed.
   * Returns false unless in motion-watch. A disturbance must exceed the
   * threshold on two consecutive samples to latch, which rejects isolated
   * single-sample glitches. Call at the sleep-loop rate.
   */
  bool pollMotion();

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
  bool _sleeping;         // In hardware sleep

  // Motion-watch (physical wake) state
  bool _motionWatch;              // Accelerometer sampled for disturbance
  bool _haveMotionBaseline;       // A previous sample exists to diff against
  float _lastAccelMag;            // Previous |accel| magnitude (mg)
  uint16_t _motionThresholdMg;    // Per-sample delta that counts as motion
  uint8_t _motionOverCount;       // Consecutive over-threshold samples
  static const uint8_t MOTION_CONFIRM_SAMPLES = 2; // Debounce count to latch

  // Resume the sensor to active/DMP from either sleep sub-mode
  void resumeActive();
  
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