// src/MotorControl.h
#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include "wiring_private.h"    // for pinPeripheral()

// Forward declaration to avoid circular dependency
class IMUControl;

class MotorControl {
public:
  /**
   * Constructor for dual motor control matching I2C implementation
   * Left Motor: SERCOM4 with A3(RX) and A2(TX)
   * Right Motor: Hardware Serial1
   */
  MotorControl();

  /**
   * Initialize UARTs, pins, and set motor ID & speed-loop mode.
   */
  void begin(uint32_t baud = 115200);

  /** Send differential drive command with speed and turn values */
  void executeCommand(int16_t speed_cmd, int16_t turn_cmd);

  /** Send direct motor speeds to left and right motors */
  void sendMotorSpeeds(int16_t left_speed, int16_t right_speed);

  /** Process incoming bytes to update RPM feedback from both motors */
  void processFeedback();

  /** True if fresh RPM readings are available */
  bool available();

  /** Get the most recent RPM readings */
  float getLeftRPM() const;
  float getRightRPM() const;

  /** Get latest encoder deltas (mm, sign-corrected, snapshot - not cleared) */
  void getEncoderPositions(uint16_t &leftPos, uint16_t &rightPos) const;

  /** Update position estimation based on motor speeds */
  void updatePosition(int16_t left_speed, int16_t right_speed);

  /** Get current estimated position (3 odom methods, shared theta) */
  void getPosition(float &x1, float &y1, float &x2, float &y2, float &x3, float &y3, float &theta);

  /** Enable/disable IMU heading integration */
  void enableIMUHeading(bool enable = true);
  
  /** Set IMU control instance */
  void setIMUControl(IMUControl* imu);
  
  /** Update heading directly from IMU (for continuous updates) */
  void updateHeadingFromIMU(float heading);
  
  /** Reset position tracking (for calibration) */
  void resetPosition();
  
  /** Force reset heading to 0 (for calibration) */
  void forceResetHeading();

private:
  // Motor communication
  Uart* _serialLeft;        // SERCOM4 for left motor
  HardwareSerial* _serialRight; // Serial1 for right motor
  
  uint8_t _motorId;         // Motor ID (both motors use same ID)
  
  // Motor speeds and feedback
  int16_t _lastLeftSpeed;
  int16_t _lastRightSpeed;
  float _lastLeftRPM;
  float _lastRightRPM;
  bool _newLeftRPM;
  bool _newRightRPM;
  
  // Position tracking - 3 odometry methods with shared heading
  // Method 1: commanded speed dead reckoning
  float _odomX1;
  float _odomY1;
  // Method 2: encoder position feedback
  float _odomX2;
  float _odomY2;
  // Method 3: commanded speed with averaged heading
  float _odomX3;
  float _odomY3;
  float _currentTheta;
  float _prevTheta;
  unsigned long _lastUpdateTime;
  
  struct EncoderState {
    uint16_t lastPos = 0;
    bool initialized = false;
    float deltaMM = 0.0f;
  };

  // Encoder position tracking for Method 2
  EncoderState _leftEncoder;
  EncoderState _rightEncoder;
  
  // IMU integration
  IMUControl* _imuControl;
  bool _useIMUHeading;
  
  // Communication buffers
  static const size_t BUFFER_SIZE = 64;
  uint8_t _leftBuffer[BUFFER_SIZE];
  uint8_t _rightBuffer[BUFFER_SIZE];
  size_t _leftBufIndex;
  size_t _rightBufIndex;
  
  // Constants matching reference implementation
  static const int16_t MAX_MOTOR_SPEED = 2100;
  static const int WHEEL_DIAMETER = 69;      // Wheel diameter in mm
  static const int TRACK_WIDTH = 189;       // Distance between motors in mm
  
  // Static instance for SERCOM4 interrupt handling
  static MotorControl* _instance;
  
  // Private methods
  uint8_t calculateCRC(const uint8_t* data, int len);
  void setMotorID(Stream &motorSerial);
  void setMotorMode(Stream &motorSerial);
  void sendMotorCommand(Stream &motorSerial, int16_t speed);
  void processMotorFeedback(Stream& serial, uint8_t* buffer, size_t& bufIndex, float& lastRPM, bool& newRPM, EncoderState& encoder, float sign);
  
  // Friend declarations for interrupt handlers
  friend void SERCOM4_0_Handler();
  friend void SERCOM4_1_Handler();
  friend void SERCOM4_2_Handler();
  friend void SERCOM4_3_Handler();
};

// C linkage for interrupt handlers (only SERCOM4 needed)
extern "C" {
  void SERCOM4_0_Handler();
  void SERCOM4_1_Handler();
  void SERCOM4_2_Handler();
  void SERCOM4_3_Handler();
}

#endif // MOTORCONTROL_H