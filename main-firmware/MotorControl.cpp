// src/MotorControl.cpp
#include "MotorControl.h"
#include "Utils.h"
#include "IMUControl.h"

// Static instance for SERCOM4 interrupt handling
MotorControl* MotorControl::_instance = nullptr;

MotorControl::MotorControl()
  : _serialLeft(nullptr), _serialRight(&Serial1),
    _motorId(1), 
    _lastLeftSpeed(0), _lastRightSpeed(0),
    _lastLeftRPM(0.0f), _lastRightRPM(0.0f),
    _newLeftRPM(false), _newRightRPM(false),
    _currentX(0.0f), _currentY(0.0f), _currentTheta(0.0f), _odomTheta(0.0f),
    _lastUpdateTime(0),
    _leftBufIndex(0), _rightBufIndex(0),
    _imuControl(nullptr), _useIMUHeading(false)
{
  // Register this instance for SERCOM4 interrupt handling
  _instance = this;

  // Create left motor UART on SERCOM4 with A3(RX) and A2(TX)
  _serialLeft = new Uart(&sercom4, PIN_A3, PIN_A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);
  
  // DON'T configure pins here - do it in begin() like the working code
}

void MotorControl::begin(uint32_t baud) {
  // Initialize left motor UART on SERCOM4
  _serialLeft->begin(baud);
  
  // Configure pins AFTER begin(), matching working code pattern
  pinPeripheral(PIN_A2, PIO_SERCOM_ALT);  // A2 as TX (SERCOM4 Pad 0)
  pinPeripheral(PIN_A3, PIO_SERCOM_ALT);  // A3 as RX (SERCOM4 Pad 1)
  
  delay(20);

  // Initialize right motor UART on dedicated Serial1
  _serialRight->begin(baud);
  delay(20);

  Serial.println("Motor UARTs initialized");

  // Initialize both motors with ID and mode
  setMotorID(*_serialLeft);
  setMotorID(*_serialRight);
  delay(100);
  
  setMotorMode(*_serialLeft);
  setMotorMode(*_serialRight);
  delay(20);

  // Initialize position tracking
  _currentX = 0.0f;
  _currentY = 0.0f;
  _currentTheta = 0.0f;
  _odomTheta = 0.0f;
  _lastUpdateTime = millis();

  Serial.println("Motors initialized");
}

void MotorControl::executeCommand(int16_t speed_cmd, int16_t turn_cmd) {
  
  // Convert Jetson commands to motor speeds using reference implementation logic
  // speed_cmd and turn_cmd are scaled by 100 (e.g., 100 = 1.0 m/s)
  
  float speed_multiplier = 1.0f;  // Reduce speed to 1/3
  float scaled_speed_cmd = speed_cmd * speed_multiplier;
  
  // Convert forward speed (scaled by 100) to mm/s
  float forwardSpeed = (scaled_speed_cmd / 100.0) * 1000.0;
  float wheelCircumference = WHEEL_DIAMETER * 3.14159;
  
  // Calculate the target RPM based on forward speed
  float targetRPM = (forwardSpeed / wheelCircumference) * 60.0;
  
  // Calculate the differential adjustment based on the turn rate (scaled by 100)
  float omega = turn_cmd / 100.0;
  float delta = (omega * TRACK_WIDTH * 60.0) / (2.0 * wheelCircumference);
  
  // For forward motion, both motors should run in the same direction
  float leftRPM = targetRPM + delta;
  float rightRPM = targetRPM - delta;
  
  // Convert the RPM values to the motor command scale (RPM * 10)
  int16_t left_speed = (int16_t)(leftRPM * 10);
  int16_t right_speed = (int16_t)(rightRPM * 10);
  
  // Clamp the commands to the allowed motor range
  float maxRatio = max(1, max(fabs(left_speed), fabs(right_speed)) / MAX_MOTOR_SPEED);
  left_speed = left_speed / maxRatio;
  right_speed = right_speed / maxRatio;
  
  // Send commands to motors
  sendMotorSpeeds(left_speed, -right_speed);
  
  // Update position estimate
  updatePosition(left_speed, right_speed);  // Use original speeds for position calculation
  
}

void MotorControl::sendMotorSpeeds(int16_t left_speed, int16_t right_speed) {
  _lastLeftSpeed = left_speed;
  _lastRightSpeed = right_speed;
  
  sendMotorCommand(*_serialLeft, left_speed);
  sendMotorCommand(*_serialRight, right_speed);
}

void MotorControl::processFeedback() {
  // Process feedback from left motor
  processMotorFeedback(_leftBuffer, _leftBufIndex, _lastLeftRPM, _newLeftRPM);
  
  // Process feedback from right motor  
  processMotorFeedback(_rightBuffer, _rightBufIndex, _lastRightRPM, _newRightRPM);
}

bool MotorControl::available() {
  bool available = _newLeftRPM || _newRightRPM;
  if (_newLeftRPM) _newLeftRPM = false;
  if (_newRightRPM) _newRightRPM = false;
  return available;
}

float MotorControl::getLeftRPM() const {
  return _lastLeftRPM;
}

float MotorControl::getRightRPM() const {
  return _lastRightRPM;
}

void MotorControl::updatePosition(int16_t left_speed, int16_t right_speed) {
  unsigned long current_time = millis();
  float dt = (current_time - _lastUpdateTime) / 1000.0f;
  
  if (_lastUpdateTime == 0) {
    _lastUpdateTime = current_time;
    return;
  }
  
  // Skip updates if dt is too small or too large (prevent wild jumps)
  if (dt < 0.001f || dt > 0.1f) {
    _lastUpdateTime = current_time;
    return;
  }
  
  float wheelCircumference = WHEEL_DIAMETER * 3.14159;
  
  // Use actual velocity reported back from the wheels (feedback RPM)
  float vFbLeft = (_lastLeftRPM / 60.0) * wheelCircumference;   // RPM to mm/s
  float vFbRight = (-_lastRightRPM / 60.0) * wheelCircumference; // negate (motor reversed)
  
  // Compute the average linear speed (forward/backward motion)
  float v = (vFbLeft + vFbRight) / 2.0;
  
  // Update odom theta from wheel differential
  float omega = (vFbRight - vFbLeft) / TRACK_WIDTH;
  _odomTheta += omega * dt;
  while (_odomTheta > PI) _odomTheta -= 2 * PI;
  while (_odomTheta < -PI) _odomTheta += 2 * PI;
  
  // Update the robot's estimated position using the current heading (theta)
  // Convert mm/s to m/s for position storage
  _currentX += (v / 1000.0) * cos(_currentTheta) * dt;
  _currentY += (v / 1000.0) * sin(_currentTheta) * dt;
  
  _lastUpdateTime = current_time;
  
  // Debug output (only print every 10th update to avoid spam)
  static int debug_counter = 0;
  if (++debug_counter >= 10) {
    debug_counter = 0;
    // Serial.print("Position update - vLeft: ");
    // Serial.print(vLeft);
    // Serial.print(", vRight: ");
    // Serial.print(vRight);
    // Serial.print(", v: ");
    // Serial.print(v);
    // Serial.print(", dt: ");
    // Serial.print(dt, 3);
    // Serial.print(", X: ");
    // Serial.print(_currentX, 3);
    // Serial.print(", Y: ");
    // Serial.print(_currentY, 3);
    // Serial.print(", Theta: ");
    // Serial.print(_currentTheta * 180.0 / PI, 1);
    // Serial.println("°");
  }
}

void MotorControl::getPosition(float &x, float &y, float &theta, float &odomTheta) {
  x = _currentX;
  y = _currentY;
  theta = _currentTheta;
  odomTheta = _odomTheta;
}

void MotorControl::enableIMUHeading(bool enable) {
  _useIMUHeading = enable && (_imuControl != nullptr);
  if (_useIMUHeading) {
    Serial.println("IMU heading integration enabled");
  } else {
    Serial.println("IMU heading integration disabled");
  }
}

void MotorControl::setIMUControl(IMUControl* imu) {
  _imuControl = imu;
}

void MotorControl::updateHeadingFromIMU(float heading) {
  // Update heading directly from IMU (like working code)
  _currentTheta = heading;
  
  // Keep theta in range [-pi, pi]
  while (_currentTheta > PI) _currentTheta -= 2 * PI;
  while (_currentTheta < -PI) _currentTheta += 2 * PI;
}

void MotorControl::resetPosition() {
  _currentX = 0.0f;
  _currentY = 0.0f;
  _currentTheta = 0.0f;
  _odomTheta = 0.0f;
  _lastUpdateTime = millis();
  Serial.println("Position tracking reset");
}

void MotorControl::forceResetHeading() {
  _currentTheta = 0.0f;
  Serial.println("Heading force reset to 0°");
}

// Private methods

uint8_t MotorControl::calculateCRC(const uint8_t* data, int len) {
  return calculateCRC8Maxim(data, len);  // Use Utils function
}

void MotorControl::setMotorID(Stream &motorSerial) {
  uint8_t cmd[10] = {0xAA, 0x55, 0x53, _motorId, 0,0,0,0,0, 0};
  cmd[9] = calculateCRC(cmd, 9);
  for (int i = 0; i < 5; i++) {
    motorSerial.write(cmd, 10);
    delay(20);
  }
}

void MotorControl::setMotorMode(Stream &motorSerial) {
  uint8_t cmd[10] = {_motorId, 0xA0, 0x02, 0,0,0,0,0,0, 0};
  cmd[9] = calculateCRC(cmd, 9);
  motorSerial.write(cmd, 10);
  delay(20);
}

void MotorControl::sendMotorCommand(Stream &motorSerial, int16_t speed) {
  uint8_t cmd[10] = {_motorId, 0x64, 0,0, 0x01, 0x00, 0x01, 0,0, 0};
  //                                      fb1=speed       accel
  splitInt16(speed, cmd[2], cmd[3]);
  cmd[9] = calculateCRC(cmd, 9);
  motorSerial.write(cmd, 10);
}

void MotorControl::processMotorFeedback(uint8_t* buffer, size_t& bufIndex, float& lastRPM, bool& newRPM) {
  Stream* serial = (buffer == _leftBuffer) ? _serialLeft : _serialRight;
  
  while (serial->available()) {
    uint8_t b = serial->read();
    buffer[bufIndex++] = b;

    if (bufIndex >= 10) {
      if (calculateCRC(buffer, 9) == buffer[9]) {
        int16_t sp = combineInt16(buffer[2], buffer[3]);
        lastRPM = sp / 10.0f;
        newRPM = true;
      }
      // Slide buffer
      memmove(buffer, buffer + 1, --bufIndex);
    }
    
    // Prevent buffer overflow
    if (bufIndex >= BUFFER_SIZE - 1) {
      bufIndex = 0;
    }
  }
}

// Simple interrupt handlers matching working code pattern
void SERCOM4_0_Handler() { 
  if (MotorControl::_instance && MotorControl::_instance->_serialLeft) 
    MotorControl::_instance->_serialLeft->IrqHandler(); 
}
void SERCOM4_1_Handler() { 
  if (MotorControl::_instance && MotorControl::_instance->_serialLeft) 
    MotorControl::_instance->_serialLeft->IrqHandler(); 
}
void SERCOM4_2_Handler() { 
  if (MotorControl::_instance && MotorControl::_instance->_serialLeft) 
    MotorControl::_instance->_serialLeft->IrqHandler(); 
}
void SERCOM4_3_Handler() { 
  if (MotorControl::_instance && MotorControl::_instance->_serialLeft) 
    MotorControl::_instance->_serialLeft->IrqHandler(); 
}