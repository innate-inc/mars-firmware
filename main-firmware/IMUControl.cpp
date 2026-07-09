#include "IMUControl.h"

// Static instance for SERCOM0 interrupt handling
IMUControl* IMUControl::_instance = nullptr;

IMUControl::IMUControl()
  : _customI2C(&sercom0, A4, A1), // A4 (SDA), A1 (SCL) using SERCOM0
    _currentHeading(0.0f), _headingOffset(0.0f),
    _newHeadingData(false), _initialized(false), _sleeping(false),
    _motionWatch(false), _haveMotionBaseline(false), _lastAccelMag(0.0f),
    _motionThresholdMg(0), _motionOverCount(0),
    _lastUpdateTime(0)
{
  // Register this instance for SERCOM0 interrupt handling
  _instance = this;
}

bool IMUControl::begin() {
  Serial.println("Initializing IMU Control...");
  
  // Initialize custom I2C first
  if (!initializeCustomI2C()) {
    Serial.println("Failed to initialize custom I2C");
    return false;
  }
  
  delay(100);
  
  // Initialize the ICM-20948
  if (!initializeIMU()) {
    Serial.println("Failed to initialize ICM-20948");
    return false;
  }
  
  delay(100);
  
  // Initialize DMP for quaternion output
  if (!initializeDMP()) {
    Serial.println("Failed to initialize DMP");
    return false;
  }
  
  _initialized = true;
  _lastUpdateTime = millis();
  
  Serial.println("IMU Control initialized successfully");
  return true;
}

void IMUControl::update() {
  if (!_initialized || _sleeping || _motionWatch) return;
  
  unsigned long currentTime = millis();
  if (currentTime - _lastUpdateTime < UPDATE_INTERVAL_MS) {
    return; // Not time for update yet
  }
  
  _lastUpdateTime = currentTime;
  
  // Read DMP data from FIFO
  ICM_20948_Status_e status = _imu.readDMPdataFromFIFO(&_dmpData);
  
  if (status == ICM_20948_Stat_Ok || status == ICM_20948_Stat_FIFOMoreDataAvail) {
    processQuaternionData();
  }
}

float IMUControl::getHeading() const {
  return _currentHeading;
}

bool IMUControl::headingAvailable() {
  // Always return true if IMU is connected and initialized
  // The heading data is continuously updated in the background
  return _initialized && _imu.isConnected();
}

void IMUControl::resetHeading() {
  // Set offset so that current raw heading becomes 0
  _headingOffset = -_rawHeading;  // Offset = -raw_value to make calibrated = 0
  _currentHeading = 0.0f;  // Force current heading to 0
  Serial.print("IMU heading reset - Raw: ");
  Serial.print(_rawHeading * 180.0 / PI, 2);
  Serial.print("°, Offset: ");
  Serial.print(_headingOffset * 180.0 / PI, 2);
  Serial.println("°");
}

void IMUControl::setSleep(bool enable) {
  if (!_initialized) return;

  if (enable) {
    if (_sleeping) return; // Already in hardware sleep
    // Hardware sleep supersedes motion-watch if we were in it. The DMP may
    // have been disabled by motion-watch; resumeActive() re-enables it.
    _imu.sleep(true);
    _sleeping = true;
    _motionWatch = false;
    Serial.println("IMU entering hardware sleep");
  } else {
    if (!_sleeping && !_motionWatch) return; // Already fully active
    resumeActive();
  }
}

void IMUControl::resumeActive() {
  _imu.sleep(false);       // No-op if not sleeping; clears the sleep bit
  delay(30);               // Let the sensor settle before touching the DMP
  _imu.enableDMP(true);    // Re-enable in case motion-watch disabled it

  // Discard anything queued while asleep/watching so the first heading
  // after resume is fresh
  _imu.resetDMP();
  _imu.resetFIFO();

  _newHeadingData = false;
  _sleeping = false;
  _motionWatch = false;
  _haveMotionBaseline = false;
  _motionOverCount = 0;
  _lastUpdateTime = millis();
  Serial.println("IMU resumed to active");
}

void IMUControl::beginMotionWatch(uint16_t threshold_mg) {
  if (!_initialized) return;

  _motionThresholdMg = (threshold_mg == 0) ? 120 : threshold_mg;

  if (_motionWatch) {
    // Already watching - just re-arm the baseline/debounce
    _haveMotionBaseline = false;
    _motionOverCount = 0;
    return;
  }

  // Coming from hardware sleep: wake the sensor front-end first
  if (_sleeping) {
    _imu.sleep(false);
    delay(30);
    _sleeping = false;
  }

  // Stop the DMP so the FIFO doesn't fill unattended; the accelerometer
  // front-end keeps running and getAGMT() reads it directly.
  _imu.enableDMP(false);

  _motionWatch = true;
  _haveMotionBaseline = false;
  _motionOverCount = 0;

  Serial.print("IMU motion-watch armed, threshold ");
  Serial.print(_motionThresholdMg);
  Serial.println(" mg");
}

bool IMUControl::pollMotion() {
  if (!_motionWatch) return false;

  // Refresh raw accel registers
  if (_imu.getAGMT() != ICM_20948_Stat_Ok) return false;

  float ax = _imu.accX();
  float ay = _imu.accY();
  float az = _imu.accZ();
  float mag = sqrtf(ax * ax + ay * ay + az * az); // mg; ~1000 at rest

  if (!_haveMotionBaseline) {
    _lastAccelMag = mag;
    _haveMotionBaseline = true;
    return false;
  }

  float delta = fabsf(mag - _lastAccelMag);
  _lastAccelMag = mag;

  if (delta > (float)_motionThresholdMg) {
    if (++_motionOverCount >= MOTION_CONFIRM_SAMPLES) {
      _motionOverCount = 0;
      return true;
    }
  } else {
    _motionOverCount = 0;
  }
  return false;
}

bool IMUControl::isConnected() const {
  // Cast away const to call isConnected() method
  return _initialized && const_cast<ICM_20948_I2C&>(_imu).isConnected();
}

void IMUControl::getAccelGyro(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  if (!_initialized) {
    ax = ay = az = gx = gy = gz = 0.0f;
    return;
  }
  
  // Get latest sensor data
  _imu.getAGMT();
  
  ax = _imu.accX();
  ay = _imu.accY(); 
  az = _imu.accZ();
  gx = _imu.gyrX();
  gy = _imu.gyrY();
  gz = _imu.gyrZ();
}

// Private methods

bool IMUControl::initializeCustomI2C() {
  // Initialize the custom I2C bus (exact copy from working example)
  _customI2C.begin();
  _customI2C.setClock(400000);
  
  // Assign the SERCOM functionality to the pins (exact copy from working example)
  // A4 (PA06) - SERCOM0 PAD2, use as SDA with alternate function
  // A1 (PA05) - SERCOM0 PAD1, use as SCL with alternate function  
  pinPeripheral(A4, PIO_SERCOM_ALT); // SDA on A4
  pinPeripheral(A1, PIO_SERCOM_ALT); // SCL on A1
  
  Serial.println("Custom I2C initialized on A4 (SDA) and A1 (SCL)");
  return true;
}

bool IMUControl::initializeIMU() {
  bool initialized = false;
  while (!initialized) {
    // Initialize the ICM-20948 using our custom I2C bus
    _imu.begin(_customI2C, 0); // AD0_VAL = 0

    Serial.print("Initialization of the sensor returned: ");
    Serial.println(_imu.statusString());
    if (_imu.status != ICM_20948_Stat_Ok) {
      Serial.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }
  
  Serial.println("ICM-20948 connected successfully");
  return true;
}

bool IMUControl::initializeDMP() {
  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP
  success &= (_imu.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP Orientation sensor (9-axis quaternion for heading)
  success &= (_imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Enable the FIFO 
  success &= (_imu.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (_imu.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (_imu.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (_imu.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success) {
    Serial.println("DMP enabled!");
  } else {
    Serial.println("Enable DMP failed!");
    Serial.println("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...");
    return false;
  }
  return true;
}

void IMUControl::processQuaternionData() {
  // Check if we have Orientation data (9-axis quaternion)
  if ((_dmpData.header & DMP_header_bitmap_Quat9) > 0) {
    // Extract quaternion components (Q30 format)
    double q1 = (double)_dmpData.Quat9.Data.Q1 / 1073741824.0;
    double q2 = (double)_dmpData.Quat9.Data.Q2 / 1073741824.0;
    double q3 = (double)_dmpData.Quat9.Data.Q3 / 1073741824.0;
    double q0 = sqrt(1.0 - (q1*q1 + q2*q2 + q3*q3));

    // Heading (yaw)
    double t0 = 2.0 * (q0*q3 + q1*q2);
    double t1 = 1.0 - 2.0 * (q2*q2 + q3*q3);
    double yaw_degrees = atan2(t0, t1) * 180.0 / PI;
    if (yaw_degrees < 0) yaw_degrees += 360.0;
    
    // Convert to radians (-180 to +180 range)
    double raw_yaw_radians = (yaw_degrees - 180.0) * PI / 180.0;
    
    // Store raw IMU value
    _rawHeading = raw_yaw_radians;
    
    // Apply offset to get calibrated heading
    _currentHeading = _rawHeading + _headingOffset;
    
    // Normalize to [-PI, PI]
    while (_currentHeading > PI) _currentHeading -= 2.0f * PI;
    while (_currentHeading < -PI) _currentHeading += 2.0f * PI;
    
    _newHeadingData = true;
    
    // Debug output
    static int debug_counter = 0;
    if (++debug_counter >= 50) {
      debug_counter = 0;
      Serial.print("Raw IMU: ");
      Serial.print(yaw_degrees, 2);
      Serial.print("° -> Calibrated: ");
      Serial.print(_currentHeading * 180.0 / PI, 2);
      Serial.print("° (offset: ");
      Serial.print(_headingOffset * 180.0 / PI, 2);
      Serial.println("°)");
    }
  }
}


// SERCOM0 interrupt handler for custom I2C
void SERCOM0_Handler() {
  if (IMUControl::_instance) {
    IMUControl::_instance->_customI2C.onService();
  }
}