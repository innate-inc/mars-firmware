#include "IMUControl.h"

// Static instance for SERCOM0 interrupt handling
IMUControl* IMUControl::_instance = nullptr;

IMUControl::IMUControl()
  : _customI2C(&sercom0, A4, A1), // A4 (SDA), A1 (SCL) using SERCOM0
    _currentHeading(0.0f), _headingOffset(0.0f), _gyroZ(0.0f),
    _newHeadingData(false), _initialized(false), _lastUpdateTime(0)
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
  if (!_initialized) return;
  
  
  unsigned long currentTime = millis();
  if (currentTime - _lastUpdateTime < UPDATE_INTERVAL_MS) {
    return; // Not time for update yet
  }
  _loopCalls++;
  
  _lastUpdateTime = currentTime;
  
  // Drain FIFO to get the freshest DMP data
  int drainCount = 0;
  ICM_20948_Status_e status;
  do {
    status = _imu.readDMPdataFromFIFO(&_dmpData);
    if (status == ICM_20948_Stat_Ok || status == ICM_20948_Stat_FIFOMoreDataAvail) {
      processQuaternionData();
      drainCount++;
    }
  } while (status == ICM_20948_Stat_FIFOMoreDataAvail);
  
  // Hz measurement: total frames from FIFO, polls with data, and loop speed
  static unsigned long hzWindowStart = 0;
  static int gyroFrames = 0;
  static int quatFrames = 0;
  static int pollsWithData = 0;
  if (hzWindowStart == 0) hzWindowStart = currentTime;
  gyroFrames += _gyroFrameCount;
  quatFrames += _quatFrameCount;
  _gyroFrameCount = 0;
  _quatFrameCount = 0;
  if (drainCount > 0) pollsWithData++;
  unsigned long elapsed = currentTime - hzWindowStart;
  if (elapsed >= 2000) {
    float gyroHz = gyroFrames * 1000.0f / elapsed;
    float quatHz = quatFrames * 1000.0f / elapsed;
    float loopHz = _loopCalls * 1000.0f / elapsed;
    Serial.print("Gyro Hz: ");
    Serial.print(gyroHz, 1);
    Serial.print("  Quat9 Hz: ");
    Serial.print(quatHz, 1);
    Serial.print("  Loop Hz: ");
    Serial.println(loopHz, 1);
    gyroFrames = 0;
    quatFrames = 0;
    pollsWithData = 0;
    _loopCalls = 0;
    hzWindowStart = currentTime;
  }
}

float IMUControl::getHeading() const {
  return _currentHeading;
}

float IMUControl::getGyroZ() const {
  return _gyroZ;
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

  // Initialize the DMP (default uses divider=19 → 55Hz)
  success &= (_imu.initializeDMP() == ICM_20948_Stat_Ok);

  // Override sample rate to 225Hz and fix gyro scale factor to match
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 9; // 1125/(1+8) = 112Hz
  mySmplrt.a = 17; // 1125/(1+17) = ~62Hz
  success &= (_imu.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt) == ICM_20948_Stat_Ok);
  success &= (_imu.setGyroSF(8, 3) == ICM_20948_Stat_Ok); // divider=8, 2000dps

  // Enable the DMP Orientation sensor (9-axis quaternion for heading)
  success &= (_imu.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Enable raw gyroscope for high-rate yaw rate
  success &= (_imu.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);

  // Gyro at full DMP rate (112Hz), Quat9 at half rate (~56Hz)
  success &= (_imu.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok);
  success &= (_imu.setDMPODRrate(DMP_ODR_Reg_Quat9, 1) == ICM_20948_Stat_Ok);

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
    _quatFrameCount++;
    
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

  // Process raw gyro data (arrives at full DMP rate)
  if ((_dmpData.header & DMP_header_bitmap_Gyro) > 0) {
    // 2000dps FSR: 1 LSB = 2000/32768 deg/s, convert to rad/s
    _gyroZ = _dmpData.Raw_Gyro.Data.Z * (2000.0f / 32768.0f) * (PI / 180.0f);
    _gyroFrameCount++;
  }
}


// SERCOM0 interrupt handler for custom I2C
void SERCOM0_Handler() {
  if (IMUControl::_instance) {
    IMUControl::_instance->_customI2C.onService();
  }
}