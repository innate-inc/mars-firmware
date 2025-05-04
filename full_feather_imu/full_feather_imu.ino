#include <Arduino.h>
#include "wiring_private.h"    // Needed for pinPeripheral()
#include <Adafruit_NeoPixel.h>
#include <Wire.h>              // Added for I2C communication
#include <SparkFunLSM9DS1.h>   // Added for IMU sensor
#include "MahonyAHRS.h"        // Added for Mahony filter

// ---------------------------------------------------------------------
// Constants and Definitions
// ---------------------------------------------------------------------
#define MOTOR_ID        0x01           // Both motors set to ID 1
#define WHEEL_DIAMETER  69           // Wheel diameter in mm
#define MAX_SPEED       2100           // Motor command range: -2100 to 2100
#define TRACK_WIDTH     189          // Distance between the two motors in mm

// IMU Definitions
#define LSM9DS1_AG	    0x6B // Accel/Gyro I2C address
#define IMU_UPDATE_RATE 100  // Update rate in Hz for IMU and Mahony filter

const uint8_t SOM[2] = {0x69, 0x69};   // Start-of-Message marker

// LED definitions
#define LED_PIN     2       // Pin for NeoPixel strip
#define NUM_LEDS    39       // Number of LEDs

// Motor feedback buffer size
#define MOTOR_BUFFER_SIZE 64

// ---------------------------------------------------------------------
// Global Variables
// ---------------------------------------------------------------------

// Robot state (position and heading)
float posX = 0.0, posY = 0.0, theta = 0.0; // Position (in mm) and heading (in radians)
int16_t latestForwardSpeed = 0;           // Latest forward speed command (scaled by 100)
int16_t latestTurnRate = 0;               // Latest turn rate command (scaled by 100)

// LED state variables
uint8_t ledMode = 0;      // 0 = Off, 1 = Solid, 2 = Blink, 3 = Ring
uint16_t ledTime = 300;   // Time parameter in ms
uint8_t ledRed = 0, ledGreen = 0, ledBlue = 0;

// LED animation timing
unsigned long ledPreviousMillis = 0;
bool blinkState = false;
int currentLEDIndex = 0;

// Integration timing
unsigned long lastIntegrationTime = 0;

// IMU and Filter Timing
unsigned long lastImuUpdateTime = 0; // Added for IMU update timing

// Motor feedback buffers and state
uint8_t motorFeedbackBufferLeft[MOTOR_BUFFER_SIZE];
uint8_t motorFeedbackIndexLeft = 0;
float leftFeedbackRpm = 0.0;
bool leftFeedbackReady = false;

uint8_t motorFeedbackBufferRight[MOTOR_BUFFER_SIZE];
uint8_t motorFeedbackIndexRight = 0;
float rightFeedbackRpm = 0.0;
bool rightFeedbackReady = false;

// ---------------------------------------------------------------------
// Serial1 (host) message parser variables
// ---------------------------------------------------------------------
enum Serial1ParseState { WAIT_FOR_SOM1, WAIT_FOR_SOM2, READ_PAYLOAD };
Serial1ParseState serial1State = WAIT_FOR_SOM1;
uint8_t serial1PayloadBuffer[8];  // Holds the 8-byte payload (7 data + 1 CRC)
int serial1PayloadIndex = 0;

// IMU and Mahony Filter Objects
LSM9DS1 imu;           // Added IMU object
Mahony mahony;       // Added Mahony filter object

// ---------------------------------------------------------------------
// Motor UART Setup (as per your sample code)
// ---------------------------------------------------------------------

// Left Motor: Using SERCOM4 with TX on PIN_A2 (Pad 0) and RX on PIN_A3 (Pad 1)
Uart SerialLeft(&sercom4, PIN_A3, PIN_A2, SERCOM_RX_PAD_1, UART_TX_PAD_0);
// Right Motor: Using SERCOM0 with TX on PIN_A4 (Pad 0) and RX on PIN_A1 (Pad 1)
Uart SerialRight(&sercom0, PIN_A1, PIN_A4, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Required SERCOM interrupt handlers for SerialLeft (SERCOM4)
void SERCOM4_0_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_1_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_2_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_3_Handler() { SerialLeft.IrqHandler(); }

// Required SERCOM interrupt handlers for SerialRight (SERCOM0)
void SERCOM0_0_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_1_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_2_Handler() { SerialRight.IrqHandler(); }
void SERCOM0_3_Handler() { SerialRight.IrqHandler(); }

// ---------------------------------------------------------------------
// LED Setup
// ---------------------------------------------------------------------
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------------------------------------------------------
// Function Prototypes
// ---------------------------------------------------------------------
uint8_t calculate_crc8_maxim(uint8_t *data, int len);
void setMotorID(Uart &motorSerial);
void setMotorMode(Uart &motorSerial);
void sendMotorCommand(Uart &motorSerial, int16_t speed);
void processLeftFeedback();
void processRightFeedback();
void processMotorFeedback();
void computeMotorCommands(int16_t forward, int16_t turn, int16_t *leftCmd, int16_t *rightCmd);
void integratePosition();
void updateLEDs();
void resetLEDAnimation();
void updateLEDStripImmediate();
void processSerial1Messages();
void processSerial1Payload(uint8_t *payload);
void sendMovementResponse();
void sendLEDResponse();
void sendStatusResponse();
void initializeIMU();    // Added prototype
void updateHeadingFromIMU(); // Added prototype

// ---------------------------------------------------------------------
// CRC-8/MAXIM Calculation Function (Polynomial 0x8C, initial value 0x00)
// ---------------------------------------------------------------------
uint8_t calculate_crc8_maxim(uint8_t *data, int len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0x8C;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// ---------------------------------------------------------------------
// Motor Initialization Functions
// ---------------------------------------------------------------------
void setMotorID(Uart &motorSerial) {
  uint8_t cmd[10] = {0};
  cmd[0] = 0xAA;
  cmd[1] = 0x55;
  cmd[2] = 0x53;
  cmd[3] = MOTOR_ID;  // Set ID to 1
  cmd[4] = 0x00;
  cmd[5] = 0x00;
  cmd[6] = 0x00;
  cmd[7] = 0x00;
  cmd[8] = 0x00;
  cmd[9] = calculate_crc8_maxim(cmd, 9);
  for (int i = 0; i < 5; i++) {
    motorSerial.write(cmd, 10);
    delay(20);
  }
}

void setMotorMode(Uart &motorSerial) {
  uint8_t cmd[10] = {0};
  cmd[0] = MOTOR_ID;
  cmd[1] = 0xA0;
  cmd[2] = 0x02;  // Speed loop mode
  cmd[3] = 0x00;
  cmd[4] = 0x00;
  cmd[5] = 0x00;
  cmd[6] = 0x00;
  cmd[7] = 0x00;
  cmd[8] = 0x00;
  cmd[9] = calculate_crc8_maxim(cmd, 9);
  motorSerial.write(cmd, 10);
  delay(20);
}

// ---------------------------------------------------------------------
// Send Motor Command (Drive Command)
// ---------------------------------------------------------------------
void sendMotorCommand(Uart &motorSerial, int16_t speed) {
  uint8_t cmd[10] = {0};
  cmd[0] = MOTOR_ID;
  cmd[1] = 0x64;  // Drive command
  cmd[2] = (speed >> 8) & 0xFF;
  cmd[3] = speed & 0xFF;
  cmd[4] = 0x00;
  cmd[5] = 0x00;
  cmd[6] = 0x01;  // Acceleration time = 1ms per 1rpm
  cmd[7] = 0x00;  // No braking (0xFF for brake)
  cmd[8] = 0x00;
  cmd[9] = calculate_crc8_maxim(cmd, 9);
  motorSerial.write(cmd, 10);
}

// ---------------------------------------------------------------------
// Process Motor Feedback for Left Motor
// ---------------------------------------------------------------------
void processLeftFeedback() {
  while (SerialLeft.available() > 0) {
    uint8_t byteRead = SerialLeft.read();
    if (motorFeedbackIndexLeft < MOTOR_BUFFER_SIZE)
      motorFeedbackBufferLeft[motorFeedbackIndexLeft++] = byteRead;
    else {
      // If buffer full, shift left to make room.
      for (uint8_t i = 1; i < MOTOR_BUFFER_SIZE; i++)
        motorFeedbackBufferLeft[i - 1] = motorFeedbackBufferLeft[i];
      motorFeedbackBufferLeft[MOTOR_BUFFER_SIZE - 1] = byteRead;
    }
    
    // Process complete 10-byte messages
    while (motorFeedbackIndexLeft >= 10) {
      if (calculate_crc8_maxim(motorFeedbackBufferLeft, 9) == motorFeedbackBufferLeft[9]) {
        int16_t speedFeedback = ((int16_t)motorFeedbackBufferLeft[2] << 8) | motorFeedbackBufferLeft[3];
        leftFeedbackRpm = speedFeedback / 10.0;  // Scale: motor feedback is multiplied by 10
        leftFeedbackReady = true;
        // Remove processed 10 bytes from buffer
        for (uint8_t i = 10; i < motorFeedbackIndexLeft; i++)
          motorFeedbackBufferLeft[i - 10] = motorFeedbackBufferLeft[i];
        motorFeedbackIndexLeft -= 10;
      } else {
        // If CRC fails, discard one byte and try to re-sync.
        for (uint8_t i = 1; i < motorFeedbackIndexLeft; i++)
          motorFeedbackBufferLeft[i - 1] = motorFeedbackBufferLeft[i];
        motorFeedbackIndexLeft--;
      }
    }
  }
}

// ---------------------------------------------------------------------
// Process Motor Feedback for Right Motor
// ---------------------------------------------------------------------
void processRightFeedback() {
  while (SerialRight.available() > 0) {
    uint8_t byteRead = SerialRight.read();
    if (motorFeedbackIndexRight < MOTOR_BUFFER_SIZE)
      motorFeedbackBufferRight[motorFeedbackIndexRight++] = byteRead;
    else {
      for (uint8_t i = 1; i < MOTOR_BUFFER_SIZE; i++)
        motorFeedbackBufferRight[i - 1] = motorFeedbackBufferRight[i];
      motorFeedbackBufferRight[MOTOR_BUFFER_SIZE - 1] = byteRead;
    }
    
    while (motorFeedbackIndexRight >= 10) {
      if (calculate_crc8_maxim(motorFeedbackBufferRight, 9) == motorFeedbackBufferRight[9]) {
        int16_t speedFeedback = ((int16_t)motorFeedbackBufferRight[2] << 8) | motorFeedbackBufferRight[3];
        rightFeedbackRpm = speedFeedback / 10.0;
        rightFeedbackReady = true;
        for (uint8_t i = 10; i < motorFeedbackIndexRight; i++)
          motorFeedbackBufferRight[i - 10] = motorFeedbackBufferRight[i];
        motorFeedbackIndexRight -= 10;
      } else {
        for (uint8_t i = 1; i < motorFeedbackIndexRight; i++)
          motorFeedbackBufferRight[i - 1] = motorFeedbackBufferRight[i];
        motorFeedbackIndexRight--;
      }
    }
  }
}

// ---------------------------------------------------------------------
// Process Motor Feedback (Both Motors)
// ---------------------------------------------------------------------
void processMotorFeedback() {
  processLeftFeedback();
  processRightFeedback();
  // (For integration, we simply use the latest RPM values.)
}

// ---------------------------------------------------------------------
// Compute Motor Commands from the Latest Velocity Command
// ---------------------------------------------------------------------
void computeMotorCommands(int16_t forward, int16_t turn, int16_t *leftCmd, int16_t *rightCmd) {
  // Convert forward speed (scaled by 100) to mm/s
  float forwardSpeed = (forward / 100.0) * 1000.0;
  float wheelCircumference = WHEEL_DIAMETER * 3.14159;
  
  // Calculate the target RPM based on forward speed
  float targetRPM = (forwardSpeed / wheelCircumference) * 60.0;
  
  // Calculate the differential adjustment based on the turn rate (scaled by 100)
  float omega = turn / 100.0;
  float delta = (omega * TRACK_WIDTH * 60.0) / (2.0 * wheelCircumference);
  
  // **Note the inversion for the left motor:**
  // For forward motion, left motor must run with a negative RPM.
  float leftRPM = -(targetRPM - delta);
  float rightRPM = targetRPM + delta;
  
  // Convert the RPM values to the motor command scale (RPM * 10)
  *leftCmd = (int16_t)(leftRPM * 10);
  *rightCmd = (int16_t)(rightRPM * 10);
  
  // Clamp the commands to the allowed motor range
  if (*leftCmd > MAX_SPEED) *leftCmd = MAX_SPEED;
  if (*leftCmd < -MAX_SPEED) *leftCmd = -MAX_SPEED;
  if (*rightCmd > MAX_SPEED) *rightCmd = MAX_SPEED;
  if (*rightCmd < -MAX_SPEED) *rightCmd = -MAX_SPEED;
}


// ---------------------------------------------------------------------
// Integrate Robot Position Using Motor Feedback and IMU Heading
// ---------------------------------------------------------------------
void integratePosition() {
  unsigned long currentTime = millis();
  if (lastIntegrationTime == 0) {
    lastIntegrationTime = currentTime;
    return;
  }
  float dt = (currentTime - lastIntegrationTime) / 1000.0; // Time step in seconds
  lastIntegrationTime = currentTime;
  
  float wheelCircumference = WHEEL_DIAMETER * 3.14159;
  
  // Apply deadzone of 0.1 RPM to filter out noise
  float leftRpmFiltered = (abs(leftFeedbackRpm) < 0.15) ? 0.0 : leftFeedbackRpm;
  float rightRpmFiltered = (abs(rightFeedbackRpm) < 0.15) ? 0.0 : rightFeedbackRpm;
  
  // For position integration, reverse the left motor feedback to get its effective speed
  float vLeft = (-leftRpmFiltered / 60.0) * wheelCircumference;
  float vRight = (rightRpmFiltered / 60.0) * wheelCircumference;
  
  // Compute the average linear speed
  float v = (vLeft + vRight) / 2.0;

  // --- Heading (theta) is now updated separately by updateHeadingFromIMU() ---
  // float omega = (-vRight + vLeft) / TRACK_WIDTH; // Removed calculation based on wheels

  // Update the robot's estimated position using the IMU heading (theta)
  posX += v * cos(theta) * dt;
  posY += v * sin(theta) * dt;
  // theta += omega * dt; // Removed update based on wheels
}


// ---------------------------------------------------------------------
// LED Animation Functions
// ---------------------------------------------------------------------
void resetLEDAnimation() {
  ledPreviousMillis = millis();
  blinkState = false;
  currentLEDIndex = 0;
}

void updateLEDStripImmediate() {
  if (ledMode == 0) { // Off
    strip.clear();
    strip.show();
  } else if (ledMode == 1) { // Solid
    for (int i = 0; i < NUM_LEDS; i++) {
      strip.setPixelColor(i, strip.Color(ledRed, ledGreen, ledBlue));
    }
    strip.show();
  }
}

void updateLEDs() {
  unsigned long currentMillis = millis();
  
  switch (ledMode) {
    case 0: // Off – no animation
      break;
    case 1: // Solid – static color already set
      break;
    case 2: // Blink – toggle on/off every ledTime ms
      if (currentMillis - ledPreviousMillis >= ledTime) {
        ledPreviousMillis = currentMillis;
        blinkState = !blinkState;
        if (blinkState) {
          for (int i = 0; i < NUM_LEDS; i++) {
            strip.setPixelColor(i, strip.Color(ledRed, ledGreen, ledBlue));
          }
        } else {
          strip.clear();
        }
        strip.show();
      }
      break;
    case 3: // Ring – one LED on at a time, moving around the strip
      if (currentMillis - ledPreviousMillis >= ledTime) {
        ledPreviousMillis = currentMillis;
        currentLEDIndex = (currentLEDIndex + 1) % NUM_LEDS;
        strip.clear();
        strip.setPixelColor(currentLEDIndex, strip.Color(ledRed, ledGreen, ledBlue));
        strip.show();
      }
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------
// Serial1 (Host) Message Processing
// ---------------------------------------------------------------------
void processSerial1Messages() {
  while (Serial1.available() > 0) {
    uint8_t byteRead = Serial1.read();
    switch (serial1State) {
      case WAIT_FOR_SOM1:
        if (byteRead == 0x69)
          serial1State = WAIT_FOR_SOM2;
        break;
      case WAIT_FOR_SOM2:
        if (byteRead == 0x69) {
          serial1State = READ_PAYLOAD;
          serial1PayloadIndex = 0;
        } else {
          serial1State = WAIT_FOR_SOM1;
        }
        break;
      case READ_PAYLOAD:
        serial1PayloadBuffer[serial1PayloadIndex++] = byteRead;
        if (serial1PayloadIndex == 8) {
          // Check CRC over first 7 bytes of payload
          uint8_t crcCalc = calculate_crc8_maxim(serial1PayloadBuffer, 7);
          if (crcCalc == serial1PayloadBuffer[7]) {
            // Process valid payload
            processSerial1Payload(serial1PayloadBuffer);
          } else {
            // CRC error; optionally print debug info
            // Serial.println("Serial1 message CRC error.");
          }
          serial1State = WAIT_FOR_SOM1;
        }
        break;
    }
  }
}

void processSerial1Payload(uint8_t *payload) {
  uint8_t cmdID = payload[0];
  if (cmdID == 0x01) { // Movement Command
    // Bytes 1-2: forward speed, Bytes 3-4: turn rate
    int16_t forwardSpeed = ((int16_t)payload[1] << 8) | payload[2];
    int16_t turnRate = ((int16_t)payload[3] << 8) | payload[4];
    latestForwardSpeed = forwardSpeed;
    latestTurnRate = turnRate;
    
    // Compute and send motor commands immediately
    int16_t leftCmd, rightCmd;
    computeMotorCommands(latestForwardSpeed, latestTurnRate, &leftCmd, &rightCmd);
    sendMotorCommand(SerialLeft, leftCmd);
    sendMotorCommand(SerialRight, rightCmd);
    
    // Send back a Movement Response with the current position
    //Serial.println("Sending Movement Response");
    sendMovementResponse();
    
  } else if (cmdID == 0x02) { // LED Command
    // Byte 1: LED mode, Bytes 2-3: time parameter, Byte 4: Red, Byte 5: Green, Byte 6: Blue
    ledMode = payload[1];
    ledTime = ((uint16_t)payload[2] << 8) | payload[3];
    ledRed = payload[4];
    ledGreen = payload[5];
    ledBlue = payload[6];
    resetLEDAnimation();
    
    // For Off or Solid modes, update the strip immediately.
    if (ledMode == 0 || ledMode == 1)
      updateLEDStripImmediate();
      
    // Send LED Response
    sendLEDResponse();
    
  } else if (cmdID == 0x03) { // Status Request Command
    // For now, send a status response with zero values.
    sendStatusResponse();
  }
}

// ---------------------------------------------------------------------
// Send Movement Response (Response ID 0x81)
// ---------------------------------------------------------------------
void sendMovementResponse() {
  uint8_t response[10];
  response[0] = 0x69;
  response[1] = 0x69;
  response[2] = 0x81;  // Movement Response ID
  
  // Pack x, y, theta into 6 bytes.
  // For simplicity, posX and posY (in mm) are cast to int16_t.
  // Theta is scaled by 100 (i.e. in hundredths of a radian).
  int16_t x_fixed = (int16_t) posX;
  int16_t y_fixed = (int16_t) posY;
  int16_t theta_fixed = (int16_t)(theta * 100);
  
  response[3] = (x_fixed >> 8) & 0xFF;
  response[4] = x_fixed & 0xFF;
  response[5] = (y_fixed >> 8) & 0xFF;
  response[6] = y_fixed & 0xFF;
  response[7] = (theta_fixed >> 8) & 0xFF;
  response[8] = theta_fixed & 0xFF;
  
  // Compute CRC over payload bytes 2-8 (7 bytes)
  response[9] = calculate_crc8_maxim(response + 2, 7);
  //Serial.println("Sending Movement Response");
  Serial1.write(response, 10);
}

// ---------------------------------------------------------------------
// Send LED Response (Response ID 0x82)
// ---------------------------------------------------------------------
void sendLEDResponse() {
  uint8_t response[10];
  response[0] = 0x69;
  response[1] = 0x69;
  response[2] = 0x82;  // LED Response ID
  
  response[3] = ledMode;
  response[4] = ledRed;
  response[5] = ledGreen;
  response[6] = ledBlue;
  response[7] = (ledTime >> 8) & 0xFF;
  response[8] = ledTime & 0xFF;
  
  response[9] = calculate_crc8_maxim(response + 2, 7);
  Serial1.write(response, 10);
}

// ---------------------------------------------------------------------
// Send Status Response (Response ID 0x83)
// ---------------------------------------------------------------------
void sendStatusResponse() {
  uint8_t response[10];
  response[0] = 0x69;
  response[1] = 0x69;
  response[2] = 0x83;  // Status Response ID
  
  // Battery Voltage (2 bytes), Motor Temperature (2 bytes), Fault Code (1 byte), Reserved (1 byte)
  uint16_t batteryVoltage = 0;  // Not implemented
  uint16_t motorTemp = 0;       // Not implemented
  uint8_t faultCode = 0;
  
  response[3] = (batteryVoltage >> 8) & 0xFF;
  response[4] = batteryVoltage & 0xFF;
  response[5] = (motorTemp >> 8) & 0xFF;
  response[6] = motorTemp & 0xFF;
  response[7] = faultCode;
  response[8] = 0x00; // Reserved
  
  response[9] = calculate_crc8_maxim(response + 2, 7);
  Serial1.write(response, 10);
}

// ---------------------------------------------------------------------
// Initialize IMU Function (New Function)
// ---------------------------------------------------------------------
void initializeIMU() {
  Serial.println("Initializing IMU...");
  Wire.begin(); // Initialize I2C

  // Initialize the LSM9DS1 for Accel/Gyro only
  if (imu.begin(LSM9DS1_AG) == false) {
    Serial.println("!!! Failed to communicate with LSM9DS1 Accel/Gyro.");
    Serial.println("Check wiring and I2C address!");
    // Optional: hang here or set an error flag
    while (1);
  }
  Serial.println("LSM9DS1 Accel/Gyro initialized.");

  // Perform Accel/Gyro calibration
  Serial.println("Calibrating accelerometer and gyroscope...");
  Serial.println("Keep the sensor perfectly still.");
  
  imu.calibrate(); // Calibrates both accel and gyro
  Serial.println("Accel/Gyro calibration complete.");

  // Initialize the Mahony filter
  mahony.begin(IMU_UPDATE_RATE); // Set the sample frequency
  Serial.println("Mahony filter initialized.");
}

// ---------------------------------------------------------------------
// Update Heading (theta) from IMU and Mahony Filter (New Function)
// ---------------------------------------------------------------------
void updateHeadingFromIMU() {
  unsigned long currentMillis = millis();
  // Check if it's time to update based on IMU_UPDATE_RATE
  if (currentMillis - lastImuUpdateTime >= (1000 / IMU_UPDATE_RATE)) {

    // Check if new accel/gyro data is available
    if (imu.gyroAvailable() && imu.accelAvailable()) {
      lastImuUpdateTime = currentMillis; // Update time only if data is processed

      // Read sensor data
      imu.readGyro();
      imu.readAccel();

      // Update the filter with the fresh sensor data (6-DOF)
      // Apply the same transformation as the example: negate gx and gy
      mahony.updateIMU(-imu.calcGyro(imu.gx), -imu.calcGyro(imu.gy), imu.calcGyro(imu.gz),
                       imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az));

      // Get Yaw angle in degrees from the filter
      float yawDegrees = mahony.getYaw();

      // Convert yaw from degrees to radians
      float yawRadians = yawDegrees * DEG_TO_RAD; // DEG_TO_RAD = PI / 180.0

      // Apply the requested PI offset
      theta = yawRadians - PI;

      // Optional: Normalize theta to be within -PI to PI or 0 to 2*PI if needed
      // while (theta > PI) theta -= 2.0 * PI;
      // while (theta <= -PI) theta += 2.0 * PI;

    }
    // Consider if lastImuUpdateTime should be updated even if data wasn't ready
  }
}

// ---------------------------------------------------------------------
// Setup Function
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);     // For debugging
  //while (!Serial);          // Wait for Serial monitor
  Serial.println("Initializing Robot Controller...");
  
  // Initialize Serial1 for host communications
  Serial1.begin(115200);
  delay(200);
  
  // Initialize motor UARTs
  SerialLeft.begin(115200);
  delay(200);
  pinPeripheral(PIN_A2, PIO_SERCOM_ALT);  // Set A2 as TX for SERCOM4
  pinPeripheral(PIN_A3, PIO_SERCOM_ALT);  // Set A3 as RX for SERCOM4
  
  SerialRight.begin(115200);
  delay(200);
  pinPeripheral(PIN_A4, PIO_SERCOM_ALT);  // Set A4 as TX for SERCOM0
  pinPeripheral(PIN_A1, PIO_SERCOM_ALT);  // Set A1 as RX for SERCOM0
  
  Serial.println("Serial1, SerialLeft, and SerialRight initialized.");
  delay(200);
  // Initialize the LED strip
  strip.begin();
  strip.clear();
  strip.show();
  delay(10000);
  // Initialize IMU
  initializeIMU(); // Added IMU initialization call
  
  // Motor initialization: set motor IDs and mode.
  setMotorID(SerialLeft);
  setMotorID(SerialRight);
  delay(100);
  setMotorMode(SerialLeft);
  setMotorMode(SerialRight);
  Serial.println("Motors initialized to ID 1 and speed loop mode.");
  
  // Initialize robot state variables
  posX = 0.0;
  posY = 0.0;
  theta = 0.0; // Initial heading set to 0, will be updated by IMU
  latestForwardSpeed = 0;
  latestTurnRate = 0;
  
  // Initialize LED state variables
  ledMode = 0;
  ledTime = 300;
  ledRed = 0;
  ledGreen = 0;
  ledBlue = 0;
  resetLEDAnimation();
  
  lastIntegrationTime = millis();
  lastImuUpdateTime = millis(); // Initialize IMU update timer
}

// ---------------------------------------------------------------------
// Main Loop Function
// ---------------------------------------------------------------------
void loop() {
  // Process all incoming host commands on Serial1
  processSerial1Messages();
  
  // Update heading from IMU before using it for integration or sending commands
  updateHeadingFromIMU(); // Added IMU heading update call

  // Print the current heading for debugging in degrees
  float thetaDegrees = theta * RAD_TO_DEG; // Convert radians to degrees (RAD_TO_DEG = 180.0 / PI)
  Serial.print("Current Heading (degrees): ");
  Serial.println(thetaDegrees, 2); // Print degrees with 2 decimal places

  // Re-send motor commands based on the latest velocity command (to maintain motion)
  int16_t leftCmd, rightCmd;
  computeMotorCommands(latestForwardSpeed, latestTurnRate, &leftCmd, &rightCmd);
  sendMotorCommand(SerialLeft, leftCmd);
  sendMotorCommand(SerialRight, rightCmd);
  
  // Process motor feedback from both motors
  processMotorFeedback();
  
  // Integrate the motor feedback and IMU heading to update the robot's position
  integratePosition();
  
  // Update LED animations (for Blink and Ring modes)
  updateLEDs();
  
  delay(1); // Reduced delay slightly, IMU update handles its own timing
}
