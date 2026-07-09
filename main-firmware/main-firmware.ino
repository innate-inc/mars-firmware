/*
 * I2C Motor Controller Follower - Adafruit ItsyBitsy M4 Express
 * Receives motor commands via I2C from Jetson Orin Nano
 * Uses modular MotorControl, IMUControl, and Utils classes
 * Provides bidirectional communication with position feedback and status
 */

#include <Wire.h>
#include <Arduino.h>
#include "MotorControl.h"
#include "IMUControl.h"
#include "Utils.h"

// ---------------------------------------------------------------------
// I2C Configuration
#define I2C_FOLLOWER_ADDRESS 0x42  // Must match MCU_ADDR in Jetson code

// Command definitions (Jetson → MCU)
#define CMD_MOVE 0x01
#define CMD_STATUS 0x03
#define CMD_CALIBRATE 0x04
#define CMD_RESET 0x05
#define CMD_POWER 0x06

// Response definitions (MCU → Jetson)
#define RESP_MOVE 0x81      // Position feedback
#define RESP_STATUS 0x83    // Health data
#define RESP_CALIBRATE 0x84 // Calibration status
#define RESP_RESET 0x85     // Component Reset
#define RESP_POWER 0x86     // Power mode status

// ---------------------------------------------------------------------
// Battery monitoring constants (matching reference implementation)
#define BATTERY_PIN A5        // Analog pin for battery voltage
#define BUZZER_PIN 4          // Digital pin for low voltage buzzer
#define HSSW_EN_PIN 12        // High Side Switch Enable Pin
#define EFUSE_EN_PIN 10
#define HSSW_IS_PIN A0
#define LOW_VOLTAGE_THRESHOLD 10.5  // Voltage threshold for buzzer warning
#define LOW_VOLTAGE_CUTOFF 10.3     // Voltage threshold for power cutoff
#define HYSTERESIS_THRESHOLD 11.0   // Voltage threshold for hysteresis to eliminate repeated threshold crossing
#define VREF 3.3              // ADC reference voltage (3.3V)
#define R_TOP 13.3            // Top resistor in kΩ (matching reference)
#define R_BOTTOM 4.7          // Bottom resistor in kΩ (matching reference)
#define ADC_MAX_VALUE 4095.0  // 12-bit ADC resolution (ItsyBitsy M4)

#define RESET_NONE 0x00
#define RESET_HSSW 0x01
#define RESET_EFUSE 0x02
#define RESET_BOTH 0x03

// Power modes (CMD_POWER data byte 1)
// EFUSE_EN is never gated during sleep: it powers the main computer,
// and cutting it would hard-restart the whole system.
#define POWER_MODE_ACTIVE 0x00
#define POWER_MODE_SLEEP 0x01

// Sleep option flags (CMD_POWER data byte 2)
// The HSSW rail powers the arm+head Dynamixels and the wheel motor
// drivers. Gating is opt-in from the Jetson because cutting the rail
// kills the Dynamixel bus (no boop-to-wake while gated) and the Jetson
// must sequence its arm node around the power loss.
#define SLEEP_FLAG_GATE_HSSW 0x01

// ---------------------------------------------------------------------
// Global objects
MotorControl motorControl;
IMUControl imuControl;

// System status variables
float battery_voltage = 12.0; // Actual battery voltage from A5
uint8_t fault_code = 0;       // No faults initially
uint8_t calibration_status = 0; // 0=Success, 1=In Progress, 2=Failure
bool calibration_in_progress = false; // Flag to prevent IMU updates during calibration
bool boot_latch = false; //Flag to ensure HSSW EN pin is high
bool hysteresis_triggered = false; // Flag for hysteresis
bool cutoff_triggered = false;  // Flag for cutoff specifically

// Low voltage buzzer variables
unsigned long buzzer_start_time = 0;
uint8_t buzz_count = 0;
bool buzzer_active = false;
bool in_cycle_pause = false;
enum BuzzerState { BUZZER_OFF, BUZZER_ON, BUZZER_PAUSE, CYCLE_PAUSE };
BuzzerState buzzer_state = BUZZER_OFF;

// I2C communication variables
uint8_t i2c_buffer[8];
bool command_ready = false;
uint8_t response_buffer[8];
bool response_ready = false;
int16_t latestSpeedCmd = 0;
int16_t latestTurnCmd = 0;

// Reset Variables
uint8_t reset_target = RESET_NONE;
unsigned long reset_start_time = 0;
bool reset_in_progress = false;

// Power management variables
uint8_t power_mode = POWER_MODE_ACTIVE;
uint8_t sleep_flags = 0x00;
bool wake_motors_reinited = false; // Diagnostic: motors re-initialized on last wake
#define SLEEP_LOOP_DELAY_MS 20     // Loop period while asleep (vs 1ms active)

// True when sleep mode is intentionally holding the HSSW rail off.
// Every place that re-asserts HSSW_EN must consult this, otherwise the
// battery-recovery path turns the rail back on within one loop pass.
bool hsswGatedBySleep() {
  return power_mode == POWER_MODE_SLEEP && (sleep_flags & SLEEP_FLAG_GATE_HSSW);
}

// ---------------------------------------------------------------------
// Low voltage buzzer management
void updateBuzzer() {
  // Check recovery condition first (hysteresis)
  if (battery_voltage >= HYSTERESIS_THRESHOLD) {
    hysteresis_triggered = false;
    cutoff_triggered = false;
    // Voltage recovered - turn off buzzer and reset state
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(HSSW_EN_PIN, hsswGatedBySleep() ? LOW : HIGH);
    digitalWrite(EFUSE_EN_PIN, HIGH);
    buzzer_state = BUZZER_OFF;
    buzz_count = 0;
    buzzer_active = false;
    in_cycle_pause = false;
    return;
  }
  
  // Latch the triggered states
  if (battery_voltage < LOW_VOLTAGE_THRESHOLD) {
    hysteresis_triggered = true;
  }
  if (battery_voltage < LOW_VOLTAGE_CUTOFF) {
    cutoff_triggered = true;
  }
  
  // If hysteresis not triggered, everything is OK
  if (!hysteresis_triggered) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzer_state = BUZZER_OFF;
    buzz_count = 0;
    buzzer_active = false;
    in_cycle_pause = false;
    return;
  }
  
  // Critical cutoff triggered - constant beep and power off
  if (cutoff_triggered) {
    digitalWrite(HSSW_EN_PIN, LOW);
    digitalWrite(EFUSE_EN_PIN, LOW);
    digitalWrite(BUZZER_PIN, HIGH); // assert buzzer on constantly when below cutoff
    return;  // Stay in cutoff state until voltage recovers above hysteresis
  }
  
  // Low voltage warning mode - manage buzzer pattern
  unsigned long current_time = millis();
  
  switch (buzzer_state) {
    case BUZZER_OFF:
      // Start buzzer cycle
      digitalWrite(BUZZER_PIN, HIGH);
      buzzer_start_time = current_time;
      buzzer_state = BUZZER_ON;
      buzz_count = 1;
      Serial.println("LOW VOLTAGE WARNING - Buzzer ON");
      break;
      
    case BUZZER_ON:
      // Check if time elapsed
      if (current_time - buzzer_start_time >= 1000) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzer_start_time = current_time;
        buzzer_state = BUZZER_PAUSE;
        Serial.println("Buzzer OFF");
      }
      break;
      
    case BUZZER_PAUSE:
      // Check if pause elapsed
      if (current_time - buzzer_start_time >= 1000) {
        if (buzz_count < 3) {
          // Continue with next buzz
          digitalWrite(BUZZER_PIN, HIGH);
          buzzer_start_time = current_time;
          buzzer_state = BUZZER_ON;
          buzz_count++;
          Serial.println("Buzzer ON");
        } else {
          // Start 10 second cycle pause
          buzzer_start_time = current_time;
          buzzer_state = CYCLE_PAUSE;
          Serial.println("Cycle pause - 10 seconds");
        }
      }
      break;
      
    case CYCLE_PAUSE:
      // Check if pause elapsed
      if (current_time - buzzer_start_time >= 10000) {
        // Reset and start new cycle
        buzz_count = 0;
        buzzer_state = BUZZER_OFF;
        Serial.println("Starting new buzzer cycle");
      }
      break;
  }
}

void updateReset() {
  if (!reset_in_progress) return;
  
  // Check if 1 second has elapsed
  if (millis() - reset_start_time >= 1000) {
    // Restore the enable pins
    if (reset_target & RESET_HSSW) {
      digitalWrite(HSSW_EN_PIN, hsswGatedBySleep() ? LOW : HIGH);
      Serial.println("HSSW restored");
    }
    if (reset_target & RESET_EFUSE) {
      digitalWrite(EFUSE_EN_PIN, HIGH);
      Serial.println("EFUSE restored");
    }

    // Send completion response only for HSSW-only reset
    // (EFUSE and BOTH cause full system restart, so no response needed)
    if (reset_target == RESET_HSSW) {
      response_buffer[0] = RESP_RESET;
      response_buffer[1] = 0x02;        // Status: 0x02 = Completed
      response_buffer[2] = RESET_HSSW;  // Target that was reset
      memset(&response_buffer[3], 0x00, 4); // Clear reserved bytes
      response_buffer[7] = calculateCRC8Maxim(response_buffer, 7);
      response_ready = true;
      Serial.println("HSSW reset completion response prepared");
    }
    
    reset_in_progress = false;
    reset_target = RESET_NONE;
    Serial.println("Reset complete");
  }
}

// ---------------------------------------------------------------------
// Sleep / low-power mode
//
// Sleep quiesces everything the MCU drives while keeping the wake surface
// alive: I2C stays fully responsive, battery monitoring and the low-voltage
// buzzer/cutoff keep running. Motors get one final zero-speed command and
// then no UART traffic at all; the ICM-20948 is put into its hardware sleep
// state; the main loop drops from ~1kHz to ~50Hz. If the Jetson sets
// SLEEP_FLAG_GATE_HSSW, the high-side-switched peripheral rail is held off
// for the duration of sleep.

void enterSleep(uint8_t flags) {
  bool already_asleep = (power_mode == POWER_MODE_SLEEP);
  bool was_gated = hsswGatedBySleep();

  if (!already_asleep) {
    Serial.print("Entering sleep mode, flags: 0x");
    Serial.println(flags, HEX);

    // Stop the base and stop commanding the motor drivers. Send the zero
    // command a few times since the motor UART has no ack.
    latestSpeedCmd = 0;
    latestTurnCmd = 0;
    for (int i = 0; i < 3; i++) {
      motorControl.sendMotorSpeeds(0, 0);
      delay(5);
    }

    // Hardware sleep for the IMU (config is retained, DMP resumes on wake)
    imuControl.setSleep(true);
  }

  power_mode = POWER_MODE_SLEEP;
  sleep_flags = flags;

  // Apply (or update) rail gating. Never touch EFUSE here - it powers the
  // main computer.
  if (hsswGatedBySleep()) {
    digitalWrite(HSSW_EN_PIN, LOW);
    if (!was_gated) Serial.println("HSSW rail gated off for sleep");
  } else if (was_gated && !cutoff_triggered) {
    // Flags changed from gated to ungated while staying asleep
    digitalWrite(HSSW_EN_PIN, HIGH);
    Serial.println("HSSW rail restored (sleep continues)");
  }
}

void exitSleep() {
  if (power_mode == POWER_MODE_ACTIVE) return;

  Serial.println("Waking from sleep mode");
  bool rail_was_gated = hsswGatedBySleep();
  power_mode = POWER_MODE_ACTIVE;
  sleep_flags = 0x00;
  wake_motors_reinited = false;

  if (!cutoff_triggered) {
    digitalWrite(HSSW_EN_PIN, HIGH);
    if (rail_was_gated) {
      // The wheel motor drivers are on the gated rail, so they were
      // power cycled and lost their ID/mode configuration. Give the
      // rail time to come up, then reconfigure them.
      delay(200);
      motorControl.reinit();
      wake_motors_reinited = true;
      Serial.println("HSSW rail restored, motors re-initialized");
    }
  }

  // Drop any stale bytes that arrived while we weren't reading feedback
  motorControl.flushFeedback();

  imuControl.setSleep(false);
  Serial.println("Wake complete");
}

void preparePowerResponse() {
  response_buffer[0] = RESP_POWER;
  response_buffer[1] = power_mode;
  response_buffer[2] = sleep_flags;
  response_buffer[3] = wake_motors_reinited ? 0x01 : 0x00;
  response_buffer[4] = 0x00; // Reserved
  response_buffer[5] = 0x00; // Reserved
  response_buffer[6] = 0x00; // Reserved
  response_buffer[7] = calculateCRC8Maxim(response_buffer, 7);

  response_ready = true;
}

float readBatteryVoltage() {
  // Read analog value from A5
  int raw = analogRead(BATTERY_PIN);
  
  // Voltage at the divider pin:
  float pinV = raw * (VREF / ADC_MAX_VALUE);
  
  // Correct divider factor = (R_TOP + R_BOTTOM) / R_BOTTOM
  float dividerFactor = (R_TOP + R_BOTTOM) / R_BOTTOM;
  float sourceV = pinV * dividerFactor;

  return sourceV;
}

// ---------------------------------------------------------------------
// Prepare response functions
void preparePositionResponse() {
  // Get current position from motor control
  float x, y, theta;
  motorControl.getPosition(x, y, theta);
  
  // Convert position to fixed-point format for transmission (matching Jetson expectations)
  // Position is in meters, convert to cm for transmission (Jetson divides by 100)
  int16_t x_cm = (int16_t)(x * 100.0);  // Convert to cm
  int16_t y_cm = (int16_t)(y * 100.0);  // Convert to cm
  int16_t theta_scaled = (int16_t)(theta * 100.0); // Scale by 100 (hundredths of radian)
  
  // Debug output
  Serial.print("Position - X: ");
  Serial.print(x_cm);
  Serial.print("cm, Y: ");
  Serial.print(y_cm);
  Serial.print("cm, Theta: ");
  Serial.print(theta * 180.0 / PI, 1);
  Serial.println("°");
  
  response_buffer[0] = RESP_MOVE;
  splitInt16(x_cm, response_buffer[1], response_buffer[2]);
  splitInt16(y_cm, response_buffer[3], response_buffer[4]);
  splitInt16(theta_scaled, response_buffer[5], response_buffer[6]);
  response_buffer[7] = calculateCRC8Maxim(response_buffer, 7);
  
  response_ready = true;
}

void prepareStatusResponse() {
  // Read actual battery voltage from pin
  battery_voltage = readBatteryVoltage();
  
  uint16_t battery_scaled = scaleBatteryVoltage(battery_voltage);
  
  response_buffer[0] = RESP_STATUS;
  response_buffer[1] = (battery_scaled >> 8) & 0xFF;
  response_buffer[2] = battery_scaled & 0xFF;
  response_buffer[3] = 0x00; // Reserved (was motor temp)
  response_buffer[4] = 0x00; // Reserved (was motor temp)
  response_buffer[5] = fault_code;
  response_buffer[6] = power_mode; // Current power mode (was reserved)
  response_buffer[7] = calculateCRC8Maxim(response_buffer, 7);

  response_ready = true;
}

void prepareCalibrateResponse() {
  response_buffer[0] = RESP_CALIBRATE;
  response_buffer[1] = calibration_status;
  response_buffer[2] = 0x00; // Reserved
  response_buffer[3] = 0x00; // Reserved
  response_buffer[4] = 0x00; // Reserved
  response_buffer[5] = 0x00; // Reserved
  response_buffer[6] = 0x00; // Reserved
  response_buffer[7] = calculateCRC8Maxim(response_buffer, 7);
  
  response_ready = true;
}

// ---------------------------------------------------------------------
// I2C Event Handlers

// Function that executes when data is received from leader
void receiveEvent(int numBytes) {
  if (numBytes == 9) {  // 8-byte command + 1 register byte
    // Read register byte (should be 0x00)
    uint8_t register_byte = Wire.read();
    
    // Read the 8-byte command
    for (int i = 0; i < 8 && Wire.available(); i++) {
      i2c_buffer[i] = Wire.read();
    }
    
    // Clear any remaining bytes
    while (Wire.available()) {
      Wire.read();
    }
    
    command_ready = true;
    
    Serial.print("Command received: ");
    for (int i = 0; i < 8; i++) {
      Serial.print("0x");
      Serial.print(i2c_buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else if (numBytes == 1) {
    // This is a read request - just consume the byte
    uint8_t read_register = Wire.read();
    Serial.print("Read request for register: 0x");
    Serial.println(read_register, HEX);
  } else {
    // Clear buffer if wrong number of bytes
    while (Wire.available()) {
      Wire.read();
    }
    Serial.println("Invalid command length");
  }
}

// Function that executes when data is requested by master
void requestEvent() {
  if (response_ready) {
    Wire.write(response_buffer, 8);
    response_ready = false;
    
    Serial.print("Response sent: ");
    for (int i = 0; i < 8; i++) {
      Serial.print("0x");
      Serial.print(response_buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    // Send empty response if nothing ready
    uint8_t empty_response[8] = {0};
    Wire.write(empty_response, 8);
    Serial.println("Empty response sent");
  }
}

// ---------------------------------------------------------------------
// Process received I2C command
void processCommand() {
  if (!command_ready) return;
  
  if (!boot_latch) {
    boot_latch = true;
    pinMode(HSSW_EN_PIN, OUTPUT);
    digitalWrite(HSSW_EN_PIN, HIGH);
  }
  
  // Verify CRC
  uint8_t received_crc = i2c_buffer[7];
  uint8_t calculated_crc = calculateCRC8Maxim(i2c_buffer, 7);

  if (received_crc != calculated_crc) {
    Serial.println("CRC mismatch - ignoring command");
    command_ready = false;
    return;
  }
  
  // Parse command
  uint8_t cmd_id = i2c_buffer[0];
  
  if (cmd_id == CMD_MOVE) {
    if (power_mode == POWER_MODE_SLEEP) {
      // The Jetson streams MOVE at 30Hz even while the robot sleeps (its
      // I2C loop stays up for battery monitoring). Keep the motors quiet
      // and just answer with the last known position; waking is only done
      // explicitly via CMD_POWER.
      preparePositionResponse();
      command_ready = false;
      return;
    }

    // Extract speed and turn values using utility function
    int16_t speed = combineInt16(i2c_buffer[1], i2c_buffer[2]);
    int16_t turn = combineInt16(i2c_buffer[3], i2c_buffer[4]);

    latestSpeedCmd = speed;
    latestTurnCmd = turn;

    Serial.print("Move command - Speed: ");
    Serial.print(speed);
    Serial.print(", Turn: ");
    Serial.println(turn);

    // Execute motor command through MotorControl class
    motorControl.executeCommand(speed, turn);

    // Prepare position response
    preparePositionResponse();

  } else if (cmd_id == CMD_STATUS) {
    Serial.println("Status request received");
    prepareStatusResponse();
    
  } else if (cmd_id == CMD_CALIBRATE) {
    if (power_mode == POWER_MODE_SLEEP) {
      // IMU is in hardware sleep - calibration cannot run. Wake first.
      Serial.println("Calibration refused - robot is in sleep mode");
      calibration_status = 2; // Failure
      prepareCalibrateResponse();
      command_ready = false;
      return;
    }

    Serial.println("Calibration command received. Resetting position and heading...");

    // Set calibration flag to prevent IMU updates
    calibration_in_progress = true;
    
    // Reset position and heading to zero
    motorControl.resetPosition();
    
    // Force reset heading to 0 immediately
    motorControl.forceResetHeading();
    
    // Reset IMU heading if available
    if (imuControl.isConnected()) {
      imuControl.resetHeading();
    }
    
    // Wait longer for reset to take effect and IMU to stabilize
    delay(1000);
    
    // Re-enable IMU updates
    calibration_in_progress = false;
    
    // Send calibration response
    calibration_status = 0; // Success
    prepareCalibrateResponse();
    
    Serial.println("Calibration complete - Position and heading reset to 0,0,0");
    
  } else if (cmd_id == CMD_RESET) {
    if (cutoff_triggered) {
        Serial.println("Reset ignored due to low-voltage cutoff");

        response_buffer[0] = RESP_RESET;
        response_buffer[1] = 0x00;        // 0 = not accepted / not started
        response_buffer[2] = RESET_NONE;  // No reset performed
        memset(&response_buffer[3], 0x00, 4);
        response_buffer[7] = calculateCRC8Maxim(response_buffer, 7);
        response_ready = true;

        return;
    }
    uint8_t target = i2c_buffer[1]; // Which device(s) to reset
    
    Serial.print("Reset command - Target: 0x");
    Serial.println(target, HEX);
    
    if (!reset_in_progress && target >= RESET_HSSW && target <= RESET_BOTH) {
      // Start the reset cycle
      reset_target = target;
      reset_start_time = millis();
      reset_in_progress = true;
      
      // Turn off requested pins immediately
      if (target & RESET_HSSW) {
        digitalWrite(HSSW_EN_PIN, LOW);
      }
      if (target & RESET_EFUSE) {
        digitalWrite(EFUSE_EN_PIN, LOW);
      }
    }
    
    // Send response
    response_buffer[0] = RESP_RESET;
    response_buffer[1] = reset_in_progress ? 0x01 : 0x00;
    response_buffer[2] = reset_target;
    memset(&response_buffer[3], 0x00, 4); // Clear reserved bytes
    response_buffer[7] = calculateCRC8Maxim(response_buffer, 7);
    response_ready = true;

  } else if (cmd_id == CMD_POWER) {
    uint8_t requested_mode = i2c_buffer[1];
    uint8_t requested_flags = i2c_buffer[2];

    Serial.print("Power command - Mode: 0x");
    Serial.print(requested_mode, HEX);
    Serial.print(", Flags: 0x");
    Serial.println(requested_flags, HEX);

    if (requested_mode == POWER_MODE_SLEEP) {
      enterSleep(requested_flags);
    } else if (requested_mode == POWER_MODE_ACTIVE) {
      exitSleep();
    } else {
      Serial.println("Unknown power mode - reporting current state");
    }

    preparePowerResponse();

  } else {
    Serial.print("Unknown command ID: 0x");
    Serial.println(cmd_id, HEX);
  }
  
  command_ready = false;
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  Serial.println("Enhanced I2C Motor Controller Follower Initializing...");

  // Set ADC resolution to 12-bit for ItsyBitsy M4
  analogReadResolution(12);

  // Initialize motor control system (handles all SERCOM setup internally)
  motorControl.begin(115200);

  // Initialize IMU control system
  if (imuControl.begin()) {
    motorControl.setIMUControl(&imuControl);
    motorControl.enableIMUHeading(true);
    Serial.println("IMU integration enabled");
  } else {
    Serial.println("IMU initialization failed - using wheel odometry only");
  }

  // Set status LED and buzzer pin
  pinMode(HSSW_EN_PIN, OUTPUT);
  digitalWrite(HSSW_EN_PIN, HIGH);

  pinMode(EFUSE_EN_PIN, OUTPUT);
  digitalWrite(EFUSE_EN_PIN, HIGH);
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("Hardware initialized");

  // Initialize I2C as follower with both receive and request handlers
  Wire.begin(I2C_FOLLOWER_ADDRESS);
  Wire.onReceive(receiveEvent);  // Handle incoming commands
  Wire.onRequest(requestEvent);  // Handle data requests from master
  
  Serial.print("Enhanced I2C follower ready at address 0x");
  Serial.println(I2C_FOLLOWER_ADDRESS, HEX);
  Serial.println("Bidirectional communication enabled");
  Serial.println("Waiting for commands from Jetson...");
}

// ---------------------------------------------------------------------
void loop() {

  // Process any received I2C commands first so a wake command takes
  // effect before this pass decides which path to run
  processCommand();

  // Update system status periodically
  static unsigned long last_status_update = 0;
  if (millis() - last_status_update > 1000) { // Update every second
    // Read battery voltage for buzzer monitoring
    battery_voltage = readBatteryVoltage();
    last_status_update = millis();
  }

  // Safety paths run in every power mode
  updateBuzzer();
  updateReset();

  if (power_mode == POWER_MODE_SLEEP) {
    // Low-power path: no motor UART traffic, no IMU polling. I2C stays
    // responsive (interrupt driven), battery/buzzer/cutoff stay active.
    delay(SLEEP_LOOP_DELAY_MS);
    return;
  }

  // Update IMU readings
  imuControl.update();

  // Update heading from IMU when available
  if (!calibration_in_progress && imuControl.isConnected() && imuControl.headingAvailable()) {
    float heading = imuControl.getHeading();
    // Update motor control heading directly
    motorControl.updateHeadingFromIMU(heading);
  }

  motorControl.executeCommand(latestSpeedCmd, latestTurnCmd);
  // Process motor feedback to get actual RPM data
  motorControl.processFeedback();

  // Small delay to prevent overwhelming the loop
  delay(1);
}