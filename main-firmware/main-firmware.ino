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
#include "FirmwareUpdate.h"

// ---------------------------------------------------------------------
// I2C Configuration
#define I2C_FOLLOWER_ADDRESS 0x42  // Must match MCU_ADDR in Jetson code

// Command definitions (Jetson → MCU)
#define CMD_MOVE 0x01
#define CMD_STATUS 0x03
#define CMD_CALIBRATE 0x04
#define CMD_RESET 0x05
#define CMD_BUZZER 0x06     // Buzzer control (query/on/off/snooze/enable/disable)
#define CMD_HSSW 0x07       // High side switch control (query/on/off)
#define CMD_IMU 0x08        // Full IMU orientation request (roll/pitch/yaw)
#define CMD_VERSION 0x09    // Firmware version request
#define CMD_FWUP_BEGIN 0x0A  // Start firmware update session (erase + stage bootloader)
#define CMD_FWUP_DATA 0x0B   // Firmware image data chunk (variable-length frame)
#define CMD_FWUP_STATUS 0x0C // Query update session state
#define CMD_FWUP_COMMIT 0x0D // Verify staged image CRC32
#define CMD_FWUP_APPLY 0x0E  // Bank swap + reset (or rollback)
#define CMD_FWUP_ABORT 0x0F  // Abandon update session

// Response definitions (MCU → Jetson)
#define RESP_MOVE 0x81      // Position feedback
#define RESP_STATUS 0x83    // Health data
#define RESP_CALIBRATE 0x84 // Calibration status
#define RESP_RESET 0x85     // Component Reset
#define RESP_BUZZER 0x86    // Buzzer state
#define RESP_HSSW 0x87      // High side switch state
#define RESP_IMU 0x88       // Full IMU orientation (roll/pitch/yaw)
#define RESP_VERSION 0x89   // Firmware version
// 0x8A-0x8F: firmware update responses (defined in FirmwareUpdate.cpp)

// Firmware version (reported via CMD_VERSION so the Jetson can detect capabilities)
#define FW_VERSION_MAJOR 7
#define FW_VERSION_MINOR 1

// Buzzer sub-commands (byte 1 of CMD_BUZZER)
#define BUZZER_QUERY 0x00
#define BUZZER_MANUAL_ON 0x01
#define BUZZER_MANUAL_OFF 0x02
#define BUZZER_SNOOZE 0x03       // bytes 2-3 = snooze duration in seconds (0 = default)
#define BUZZER_UNSNOOZE 0x04
#define BUZZER_LOWBATT_ENABLE 0x05
#define BUZZER_LOWBATT_DISABLE 0x06

#define BUZZER_DEFAULT_SNOOZE_S 60

// HSSW sub-commands (byte 1 of CMD_HSSW)
#define HSSW_QUERY 0x00
#define HSSW_ON 0x01
#define HSSW_OFF 0x02

// System flags bitfield (RESP_STATUS byte 6, RESP_BUZZER byte 1, RESP_HSSW byte 2)
#define SYSFLAG_BUZZER_ON        0x01  // Buzzer pin currently driven high
#define SYSFLAG_LOWBATT_WARNING  0x02  // Low voltage warning latched (hysteresis)
#define SYSFLAG_LOWBATT_CUTOFF   0x04  // Low voltage cutoff latched
#define SYSFLAG_BUZZER_SNOOZED   0x08  // Low battery buzzer currently snoozed
#define SYSFLAG_BUZZER_ENABLED   0x10  // Low battery buzzer enabled
#define SYSFLAG_HSSW_COMMANDED   0x20  // HSSW commanded on
#define SYSFLAG_BUZZER_MANUAL    0x40  // Buzzer manually commanded on
#define SYSFLAG_RESET_ACTIVE     0x80  // Reset pulse in progress

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

// Jetson-controllable buzzer state
bool buzzer_manual_on = false;        // Jetson requested the buzzer on
bool low_batt_buzzer_enabled = true;  // Master enable for low battery beep pattern
unsigned long snooze_start_ms = 0;    // When the current snooze began
unsigned long snooze_duration_ms = 0; // 0 = not snoozed

// Jetson-controllable high side switch state
bool hssw_commanded = true;           // Desired HSSW state (cutoff/reset can override the pin)

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

// Firmware update chunk staging (filled in the I2C ISR, consumed in loop)
volatile bool dfu_chunk_ready = false;
uint8_t dfu_chunk_buffer[DFU_MAX_FRAME];
volatile uint16_t dfu_chunk_len = 0;

// ---------------------------------------------------------------------
// Buzzer helpers

bool buzzerSnoozed() {
  if (snooze_duration_ms == 0) return false;
  if (millis() - snooze_start_ms >= snooze_duration_ms) {
    snooze_duration_ms = 0; // Snooze expired
    return false;
  }
  return true;
}

uint16_t snoozeRemainingSeconds() {
  if (!buzzerSnoozed()) return 0;
  unsigned long remaining_ms = snooze_duration_ms - (millis() - snooze_start_ms);
  return (uint16_t)(remaining_ms / 1000);
}

void resetBuzzerPattern() {
  buzzer_state = BUZZER_OFF;
  buzz_count = 0;
  buzzer_active = false;
  in_cycle_pause = false;
}

// Drive the buzzer pin: low battery pattern (when active) overlays the manual state
void applyBuzzerOutput(bool pattern_on) {
  digitalWrite(BUZZER_PIN, (pattern_on || buzzer_manual_on) ? HIGH : LOW);
}

// System flags bitfield shared by RESP_STATUS, RESP_BUZZER, and RESP_HSSW
uint8_t getSystemFlags() {
  uint8_t flags = 0;
  if (digitalRead(BUZZER_PIN)) flags |= SYSFLAG_BUZZER_ON;
  if (hysteresis_triggered) flags |= SYSFLAG_LOWBATT_WARNING;
  if (cutoff_triggered) flags |= SYSFLAG_LOWBATT_CUTOFF;
  if (buzzerSnoozed()) flags |= SYSFLAG_BUZZER_SNOOZED;
  if (low_batt_buzzer_enabled) flags |= SYSFLAG_BUZZER_ENABLED;
  if (hssw_commanded) flags |= SYSFLAG_HSSW_COMMANDED;
  if (buzzer_manual_on) flags |= SYSFLAG_BUZZER_MANUAL;
  if (reset_in_progress) flags |= SYSFLAG_RESET_ACTIVE;
  return flags;
}

// ---------------------------------------------------------------------
// Low voltage buzzer management
void updateBuzzer() {
  // Check recovery condition first (hysteresis)
  if (battery_voltage >= HYSTERESIS_THRESHOLD) {
    bool was_triggered = hysteresis_triggered;
    hysteresis_triggered = false;
    cutoff_triggered = false;
    // Voltage recovered - stop pattern, restore power rails
    // (leave a pin alone while a reset pulse is holding it low)
    if (!(reset_in_progress && (reset_target & RESET_HSSW))) {
      digitalWrite(HSSW_EN_PIN, hssw_commanded ? HIGH : LOW);
    }
    if (!(reset_in_progress && (reset_target & RESET_EFUSE))) {
      digitalWrite(EFUSE_EN_PIN, HIGH);
    }
    resetBuzzerPattern();
    if (was_triggered) {
      snooze_duration_ms = 0; // Clear snooze so the next low battery event beeps again
    }
    applyBuzzerOutput(false);
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
    resetBuzzerPattern();
    applyBuzzerOutput(false);
    return;
  }

  // Critical cutoff triggered - constant beep and power off
  // Not snoozable: the EFUSE drops the Jetson here, so nobody is listening anyway
  if (cutoff_triggered) {
    digitalWrite(HSSW_EN_PIN, LOW);
    digitalWrite(EFUSE_EN_PIN, LOW);
    digitalWrite(BUZZER_PIN, HIGH); // assert buzzer on constantly when below cutoff
    return;  // Stay in cutoff state until voltage recovers above hysteresis
  }

  // Warning pattern suppressed while snoozed or disabled (manual control still works)
  if (!low_batt_buzzer_enabled || buzzerSnoozed()) {
    resetBuzzerPattern();
    applyBuzzerOutput(false);
    return;
  }

  // Low voltage warning mode - manage buzzer pattern
  unsigned long current_time = millis();

  switch (buzzer_state) {
    case BUZZER_OFF:
      // Start buzzer cycle
      applyBuzzerOutput(true);
      buzzer_start_time = current_time;
      buzzer_state = BUZZER_ON;
      buzz_count = 1;
      Serial.println("LOW VOLTAGE WARNING - Buzzer ON");
      break;

    case BUZZER_ON:
      // Check if time elapsed
      if (current_time - buzzer_start_time >= 1000) {
        applyBuzzerOutput(false);
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
          applyBuzzerOutput(true);
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
      digitalWrite(HSSW_EN_PIN, HIGH);
      hssw_commanded = true; // A reset pulse always ends with the HSSW on
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

  // Motor winding temps in degrees C from DDSM210 feedback; 0xFF = no fresh data.
  // Revives the R6 spec's motor temperature field (sent as 0x00 in R6..R7.0).
  uint8_t left_temp, right_temp;
  motorControl.getMotorTemps(left_temp, right_temp);

  response_buffer[0] = RESP_STATUS;
  response_buffer[1] = (battery_scaled >> 8) & 0xFF;
  response_buffer[2] = battery_scaled & 0xFF;
  response_buffer[3] = left_temp;
  response_buffer[4] = right_temp;
  response_buffer[5] = fault_code;
  response_buffer[6] = getSystemFlags(); // Was reserved (0x00); additive system flags bitfield
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

void prepareBuzzerResponse(uint8_t ack) {
  uint16_t snooze_remaining = snoozeRemainingSeconds();

  response_buffer[0] = RESP_BUZZER;
  response_buffer[1] = getSystemFlags();
  response_buffer[2] = (snooze_remaining >> 8) & 0xFF;
  response_buffer[3] = snooze_remaining & 0xFF;
  response_buffer[4] = ack; // 0x00 = accepted, 0x01 = rejected/unknown sub-command
  response_buffer[5] = 0x00; // Reserved
  response_buffer[6] = 0x00; // Reserved
  response_buffer[7] = calculateCRC8Maxim(response_buffer, 7);

  response_ready = true;
}

void prepareHSSWResponse(uint8_t ack) {
  uint16_t current_sense = (uint16_t)analogRead(HSSW_IS_PIN);

  response_buffer[0] = RESP_HSSW;
  response_buffer[1] = hssw_commanded ? 0x01 : 0x00;
  response_buffer[2] = getSystemFlags();
  response_buffer[3] = (current_sense >> 8) & 0xFF;
  response_buffer[4] = current_sense & 0xFF;
  response_buffer[5] = digitalRead(HSSW_EN_PIN) ? 0x01 : 0x00; // Actual pin level
  response_buffer[6] = ack; // 0x00 = accepted, 0x01 = rejected (cutoff), 0x02 = unknown sub-command
  response_buffer[7] = calculateCRC8Maxim(response_buffer, 7);

  response_ready = true;
}

void prepareIMUResponse() {
  // Angles in milliradians; 0x7FFF in all fields means IMU unavailable
  int16_t roll_mrad = 0x7FFF;
  int16_t pitch_mrad = 0x7FFF;
  int16_t yaw_mrad = 0x7FFF;

  if (imuControl.isConnected()) {
    float roll, pitch, yaw;
    imuControl.getOrientation(roll, pitch, yaw);
    roll_mrad = (int16_t)(roll * 1000.0f);
    pitch_mrad = (int16_t)(pitch * 1000.0f);
    yaw_mrad = (int16_t)(yaw * 1000.0f);
  }

  response_buffer[0] = RESP_IMU;
  splitInt16(roll_mrad, response_buffer[1], response_buffer[2]);
  splitInt16(pitch_mrad, response_buffer[3], response_buffer[4]);
  splitInt16(yaw_mrad, response_buffer[5], response_buffer[6]);
  response_buffer[7] = calculateCRC8Maxim(response_buffer, 7);

  response_ready = true;
}

void prepareVersionResponse() {
  response_buffer[0] = RESP_VERSION;
  response_buffer[1] = FW_VERSION_MAJOR;
  response_buffer[2] = FW_VERSION_MINOR;
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
  if (numBytes >= 7) {
    // Read register byte (should be 0x00) and peek the frame ID
    uint8_t register_byte = Wire.read();
    (void)register_byte;
    uint8_t frame_id = Wire.read();

    if (frame_id == CMD_FWUP_DATA) {
      // Variable-length firmware data chunk: [0x0B][seq_hi][seq_lo][len][data...][CRC8]
      if (!dfu_chunk_ready) {
        dfu_chunk_buffer[0] = frame_id;
        uint16_t i = 1;
        while (Wire.available() && i < sizeof(dfu_chunk_buffer)) {
          dfu_chunk_buffer[i++] = Wire.read();
        }
        while (Wire.available()) {
          Wire.read();
        }
        dfu_chunk_len = i;
        dfu_chunk_ready = true;
      } else {
        // Previous chunk not yet processed - drop; leader recovers via seq ack
        while (Wire.available()) {
          Wire.read();
        }
      }
      return;
    }

    if (numBytes == 9) {  // 8-byte command + 1 register byte
      i2c_buffer[0] = frame_id;
      for (int i = 1; i < 8 && Wire.available(); i++) {
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
    } else {
      // Unknown frame shape - discard
      while (Wire.available()) {
        Wire.read();
      }
      Serial.println("Invalid command length");
    }
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
    digitalWrite(HSSW_EN_PIN, hssw_commanded ? HIGH : LOW);
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
    
  } else if (cmd_id == CMD_BUZZER) {
    uint8_t sub_cmd = i2c_buffer[1];
    uint8_t ack = 0x00;

    Serial.print("Buzzer command - Sub: 0x");
    Serial.println(sub_cmd, HEX);

    switch (sub_cmd) {
      case BUZZER_QUERY:
        break;
      case BUZZER_MANUAL_ON:
        buzzer_manual_on = true;
        digitalWrite(BUZZER_PIN, HIGH);
        break;
      case BUZZER_MANUAL_OFF:
        buzzer_manual_on = false;
        // Only drop the pin if the low battery pattern isn't mid-beep
        if (buzzer_state != BUZZER_ON && !cutoff_triggered) {
          digitalWrite(BUZZER_PIN, LOW);
        }
        break;
      case BUZZER_SNOOZE: {
        uint16_t seconds = ((uint16_t)i2c_buffer[2] << 8) | i2c_buffer[3];
        if (seconds == 0) seconds = BUZZER_DEFAULT_SNOOZE_S;
        snooze_start_ms = millis();
        snooze_duration_ms = (unsigned long)seconds * 1000UL;
        Serial.print("Low battery buzzer snoozed for ");
        Serial.print(seconds);
        Serial.println("s");
        break;
      }
      case BUZZER_UNSNOOZE:
        snooze_duration_ms = 0;
        break;
      case BUZZER_LOWBATT_ENABLE:
        low_batt_buzzer_enabled = true;
        break;
      case BUZZER_LOWBATT_DISABLE:
        low_batt_buzzer_enabled = false;
        break;
      default:
        ack = 0x01; // Unknown sub-command
        break;
    }

    prepareBuzzerResponse(ack);

  } else if (cmd_id == CMD_HSSW) {
    uint8_t sub_cmd = i2c_buffer[1];
    uint8_t ack = 0x00;

    Serial.print("HSSW command - Sub: 0x");
    Serial.println(sub_cmd, HEX);

    switch (sub_cmd) {
      case HSSW_QUERY:
        break;
      case HSSW_ON:
        if (cutoff_triggered) {
          ack = 0x01; // Rejected: low voltage cutoff is forcing the HSSW off
        } else {
          hssw_commanded = true;
          if (!(reset_in_progress && (reset_target & RESET_HSSW))) {
            digitalWrite(HSSW_EN_PIN, HIGH);
          }
        }
        break;
      case HSSW_OFF:
        hssw_commanded = false;
        digitalWrite(HSSW_EN_PIN, LOW);
        break;
      default:
        ack = 0x02; // Unknown sub-command
        break;
    }

    prepareHSSWResponse(ack);

  } else if (cmd_id == CMD_IMU) {
    Serial.println("IMU orientation request received");
    prepareIMUResponse();

  } else if (cmd_id == CMD_VERSION) {
    Serial.println("Version request received");
    prepareVersionResponse();

  } else if (cmd_id == CMD_FWUP_BEGIN) {
    dfuHandleBegin(i2c_buffer, response_buffer);
    response_ready = true;

  } else if (cmd_id == CMD_FWUP_STATUS) {
    dfuHandleStatus(response_buffer);
    response_ready = true;

  } else if (cmd_id == CMD_FWUP_COMMIT) {
    dfuHandleCommit(i2c_buffer, response_buffer);
    response_ready = true;

  } else if (cmd_id == CMD_FWUP_APPLY) {
    dfuHandleApply(i2c_buffer, response_buffer);
    response_ready = true;

  } else if (cmd_id == CMD_FWUP_ABORT) {
    dfuHandleAbort(response_buffer);
    response_ready = true;

  } else {
    Serial.print("Unknown command ID: 0x");
    Serial.println(cmd_id, HEX);
  }

  command_ready = false;
}

// ---------------------------------------------------------------------
void setup() {
  // Assert power rails first: EFUSE_EN keeps the Jetson powered, so it must go
  // high as early as possible after any MCU reset (before serial delays or the
  // IMU init retry loop, which can block indefinitely if the IMU is absent)
  pinMode(EFUSE_EN_PIN, OUTPUT);
  digitalWrite(EFUSE_EN_PIN, HIGH);

  pinMode(HSSW_EN_PIN, OUTPUT);
  digitalWrite(HSSW_EN_PIN, HIGH);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

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
  
  // Update IMU readings
  imuControl.update();
  
  // Update heading from IMU when available
  if (!calibration_in_progress && imuControl.isConnected() && imuControl.headingAvailable()) {
    float heading = imuControl.getHeading();
    // Update motor control heading directly
    motorControl.updateHeadingFromIMU(heading);
  }
  
  // Process any received I2C commands
  processCommand();

  // Process firmware update data chunks and pending bank swaps
  if (dfu_chunk_ready) {
    dfuHandleDataChunk(dfu_chunk_buffer, dfu_chunk_len, response_buffer);
    response_ready = true;
    dfu_chunk_ready = false;
  }
  dfuUpdate();


  motorControl.executeCommand(latestSpeedCmd, latestTurnCmd);
  // Process motor feedback to get actual RPM data
  motorControl.processFeedback();
  
  // Update buzzer for low voltage warning (independent of Jetson communication)
  updateBuzzer();
  updateReset();
  
  // Update system status periodically
  static unsigned long last_status_update = 0;
  if (millis() - last_status_update > 1000) { // Update every second
    // Read battery voltage for buzzer monitoring
    battery_voltage = readBatteryVoltage();
    last_status_update = millis();
  }
  
  // Small delay to prevent overwhelming the loop
  delay(1);
}