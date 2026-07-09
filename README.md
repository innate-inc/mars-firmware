# Micro Controller Protocol

## 1. Overview

This document describes the **R6 protocol**, which defines communication between the robot's main computer and its microcontroller. The R6 protocol was originally designed for a UART interface; however, the current firmware implementation uses **I2C** as the physical transport (see `main-firmware.ino`). The message framing and payload structure described below still apply. Each transmitted message consists of:

- **2 bytes for the Start-of-Message (SOM) marker:** Fixed value 0x69 0x69, which delineates the start of a new message.
- **8 bytes of protocol message:** This includes 7 bytes of payload (ID and data fields) and 1 byte of CRC (calculated using CRC‑8/MAXIM over the payload bytes).

The overall physical message length is therefore **10 bytes**.

---

## 2. Physical Layer Parameters (R6 Spec — UART)

> **Note:** The UART parameters below are part of the original R6 specification. The current firmware communicates over **I2C** (follower address `0x42`) instead of UART. The message structure and CRC scheme remain the same; only the transport layer differs.

The R6 UART interface is configured as follows:

- **Baud Rate:** 115200 baud
    
    (115200 bits are transmitted per second.)
    
- **Data Bits:** 8 bits per frame
    
    (Each transmitted data frame contains 8 bits of actual data.)
    
- **Parity:** None
    
    (No parity bit is used.)
    
- **Stop Bits:** 1 stop bit
    
    (One stop bit follows each data byte.)
    
- **Frame Size:** 10 bits per byte
    
    (1 start bit, 8 data bits, and 1 stop bit per byte.)
    

*Note:* In the R6 UART spec, the SOM marker and each subsequent byte are transmitted using these framing settings. Over I2C the SOM marker is not used; the main computer writes the 8-byte payload directly to the follower address.

---

## 3. Message Structure

Each physical message transmitted is 10 bytes long and is structured as follows:

| Byte Offset | Field Name | Size | Description |
| --- | --- | --- | --- |
| 0–1 | **SOM Marker** | 2 bytes | A fixed two‐byte sequence that marks the start of a message. **Value:** 0x69 0x69. |
| 2 | ID | 1 byte | Command ID (for commands) or Response ID (for responses). |
| 3 | Data 1 | 1 byte | First data parameter or part of a multi‑byte field. |
| 4 | Data 2 | 1 byte | Second data parameter or part of a multi‑byte field. |
| 5 | Data 3 | 1 byte | Third data parameter or part of a multi‑byte field. |
| 6 | Data 4 | 1 byte | Fourth data parameter or part of a multi‑byte field. |
| 7 | Data 5 | 1 byte | Fifth data parameter or part of a multi‑byte field. |
| 8 | Data 6 | 1 byte | Sixth data parameter or reserved/padding when fewer than 6 data bytes are used. |
| 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over the 7-byte payload (i.e. bytes 2–8 of the full message). The SOM marker is not included in the CRC calculation. |

### CRC Calculation Details

- **Algorithm:** CRC‑8/MAXIM
- **Polynomial (Reflected):** 0x8C
- **Initial Value:** 0x00
- **Data for CRC:** The CRC is computed over bytes 2–8 (the entire protocol message excluding the 2-byte SOM marker).
- **Example Implementation in C:**
    
    ```c
    byte calculate_crc8_maxim(byte *data, int len) {
        byte crc = 0x00;
        while (len--) {
            crc ^= *data++;
            for (byte i = 0; i < 8; i++) {
                if (crc & 1)
                    crc = (crc >> 1) ^ 0x8C;
                else
                    crc >>= 1;
            }
        }
        return crc;
    }
    ```
    

*Note:* In this example, `data` points to byte offset 2 of the full message, and `len` is 7.

---

## 4. Command Message Types

Commands are sent from the main computer to the microcontroller. The first byte of the protocol message (byte 2 of the overall 10-byte message) is the command ID.

### 4.1. Movement (Set Speed) Command

- **Command ID:** `0x01`
- **Purpose:** Set the robot's forward speed and turn rate.
- **Payload Layout (Bytes 2–8):**
    
    
    | Byte Offset (of protocol message) | Field Name | Size | Description |
    | --- | --- | --- | --- |
    | 2 | Cmd ID | 1 byte | `0x01` (Movement command) |
    | 3–4 | Forward Speed | 2 bytes | 16‑bit signed fixed‑point value for forward speed, scaled by 100 (e.g., +0.64 → +64; range: ±1.28 → ±128). |
    | 5–6 | Turn Rate | 2 bytes | 16‑bit signed fixed‑point value for turn rate, scaled by 100 (e.g., −1.28 → −128; range: ±2.56 → ±256). |
    | 7–8 | Reserved | 2 bytes | Padding bytes; set to `0x00`. |
    | 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over bytes 2–8. |

### 4.2. LED Command

- **Command ID:** `0x02`
- **Purpose:** Configure the LED parameters.
- **Payload Layout (Bytes 2–8):**
    
    
    | Byte Offset | Field | Size | Description |
    | --- | --- | --- | --- |
    | 2 | Cmd ID | 1 byte | `0x02` (LED command) |
    | 3 | LED Mode | 1 byte | LED mode: 0 = Off, 1 = Solid, 2 = Blink, 3 = Ring |
    | 4–5 | Time Parameter | 2 bytes | 16‑bit unsigned integer representing time in milliseconds (range: 1–10,000 ms). Byte 4 is the high byte; byte 5 is the low byte. |
    | 6 | Red | 1 byte | Red intensity (0–255) |
    | 7 | Green | 1 byte | Green intensity (0–255) |
    | 8 | Blue | 1 byte | Blue intensity (0–255) |
    | 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over bytes 2–8. |

### 4.3. Status Request Command

- **Command ID:** `0x03`
- **Purpose:** Request a system status update.
- **Payload Layout (Bytes 2–8):**
    
    
    | Byte Offset | Field | Size | Description |
    | --- | --- | --- | --- |
    | 2 | Cmd ID | 1 byte | `0x03` (Status Request command) |
    | 3–8 | Reserved | 6 bytes | Set to `0x00` (no additional data required). |
    | 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over bytes 2–8. |
    
### 4.4. Calibration Command

- **Command ID:** `0x04`
- **Purpose:** Trigger the robot's self-calibration routine.

**Full 10-byte Layout:**

| Byte Offset | Field | Size | Value / Description |
| --- | --- | --- | --- |
| 0–1 | **SOM** | 2 | `0x69 0x69` |
| 2 | Cmd ID | 1 | `0x04` |
| 3–8 | Reserved | 6 | All `0x00` |
| 9 | **CRC** | 1 | CRC-8/MAXIM over bytes 2–8 |

### 4.5. Power Mode Command

- **Command ID:** `0x06`
- **Purpose:** Put the microcontroller into low-power sleep mode, or wake it back to active mode.

**Full 10-byte Layout:**

| Byte Offset | Field | Size | Value / Description |
| --- | --- | --- | --- |
| 0–1 | **SOM** | 2 | `0x69 0x69` |
| 2 | Cmd ID | 1 | `0x06` |
| 3 | Power Mode | 1 | 0x00 = Active (wake), 0x01 = Sleep |
| 4 | Sleep Flags | 1 | Bitfield, only used when entering sleep. See below. |
| 5 | Motion Threshold | 1 | Only used with wake-on-motion. Detection threshold in 4 mg steps (value × 4 mg, range 4–1020 mg). 0 = firmware default (120 mg). |
| 6–8 | Reserved | 3 | All `0x00` |
| 9 | **CRC** | 1 | CRC-8/MAXIM over bytes 2–8 |

**Sleep Flags (byte 4) bitfield:**

| Bit | Name | Description |
| --- | --- | --- |
| 0 | Gate HSSW | Hold the HSSW peripheral rail off for the duration of sleep. The HSSW rail powers the arm+head Dynamixels and the wheel motor drivers. |
| 1 | Wake on Motion | Keep the IMU accelerometer alive during sleep and watch for a physical disturbance (tap/nudge/pickup). |
| 2 | Motion Auto-Wake | Have the MCU auto-exit sleep the instant motion is detected, rather than only latching the event for the main computer to act on. Requires bit 1 (ignored and cleared without it). |

The Sleep Flags reported back (in the Power Mode response and while asleep) are the **effective** flags: bit 2 is cleared if bit 1 was not set, and bits 1–2 are cleared if the IMU failed to initialize at boot (no motion wake is possible — the main computer must not rely on a physical wake path that does not exist).

**Sleep mode behavior:**

- Motors receive a final zero-speed command, then all motor UART traffic stops.
- The main loop drops from ~1 kHz to ~50 Hz.
- I2C stays fully responsive; battery monitoring, the low-voltage buzzer, and the low-voltage cutoff keep running.
- Movement commands (`0x01`) received during sleep are acknowledged with a position response but do **not** drive the motors. Waking is only done explicitly via this command with mode 0x00 (or by motion auto-wake, if enabled).
- Calibration commands (`0x04`) received during sleep are refused with a Failure status.
- **IMU:** without the Wake-on-Motion flag, the ICM-20948 is put into full hardware sleep (configuration retained). With the flag, the accelerometer is kept running (DMP disabled) and sampled at the sleep-loop rate (~50 Hz). A frame-to-frame change in acceleration magnitude exceeding the threshold on two consecutive samples latches a "motion wake pending" event. The IMU sits on the MCU's own I2C bus (not the HSSW rail), so wake-on-motion keeps working even when the rail is gated — it is the physical wake path that gating would otherwise remove. Note that with the arm unpowered and limp, a gentle head-boop may not couple enough acceleration into the chassis; this is a "tap/nudge/pickup" wake, tunable via the threshold.
- The latched motion event is reported to the main computer two ways: bit 1 of the Status response power/wake-flags byte (so the routine battery poll surfaces it) and byte 6 of the Power Mode response. The latch is cleared by every Power Mode command — a sleep command re-arms it, an active command acknowledges it — and it deliberately **survives a motion auto-wake**, so after an auto-wake the status poll shows "active + motion wake pending" and the main computer can see why the robot woke. The Power Mode response always reflects the post-command state (an active command's response therefore reports the latch already cleared).
- If bit 0 of the Sleep Flags is set, the HSSW enable pin is held low for the whole sleep period (including through low-voltage recovery and reset cycles). This cuts power to the arm+head Dynamixels and the wheel motor drivers. The main computer must sequence around this: the Dynamixel bus is dead while gated (so any touch/boop wake detection based on servo polling stops working — use wake-on-motion instead) and its arm node should stop polling the bus. The EFUSE rail is never gated by sleep — it powers the main computer.

**Wake behavior:**

- The HSSW rail is restored (unless the low-voltage cutoff is active).
- If the rail was gated during sleep, the wheel motor drivers were power cycled: the firmware waits 200 ms for the rail to settle and then re-sends their ID/mode configuration. The arm+head Dynamixels also come back up at this point; re-initializing them is the main computer's job.
- The IMU is resumed to full active/DMP operation and its DMP/FIFO reset so the first heading after wake is fresh.
- With Motion Auto-Wake enabled, the MCU performs this wake itself the instant motion is detected — the rail comes up and wheels re-init while the main computer is still noticing the latched bit, hiding the ~550 ms rail-settle + re-init latency. The motion-wake-pending latch survives the auto-wake and is cleared when the main computer acknowledges with an active-mode Power command (or re-arms with a new sleep command).

---

## 5. Response Message Types

Responses are sent from the microcontroller back to the main computer. The convention is to derive the response ID by adding `0x80` to the originating command ID.

### 5.1. Movement (Position Update) Response

- **Response ID:** `0x81` (i.e., 0x01 + 0x80)
- **Purpose:** Report the robot's current position.
- **Payload Layout (Bytes 2–8):**
    
    
    | Byte Offset | Field | Size | Description |
    | --- | --- | --- | --- |
    | 2 | Response ID | 1 byte | `0x81` indicates a Movement (Position Update) response. |
    | 3–4 | X Coordinate | 2 bytes | 16‑bit signed/fixed‑point value for the x-coordinate. |
    | 5–6 | Y Coordinate | 2 bytes | 16‑bit signed/fixed‑point value for the y-coordinate. |
    | 7–8 | Theta | 2 bytes | 16‑bit signed/fixed‑point value for the heading (θ). |
    | 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over bytes 2–8. |

### 5.2. LED Response

- **Response ID:** `0x82` (i.e., 0x02 + 0x80)
- **Purpose:** Report the current LED settings.
- **Payload Layout (Bytes 2–8):**
    
    
    | Byte Offset | Field | Size | Description |
    | --- | --- | --- | --- |
    | 2 | Response ID | 1 byte | `0x82` indicates an LED response. |
    | 3 | LED Mode | 1 byte | Current LED mode (0 = Off, 1 = Solid, 2 = Blink, 3 = Ring). |
    | 4 | Red | 1 byte | Current red intensity (0–255). |
    | 5 | Green | 1 byte | Current green intensity (0–255). |
    | 6 | Blue | 1 byte | Current blue intensity (0–255). |
    | 7–8 | Time Parameter | 2 bytes | Current 16‑bit time parameter in milliseconds (byte 7 is high, byte 8 is low). |
    | 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over bytes 2–8. |

### 5.3. Status Response

- **Response ID:** `0x83` (i.e., 0x03 + 0x80)
- **Purpose:** Report system status including battery voltage, motor temperature, and fault codes.
- **Payload Layout (Bytes 2–8):**
    
    
    | Byte Offset | Field | Size | Description |
    | --- | --- | --- | --- |
    | 2 | Response ID | 1 byte | `0x83` indicates a Status Response. |
    | 3–4 | Battery Voltage | 2 bytes | 16‑bit unsigned value representing battery voltage scaled by 100 (e.g., 12.34V → 1234). |
    | 5–6 | Motor Temperature | 2 bytes | 16‑bit value for motor temperature (units and scaling defined by the system). |
    | 7 | Fault Code | 1 byte | Fault code or bitfield indicating errors or warnings. |
    | 8 | Power/Wake Flags | 1 byte | Bitfield (previously reserved). Bit 0: sleep mode active. Bit 1: motion wake pending (a physical disturbance was detected during sleep). |
    | 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over bytes 2–8. |

### 5.4. Calibration Response

- **Response ID:** `0x84` (i.e. `0x04 + 0x80`)
- **Purpose:** Return the status of the calibration.

**Full 10-byte Layout:**

| Byte Offset | Field | Size | Value / Description |
| --- | --- | --- | --- |
| 0–1 | **SOM** | 2 | `0x69 0x69` |
| 2 | Resp ID | 1 | `0x84` |
| 3 | Status | 1 | 0 = Success, 1 = In Progress, 2 = Failure |
| 4–8 | Reserved | 5 | All `0x00` |
| 9 | **CRC** | 1 | CRC-8/MAXIM over bytes 2–8 |

### 5.5. Power Mode Response

- **Response ID:** `0x86` (i.e. `0x06 + 0x80`)
- **Purpose:** Report the power mode state after a Power Mode command.

**Full 10-byte Layout:**

| Byte Offset | Field | Size | Value / Description |
| --- | --- | --- | --- |
| 0–1 | **SOM** | 2 | `0x69 0x69` |
| 2 | Resp ID | 1 | `0x86` |
| 3 | Power Mode | 1 | Current mode: 0x00 = Active, 0x01 = Sleep |
| 4 | Sleep Flags | 1 | Active sleep flags (0x00 when awake) |
| 5 | Motors Reinit | 1 | 0x01 if the motor drivers were re-initialized on the last wake (rail was gated), else 0x00 |
| 6 | Motion Wake Pending | 1 | 0x01 if a physical disturbance was detected during sleep (was reserved), else 0x00 |
| 7–8 | Reserved | 2 | All `0x00` |
| 9 | **CRC** | 1 | CRC-8/MAXIM over bytes 2–8 |

---

## 6. Implementation and Synchronization (R6 / UART)

> **Note:** The synchronization scheme below applies to the original R6 UART transport. The current I2C implementation does not use the SOM marker; message boundaries are handled by the I2C protocol itself.

- **Message Reception:**
    
    In the R6 UART mode, the receiver monitors the stream for the two-byte SOM marker (0x69 0x69). When this marker is detected, it begins buffering the next 8 bytes, which form the protocol message.
    
- **CRC Verification:**
    
    Once 8 bytes are buffered (bytes 2–9 of the overall 10-byte message), the receiver computes the CRC‑8/MAXIM over bytes 2–8 and compares it with the received CRC in byte 9. If the CRC matches, the message is accepted as valid.
    
- **Resynchronization:**
    
    If the expected SOM marker is not found or if the CRC fails, the receiver discards the current data and scans for the next occurrence of 0x69 0x69. This explicit SOM marker ensures that message boundaries are clearly defined.
    

---

## 7. Battery Voltage Reading

The battery voltage is read through pin A5 using a voltage divider circuit with the following specifications:

- **Top Resistor (R_TOP):** 13.3 kΩ
- **Bottom Resistor (R_BOTTOM):** 4.7 kΩ  
- **ADC Reference Voltage:** 3.3V
- **ADC Resolution:** 12-bit (0–4095)

The voltage conversion formula is:

```
pin_voltage    = raw_adc * (3.3 / 4095)
battery_voltage = pin_voltage * (R_TOP + R_BOTTOM) / R_BOTTOM
                = pin_voltage * (13.3 + 4.7) / 4.7
```