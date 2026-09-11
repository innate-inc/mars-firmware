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

### 4.5. Buzzer Command (R7.1 addition)

- **Command ID:** `0x06`
- **Purpose:** Control the buzzer: query state, manual on/off, snooze the low battery warning, or enable/disable the low battery warning entirely.
- **Payload Layout (Bytes 2–8):**

    | Byte Offset | Field | Size | Description |
    | --- | --- | --- | --- |
    | 2 | Cmd ID | 1 byte | `0x06` (Buzzer command) |
    | 3 | Sub-command | 1 byte | `0x00` = Query, `0x01` = Manual ON, `0x02` = Manual OFF, `0x03` = Snooze low battery warning, `0x04` = Cancel snooze, `0x05` = Enable low battery warning, `0x06` = Disable low battery warning |
    | 4–5 | Snooze Duration | 2 bytes | Only used with sub-command `0x03`: 16-bit unsigned snooze duration in seconds (`0` = default 60 s). Byte 4 is high byte. |
    | 6–8 | Reserved | 3 bytes | Set to `0x00`. |
    | 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over bytes 2–8. |

    **Behavior notes:**
    - The low battery warning pattern (3 beeps + 10 s pause) acts like an interrupt: while active it overrides the manual buzzer state; when it ends (or is snoozed/disabled) the buzzer returns to the manual state.
    - Snooze suppresses only the low battery *warning* pattern; it auto-expires after the requested duration and is cleared when the battery voltage recovers above the hysteresis threshold.
    - The constant beep in the low voltage *cutoff* state is not snoozable (the e-fuse has already cut Jetson power at that point).

### 4.6. High Side Switch Command (R7.1 addition)

- **Command ID:** `0x07`
- **Purpose:** Query or set the high side switch (HSSW) output directly, in addition to the fixed-duration pulse of the Reset command (`0x05`).
- **Payload Layout (Bytes 2–8):**

    | Byte Offset | Field | Size | Description |
    | --- | --- | --- | --- |
    | 2 | Cmd ID | 1 byte | `0x07` (HSSW command) |
    | 3 | Sub-command | 1 byte | `0x00` = Query, `0x01` = ON, `0x02` = OFF |
    | 4–8 | Reserved | 5 bytes | Set to `0x00`. |
    | 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over bytes 2–8. |

    **Behavior notes:**
    - `ON` is rejected while the low voltage cutoff is latched (the firmware is forcing the HSSW off to protect the battery).
    - A Reset command (`0x05`) targeting the HSSW still pulses it off for 1 s; the pulse always ends with the HSSW on (commanded state is set back to ON).
    - The commanded state survives a low-battery cutoff/recovery cycle: on voltage recovery the HSSW is restored to the last commanded state, not blindly turned on.

### 4.7. IMU Orientation Request (R7.1 addition)

- **Command ID:** `0x08`
- **Purpose:** Request the full IMU orientation (roll, pitch, yaw) rather than only the heading embedded in the Movement response.
- **Payload Layout:** Bytes 3–8 reserved (`0x00`), CRC over bytes 2–8.

### 4.8. Firmware Version Request (R7.1 addition)

- **Command ID:** `0x09`
- **Purpose:** Query the firmware version so the main computer can detect which protocol features are available. Firmware older than 7.1 does not answer this command (the response read will return all zeros), which itself signals that only the base R6 command set is supported.
- **Payload Layout:** Bytes 3–8 reserved (`0x00`), CRC over bytes 2–8.

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
    | 8 | System Flags | 1 byte | **R7.1 addition** (previously reserved, always `0x00`): system flags bitfield — see [Section 5.9](#59-system-flags-bitfield). Firmware older than 7.1 always sends `0x00` here, so parsers that ignored this byte remain compatible. |
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

### 5.5. Buzzer Response (R7.1 addition)

- **Response ID:** `0x86` (i.e., 0x06 + 0x80)
- **Purpose:** Report buzzer and low battery warning state. Sent in reply to any Buzzer command, including queries.
- **Payload Layout (Bytes 2–8):**

    | Byte Offset | Field | Size | Description |
    | --- | --- | --- | --- |
    | 2 | Response ID | 1 byte | `0x86` |
    | 3 | System Flags | 1 byte | Bitfield — see [Section 5.9](#59-system-flags-bitfield). |
    | 4–5 | Snooze Remaining | 2 bytes | 16-bit unsigned seconds remaining on the current snooze (`0` if not snoozed). Byte 4 is high byte. |
    | 6 | Ack | 1 byte | `0x00` = sub-command accepted, `0x01` = unknown sub-command. |
    | 7–8 | Reserved | 2 bytes | `0x00`. |
    | 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over bytes 2–8. |

### 5.6. High Side Switch Response (R7.1 addition)

- **Response ID:** `0x87` (i.e., 0x07 + 0x80)
- **Purpose:** Report HSSW state and current-sense telemetry. Sent in reply to any HSSW command, including queries.
- **Payload Layout (Bytes 2–8):**

    | Byte Offset | Field | Size | Description |
    | --- | --- | --- | --- |
    | 2 | Response ID | 1 byte | `0x87` |
    | 3 | Commanded State | 1 byte | `0x01` = ON commanded, `0x00` = OFF commanded. |
    | 4 | System Flags | 1 byte | Bitfield — see [Section 5.9](#59-system-flags-bitfield). |
    | 5–6 | Current Sense | 2 bytes | Raw 12-bit ADC reading of the HSSW current-sense pin (A0), 16-bit unsigned, byte 5 is high byte. |
    | 7 | Actual Pin Level | 1 byte | `0x01` = HSSW enable pin currently high, `0x00` = low (can differ from commanded state during a reset pulse or cutoff). |
    | 8 | Ack | 1 byte | `0x00` = accepted, `0x01` = ON rejected due to low voltage cutoff, `0x02` = unknown sub-command. |
    | 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over bytes 2–8. |

### 5.7. IMU Orientation Response (R7.1 addition)

- **Response ID:** `0x88` (i.e., 0x08 + 0x80)
- **Purpose:** Report the full IMU orientation.
- **Payload Layout (Bytes 2–8):**

    | Byte Offset | Field | Size | Description |
    | --- | --- | --- | --- |
    | 2 | Response ID | 1 byte | `0x88` |
    | 3–4 | Roll | 2 bytes | 16-bit signed roll (rotation about X) in **milliradians**. |
    | 5–6 | Pitch | 2 bytes | 16-bit signed pitch (rotation about Y) in **milliradians**. |
    | 7–8 | Yaw | 2 bytes | 16-bit signed yaw (heading, calibration offset applied — same angle as Theta in the Movement response) in **milliradians**. |
    | 9 | **CRC** | 1 byte | CRC‑8/MAXIM computed over bytes 2–8. |

    **Note:** All three fields are set to `0x7FFF` (32767) when the IMU is not connected. Milliradian scaling (×1000) gives ~0.057° resolution, finer than the ×100 radian scaling used for Theta in the Movement response; the legacy Theta field is unchanged.

### 5.8. Firmware Version Response (R7.1 addition)

- **Response ID:** `0x89` (i.e., 0x09 + 0x80)
- **Payload Layout:** Byte 3 = major version, byte 4 = minor version (e.g., `7`, `1` for firmware R7.1), bytes 5–8 reserved (`0x00`), CRC over bytes 2–8.

### 5.9. System Flags Bitfield

Shared bitfield reported in the Status response (byte 8), Buzzer response (byte 3), and HSSW response (byte 4):

| Bit | Mask | Meaning |
| --- | --- | --- |
| 0 | `0x01` | Buzzer pin currently driven high |
| 1 | `0x02` | Low voltage warning latched (battery dipped below 10.5 V, not yet recovered above 11.0 V) |
| 2 | `0x04` | Low voltage cutoff latched (battery dipped below 10.3 V — HSSW and e-fuse forced off) |
| 3 | `0x08` | Low battery buzzer currently snoozed |
| 4 | `0x10` | Low battery buzzer enabled (default 1) |
| 5 | `0x20` | HSSW commanded on |
| 6 | `0x40` | Buzzer manually commanded on |
| 7 | `0x80` | Reset pulse in progress |

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
- **ADC Resolution:** 12-bit (0–4095) — the firmware calls `analogReadResolution(12)` on the ItsyBitsy M4

The voltage conversion formula is:

```
V_battery = (raw / 4095) × 3.3 × (R_TOP + R_BOTTOM) / R_BOTTOM
```

---

## 8. Firmware Update over I2C — Feasibility (R7.1 analysis)

**Question:** Can the Jetson reflash the MCU over the existing I2C bus, given that the Jetson's own power depends on the MCU holding the e-fuse enable line high?

**Short answer:** Not with the stock setup, but yes with an application-level A/B update scheme — with one hardware caveat around the reset moment.

### Why it doesn't work today

The ItsyBitsy M4's stock UF2 bootloader (16 KB, protected region at the start of flash) only speaks **USB**: UF2 drag-and-drop mass storage and BOSSA/SAM-BA over USB CDC. It has no I2C download path, so the Jetson cannot talk to it. Entering the bootloader also tristates all GPIO — the e-fuse enable (pin 10) would float, the e-fuse would drop, and the Jetson would power itself off while sitting in a bootloader that waits forever for USB. That is the bricking scenario to avoid.

### Recommended approach: application-level A/B update (no bootloader changes)

The SAMD51 has 512 KB of flash in **two independently writable banks** (BANKA/BANKB, 256 KB each) and native NVMCTRL support for **bank swapping** (`BKSWRST` command swaps which bank is mapped to address 0 and resets). The current firmware uses ~64 KB, so an entire new image fits in the inactive bank with a wide margin. The update flow:

1. **Transfer:** Add new I2C commands (e.g., `0x0A` update-begin / `0x0B` data-chunk / `0x0C` verify+commit). The running application receives the image in chunks (a new, longer I2C write frame with a sequence number and per-chunk CRC), erasing/writing the inactive bank as it goes. The robot stays fully operational during transfer — the running code is untouched, and I2C writes to flash can be paced so motor/battery/buzzer duties are not starved.
2. **Verify:** After the last chunk, the MCU checks a whole-image CRC32 before anything irreversible happens. A failed transfer is harmless: the active bank still runs, just start over.
3. **Commit:** Issue `BKSWRST`. The MCU resets into the new image; the old image remains intact in the now-inactive bank for rollback (pair this with a watchdog + "boot confirmed" command from the Jetson: if the new firmware never gets confirmed, swap back).

### The reset-moment caveat

During the `BKSWRST` reset, GPIO tristates until the new application re-asserts `EFUSE_EN`. As of R7.1, `setup()` asserts the power pins **first**, before serial delays and the (potentially blocking) IMU init, so the gap is roughly the POR + bootloader-jump time — on the order of ~100 ms. What happens in that window depends on the e-fuse enable circuit:

- If `EFUSE_EN` has a pull-up or RC hold-up that keeps the e-fuse enabled for ~200 ms, the Jetson never notices. **Recommended next PCB rev change if not already present.**
- If the enable floats off, the Jetson browns out and reboots. This is annoying but *not* a brick: the MCU completes the bank swap autonomously once the image is transferred, boots the new firmware, re-asserts the e-fuse, and the Jetson comes back up on new firmware.

The unrecoverable-without-USB case is a new image that hard-faults before asserting `EFUSE_EN`. The CRC check plus watchdog-rollback closes most of that gap; keeping the power-pin assertions as the first statements of `setup()` (and never moving them) closes the rest.

### Alternatives considered

- **Custom I2C bootloader** (fork the UF2 bootloader to assert `EFUSE_EN` immediately at reset and act as an I2C follower for DFU): fully robust against bad application images, but bootloader development is high-risk — a bad bootloader flash requires SWD to recover — and it duplicates what the A/B scheme achieves in the application.
- **SWD from the Jetson** (2 wires: SWDIO/SWCLK from Jetson GPIO to the MCU's SWD pads, driven by OpenOCD's `linuxgpiod` bitbang driver): the most robust option — it can recover a fully bricked MCU, and while the core is halted for flashing the GPIO output latches retain their state, so `EFUSE_EN` stays high during programming. Requires two traces/wires that don't exist on the current board; worth adding on the next PCB revision as a recovery path even if I2C A/B update is the everyday mechanism.
