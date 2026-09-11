# Micro Controller Protocol

## 1. Overview

This document covers two things:

1. **The current I2C protocol (Part A)** — the complete, authoritative reference for how the Jetson Orin Nano talks to the MCU (ItsyBitsy M4 Express) as of firmware **R7.1**. This section is self-contained and is what a client implementation on the Jetson should be written against.
2. **The legacy R6 UART specification (Part B)** — the original spec the protocol payloads were derived from, kept for historical reference. The current firmware does not use UART, the SOM marker, or the 10-byte physical framing described there.

---

# Part A — I2C Protocol Reference (Current Firmware, R7.1)

## A.1. Transport

- **Bus:** I2C. The MCU is a **follower (slave)** at address **`0x42`**. The Jetson is the bus leader.
- **Frame:** Every command and every response is exactly **8 bytes**:

  | Byte | Content |
  | --- | --- |
  | 0 | Command ID (Jetson → MCU) or Response ID (MCU → Jetson) |
  | 1–6 | Data fields (layouts per message type below) |
  | 7 | **CRC-8/MAXIM** computed over bytes 0–6 (polynomial `0x8C` reflected, initial value `0x00`) |

- **Sending a command:** perform an I2C **write of 9 bytes** to address `0x42`: a register byte of **`0x00`** followed by the 8-byte command frame. (With Linux smbus this is `write_i2c_block_data(0x42, 0x00, frame)` where `frame` is the 8-byte list.)
- **Reading the response:** perform an I2C **read of 8 bytes** from address `0x42`. The MCU holds a single response buffer, filled when it processes a command:
  - Commands are processed in the MCU main loop, not in the I2C interrupt — allow a few milliseconds between write and read.
  - If no response is pending (nothing was commanded, the command failed CRC, or the response was already read), the read returns **8 bytes of `0x00`**.
  - The buffer is one deep: each new command overwrites the previous response. Use a strict write → read cycle per command.
- **CRC checking:** the MCU silently discards any command whose CRC does not match (the subsequent read returns zeros). The Jetson should likewise verify the CRC on every response and treat all-zeros as "no response".
- **Multi-byte fields** are **big-endian** (high byte first) unless stated otherwise. Signed fields are two's-complement `int16`.

Reference CRC implementation (Python):

```python
def crc8_maxim(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc

def make_frame(cmd_id: int, data: bytes = b"") -> list[int]:
    frame = bytes([cmd_id]) + data.ljust(6, b"\x00")
    return list(frame + bytes([crc8_maxim(frame)]))
```

## A.2. Command Set Summary

| Cmd ID | Name | Response ID | Since | Purpose |
| --- | --- | --- | --- | --- |
| `0x01` | Move | `0x81` | R6 | Set speed/turn; returns position |
| `0x03` | Status | `0x83` | R6 | Battery, faults, system flags |
| `0x04` | Calibrate | `0x84` | R6 | Zero position and heading |
| `0x05` | Reset | `0x85` | R6 | 1 s power pulse on HSSW and/or e-fuse |
| `0x06` | Buzzer | `0x86` | **R7.1** | Buzzer control: manual on/off, snooze, enable/disable |
| `0x07` | HSSW | `0x87` | **R7.1** | High side switch on/off toggle + current sense |
| `0x08` | IMU | `0x88` | **R7.1** | Full orientation: roll/pitch/yaw |
| `0x09` | Version | `0x89` | **R7.1** | Firmware version (capability detection) |

**Backwards compatibility:** the R7.1 additions are strictly additive. Frames for `0x01`–`0x05` are byte-identical to older firmware. Firmware older than R7.1 ignores unknown command IDs (the read returns zeros) — so a zero response to `CMD_VERSION` means "pre-R7.1 firmware, base command set only".

## A.3. Commands and Responses

### `0x01` Move → `0x81`

Command:

| Byte | Field | Description |
| --- | --- | --- |
| 0 | `0x01` | |
| 1–2 | Speed | `int16`, forward speed command |
| 3–4 | Turn | `int16`, turn rate command |
| 5–6 | Reserved | `0x00` |
| 7 | CRC | |

The commanded speed/turn is latched and re-applied continuously by the MCU until the next Move command.

Response:

| Byte | Field | Description |
| --- | --- | --- |
| 0 | `0x81` | |
| 1–2 | X | `int16`, position X in **cm** |
| 3–4 | Y | `int16`, position Y in **cm** |
| 5–6 | Theta | `int16`, heading in **radians × 100** |
| 7 | CRC | |

### `0x03` Status → `0x83`

Command: byte 0 = `0x03`, bytes 1–6 = `0x00`, byte 7 = CRC.

Response:

| Byte | Field | Description |
| --- | --- | --- |
| 0 | `0x83` | |
| 1–2 | Battery Voltage | `uint16`, volts × 100 (e.g., 1234 = 12.34 V) |
| 3 | Left Motor Temp | **R7.1+**: `uint8`, winding temperature in °C from DDSM210 feedback; `0xFF` = no fresh data (motor silent > 2 s). Sent as `0x00` by older firmware. |
| 4 | Right Motor Temp | **R7.1+**: as byte 3, right motor. |
| 5 | Fault Code | Fault code / bitfield |
| 6 | System Flags | **R7.1+**: bitfield per [A.4](#a4-system-flags-bitfield); always `0x00` on older firmware |
| 7 | CRC | |

### `0x04` Calibrate → `0x84`

Command: byte 0 = `0x04`, bytes 1–6 = `0x00`, byte 7 = CRC.

Resets position to (0, 0) and heading to 0. **The MCU blocks for ~1 second** while the IMU settles; do not expect it to respond to other commands during that window.

Response: byte 1 = status (`0` = success, `1` = in progress, `2` = failure), bytes 2–6 = `0x00`.

### `0x05` Reset → `0x85`

Pulses the selected enable line(s) low for 1 second, then restores them high. Note: an e-fuse reset cuts the Jetson's own power (the Jetson sits behind the e-fuse), so the Jetson will not see the completion response for targets `0x02`/`0x03`.

Command: byte 1 = target: `0x01` = HSSW, `0x02` = e-fuse, `0x03` = both. Bytes 2–6 = `0x00`.

Response:

| Byte | Field | Description |
| --- | --- | --- |
| 0 | `0x85` | |
| 1 | Status | `0x00` = not accepted (e.g., low-voltage cutoff active, or invalid target), `0x01` = started, `0x02` = completed (sent after the pulse, HSSW-only resets) |
| 2 | Target | Target being reset (`0x00` if none) |
| 3–6 | Reserved | `0x00` |
| 7 | CRC | |

### `0x06` Buzzer → `0x86` (R7.1)

Command:

| Byte | Field | Description |
| --- | --- | --- |
| 0 | `0x06` | |
| 1 | Sub-command | `0x00` = Query, `0x01` = Manual ON, `0x02` = Manual OFF, `0x03` = Snooze low-battery warning, `0x04` = Cancel snooze, `0x05` = Enable low-battery warning, `0x06` = Disable low-battery warning |
| 2–3 | Snooze Duration | `uint16` seconds; only used with sub-command `0x03` (`0` = default 60 s) |
| 4–6 | Reserved | `0x00` |
| 7 | CRC | |

Response:

| Byte | Field | Description |
| --- | --- | --- |
| 0 | `0x86` | |
| 1 | System Flags | Bitfield per [A.4](#a4-system-flags-bitfield) |
| 2–3 | Snooze Remaining | `uint16` seconds left on the current snooze (`0` if not snoozed) |
| 4 | Ack | `0x00` = accepted, `0x01` = unknown sub-command |
| 5–6 | Reserved | `0x00` |
| 7 | CRC | |

Behavior:

- The low-battery warning pattern (3 × 1 s beeps, 10 s pause, repeat; triggers below 10.5 V) behaves like an interrupt: it overlays the manual buzzer state while active, and the buzzer returns to the manual state when it ends or is snoozed/disabled.
- Snooze suppresses only the warning pattern, auto-expires, and is cleared when the battery recovers above 11.0 V (hysteresis), so the next low-battery event beeps again.
- The constant beep in the low-voltage **cutoff** state (below 10.3 V) is not snoozable — the e-fuse has already dropped Jetson power at that point.

### `0x07` HSSW → `0x87` (R7.1)

Command: byte 1 = sub-command: `0x00` = Query, `0x01` = ON, `0x02` = OFF. Bytes 2–6 = `0x00`.

Response:

| Byte | Field | Description |
| --- | --- | --- |
| 0 | `0x87` | |
| 1 | Commanded State | `0x01` = ON commanded, `0x00` = OFF commanded |
| 2 | System Flags | Bitfield per [A.4](#a4-system-flags-bitfield) |
| 3–4 | Current Sense | `uint16`, raw 12-bit ADC reading of the HSSW current-sense pin (A0) |
| 5 | Actual Pin Level | `0x01` = enable pin high, `0x00` = low (differs from commanded during a reset pulse or cutoff) |
| 6 | Ack | `0x00` = accepted, `0x01` = ON rejected (low-voltage cutoff active), `0x02` = unknown sub-command |
| 7 | CRC | |

Behavior:

- ON is rejected while the low-voltage cutoff is latched.
- A Reset command (`0x05`) targeting the HSSW still pulses it; the pulse always ends with the HSSW commanded ON.
- The commanded state survives cutoff/recovery: on voltage recovery the HSSW is restored to the last commanded state.

### `0x08` IMU → `0x88` (R7.1)

Command: byte 0 = `0x08`, bytes 1–6 = `0x00`, byte 7 = CRC.

Response:

| Byte | Field | Description |
| --- | --- | --- |
| 0 | `0x88` | |
| 1–2 | Roll | `int16`, **milliradians** (rotation about X) |
| 3–4 | Pitch | `int16`, **milliradians** (rotation about Y) |
| 5–6 | Yaw | `int16`, **milliradians** (heading; calibration offset applied — the same angle as Theta in `0x81`, at 10× finer resolution) |
| 7 | CRC | |

All three fields read `0x7FFF` (32767) when the IMU is unavailable.

### `0x09` Version → `0x89` (R7.1)

Command: byte 0 = `0x09`, bytes 1–6 = `0x00`, byte 7 = CRC.

Response: byte 1 = major version, byte 2 = minor version (R7.1 firmware reports `7`, `1`), bytes 3–6 = `0x00`. An all-zero read after this command indicates pre-R7.1 firmware.

## A.4. System Flags Bitfield

Reported in Status byte 6, Buzzer response byte 1, and HSSW response byte 2:

| Bit | Mask | Meaning |
| --- | --- | --- |
| 0 | `0x01` | Buzzer pin currently driven high |
| 1 | `0x02` | Low-voltage warning latched (dipped below 10.5 V, not yet recovered above 11.0 V) |
| 2 | `0x04` | Low-voltage cutoff latched (dipped below 10.3 V — HSSW and e-fuse forced off) |
| 3 | `0x08` | Low-battery buzzer snoozed |
| 4 | `0x10` | Low-battery buzzer enabled (default 1) |
| 5 | `0x20` | HSSW commanded on |
| 6 | `0x40` | Buzzer manually commanded on |
| 7 | `0x80` | Reset pulse in progress |

## A.5. Jetson Client Example (Python, smbus2)

```python
import time
from smbus2 import SMBus, i2c_msg

MCU_ADDR = 0x42

def transact(bus: SMBus, cmd_id: int, data: bytes = b"") -> bytes:
    bus.write_i2c_block_data(MCU_ADDR, 0x00, make_frame(cmd_id, data))
    time.sleep(0.005)  # let the MCU main loop process the command
    read = i2c_msg.read(MCU_ADDR, 8)
    bus.i2c_rdwr(read)
    resp = bytes(read)
    if resp == bytes(8):
        return b""  # no response pending (or pre-R7.1 firmware for new commands)
    if crc8_maxim(resp[:7]) != resp[7]:
        raise IOError(f"response CRC mismatch: {resp.hex()}")
    return resp

with SMBus(7) as bus:  # use the I2C bus number wired to the MCU
    fw = transact(bus, 0x09)                          # version query
    print("firmware", f"{fw[1]}.{fw[2]}" if fw else "pre-R7.1")
    print("status  ", transact(bus, 0x03).hex())      # battery/faults/flags
    print("imu     ", transact(bus, 0x08).hex())      # roll/pitch/yaw (mrad)
    transact(bus, 0x06, bytes([0x03, 0x00, 0x78]))    # snooze buzzer 120 s
```

---

# Part B — Legacy R6 UART Specification (Historical)

This part describes the original **R6 protocol**, designed for a UART interface. The current firmware uses I2C (Part A); the payload structures here are the ancestors of the I2C frames. In the R6 spec, each transmitted message consists of:

- **2 bytes for the Start-of-Message (SOM) marker:** Fixed value 0x69 0x69, which delineates the start of a new message.
- **8 bytes of protocol message:** This includes 7 bytes of payload (ID and data fields) and 1 byte of CRC (calculated using CRC‑8/MAXIM over the payload bytes).

The overall physical message length is therefore **10 bytes**. **Byte offsets in Part B are offsets into this 10-byte UART message** (payload starts at byte 2); over I2C the same payloads start at byte 0 — see Part A.

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
    | 8 | System Flags | 1 byte | **R7.1 addition** (previously reserved, always `0x00`): system flags bitfield — see [Section A.4](#a4-system-flags-bitfield) in Part A. Firmware older than 7.1 always sends `0x00` here, so parsers that ignored this byte remain compatible. |
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

> **Update: implemented as of R7.1.** The application-level A/B bank-swap
> scheme described below is now in the firmware (`FirmwareUpdate.cpp`,
> commands `0x0A`–`0x0F`). The full protocol, a reference Python updater,
> and the test plan live in [I2C_FIRMWARE_UPDATE.md](I2C_FIRMWARE_UPDATE.md).

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
