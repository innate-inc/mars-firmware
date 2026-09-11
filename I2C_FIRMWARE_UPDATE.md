# MCU Firmware Update over I2C — Protocol & Test Plan (R7.1)

This document is for the **Jetson-side agent**. It describes how to update the
MCU (Adafruit ItsyBitsy M4 / SAMD51, I2C follower at `0x42`) over the existing
I2C bus, with no USB cable and no bootloader entry.

## 1. How it works (and why it's safe)

The SAMD51 has two independent 256 KB flash banks. The running firmware stays
untouched in the **active bank** while the new image is streamed into the
**inactive bank** (always mapped at MCU address `0x40000`). Flash on this part
is read-while-write across banks, so the robot remains fully operational during
the transfer — motors, battery protection, and all I2C commands keep working.

Sequence: `BEGIN` erases the staged region and the MCU **copies its own UF2
bootloader** into the staged bank (so address 0 is always valid after a swap —
the Jetson never handles the bootloader). `DATA` chunks stream the application
image. `COMMIT` makes the MCU CRC32-verify the staged image. `APPLY` issues the
NVMCTRL bank-swap-and-reset. The old firmware remains intact in the now-inactive
bank; `APPLY` with the rollback sub-command swaps back to it.

Nothing irreversible happens until `APPLY`, and `APPLY` refuses to run unless
the image passed CRC verification **and** the staged bank's bootloader and
application vector tables look sane (stack pointer in RAM, thumb reset vector
in flash). A failed or abandoned transfer leaves the running firmware untouched.

**The one caveat — power blip at APPLY:** during the ~100–200 ms swap-reset,
the MCU's pins tristate and the e-fuse enable line (which keeps the Jetson
powered) may drop, depending on the board's hold-up. **Assume the Jetson may
power-cycle at APPLY.** This does not endanger the update — the MCU completes
the swap autonomously and re-asserts the e-fuse first thing in `setup()` — but
sync your filesystem and be ready to reconnect. Stages 1–2 of the test plan
carry no such risk.

## 2. What image to send

The **application binary only**, compiled for this board (linked at `0x4000`):
the `main-firmware.ino.bin` produced by
`arduino-cli compile --fqbn adafruit:samd:adafruit_itsybitsy_m4 --export-binaries`.
Never send a `.hex`, an `.elf`, a UF2, or a binary built for another board.
Max size: 245,760 bytes.

For this test round the file is `main-firmware.ino.bin`
(66,624 bytes, CRC32 `0x3B9A2D73` — recompute with `binascii.crc32(data)` and
use your computed values; don't hardcode these).

## 3. Protocol

Transport is unchanged from the base protocol: command = I2C write of a
register byte `0x00` + frame; response = I2C read of 8 bytes (all-zeros = no
response pending; poll). CRC8-MAXIM (poly `0x8C` reflected, init 0) over all
frame bytes except the CRC byte itself. Multi-byte fields big-endian.

All commands below use the standard 8-byte frame **except `0x0B` DATA**, which
is variable-length. Because DATA frames exceed the 32-byte SMBus limit, use raw
I2C transfers (`i2c_msg` / `i2c_rdwr` in smbus2), not `write_i2c_block_data`.

### `0x0A` FWUP_BEGIN → `0x8A`

Start a session. MCU erases the staged region and copies its bootloader across.
**This takes up to ~3 s** (the MCU main loop is busy; motors hold their last
command — run updates with the robot stationary). Poll for the response with a
generous timeout.

Command: byte1–4 = `uint32` application image size in bytes. Bytes 5–6 = 0.

Response: byte1 = status (`0x00` ok, `0x01` bad size), byte2 = chunk size
(128), byte3 = active bank (0/1 — note it, it should flip after APPLY),
bytes4–5 = `uint16` total chunk count the MCU expects.

### `0x0B` FWUP_DATA → `0x8B` (variable-length frame)

One chunk per I2C write: `[0x0B][seq_hi][seq_lo][len][data × len][CRC8]`
(CRC8 over everything before it). `seq` counts from 0. Every chunk must be
exactly 128 bytes except the final one (the remainder). Chunk `seq` is written
at application offset `seq × 128`.

Strict lockstep: **write one chunk, poll-read its ack, then send the next.**

Ack: byte1 = status (`0x00` ok, `0x01` bad frame/CRC — retry same chunk,
`0x02` out-of-order — resync to expected seq, `0x03` no session,
`0x04` range/length error — session dead, re-BEGIN), bytes2–3 = `uint16` next
expected seq, byte4 = session state (0 idle / 1 receiving / 2 verified /
3 error). A retransmit of the last-acked chunk is acked `0x00` and ignored, so
a lost ack is safe to retry.

### `0x0C` FWUP_STATUS → `0x8C`

Query anytime. Response: byte1 = state, bytes2–3 = expected seq, bytes4–5 =
total chunks, byte6 = (active bank << 4) | last error code.

### `0x0D` FWUP_COMMIT → `0x8D`

Command: byte1–4 = `uint32` CRC32 of the full application image (standard
zlib/`binascii.crc32`). Requires all chunks received.

Response: byte1 = status (`0x00` verified, `0x01` CRC mismatch, `0x02`
incomplete/no session), bytes2–5 = the CRC32 the MCU computed from flash (for
debugging mismatches). Verification reads back the actual flash contents.

### `0x0E` FWUP_APPLY → `0x8E`

Command: byte1 = sub-command (`0x01` = apply verified staged image, `0x02` =
rollback — swap to whatever firmware is in the inactive bank), byte2 = `0xB5`,
byte3 = `0x77` (magic; refused without it).

Response: byte1 = status (`0x00` accepted — **MCU swaps banks and resets
~150 ms after this ack**, `0x01` not verified, `0x02` bad magic, `0x03` target
bank failed the vector sanity check — swap refused, `0x04` unknown sub),
byte3 = active bank at time of command.

Read the ack immediately after writing; then expect the MCU (and possibly the
Jetson itself — see §1) to reboot. Rollback (`0x02`) needs no session, but is
only safe after at least one successful update has left real firmware in the
inactive bank; the sanity check enforces this.

### `0x0F` FWUP_ABORT → `0x8F`

Clears the session state. The staged bank contents are irrelevant once
abandoned. Response: byte1 = 0.

## 4. Reference updater (Python / smbus2)

```python
import time, binascii
from smbus2 import SMBus, i2c_msg

MCU, CHUNK = 0x42, 128

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc

def frame8(cmd, data=b""):
    f = bytes([cmd]) + data.ljust(6, b"\x00")
    return f + bytes([crc8(f)])

def send(bus, payload):
    bus.i2c_rdwr(i2c_msg.write(MCU, bytes([0x00]) + payload))

def read_resp(bus, expect_id, timeout=0.5):
    t0 = time.time()
    while time.time() - t0 < timeout:
        m = i2c_msg.read(MCU, 8)
        bus.i2c_rdwr(m)
        r = bytes(m)
        if r != bytes(8):
            assert crc8(r[:7]) == r[7], f"resp CRC bad: {r.hex()}"
            assert r[0] == expect_id, f"unexpected resp: {r.hex()}"
            return r
        time.sleep(0.005)
    raise TimeoutError(f"no response for 0x{expect_id:02X}")

def dfu_update(bus, image: bytes, apply=False):
    read_resp_drain(bus)                       # optional: one read to drain stale resp
    send(bus, frame8(0x0A, len(image).to_bytes(4, "big")))
    r = read_resp(bus, 0x8A, timeout=5.0)      # BEGIN: erase + bootloader copy
    assert r[1] == 0, f"BEGIN failed: {r.hex()}"
    bank_before = r[3]

    total = (len(image) + CHUNK - 1) // CHUNK
    for seq in range(total):
        data = image[seq * CHUNK:(seq + 1) * CHUNK]
        hdr = bytes([0x0B, seq >> 8, seq & 0xFF, len(data)]) + data
        for attempt in range(3):
            send(bus, hdr + bytes([crc8(hdr)]))
            r = read_resp(bus, 0x8B)
            if r[1] == 0x00:
                break
            assert r[1] in (0x01, 0x02), f"chunk {seq} fatal: {r.hex()}"
        else:
            raise IOError(f"chunk {seq} failed after retries")

    crc32 = binascii.crc32(image) & 0xFFFFFFFF
    send(bus, frame8(0x0D, crc32.to_bytes(4, "big")))
    r = read_resp(bus, 0x8D, timeout=2.0)
    assert r[1] == 0, f"COMMIT failed, MCU computed {r[2:6].hex()}: {r.hex()}"
    print(f"verified OK ({len(image)} bytes, crc32 {crc32:08x}), bank {bank_before} active")

    if apply:
        send(bus, frame8(0x0E, bytes([0x01, 0xB5, 0x77])))
        r = read_resp(bus, 0x8E)
        assert r[1] == 0, f"APPLY refused: {r.hex()}"
        print("APPLY accepted - MCU swaps banks and resets in ~150 ms "
              "(Jetson may power-cycle; see doc)")

def read_resp_drain(bus):
    m = i2c_msg.read(MCU, 8)
    bus.i2c_rdwr(m)
```

## 5. Test plan (run in order)

**Stage 0 — baseline (safe):** `0x09` version query → expect `7,1`.
`0x0C` FWUP_STATUS → state 0 (idle); record the active bank from byte6 >> 4.

**Stage 1 — session smoke test (safe):** BEGIN with size 256 → expect ack
`0x00`, chunk size 128, total chunks 2. Then ABORT → STATUS shows state 0.
Nothing running is affected.

**Stage 2 — full transfer + verify (safe, no reboot):** run `dfu_update(bus,
image, apply=False)` with the provided `main-firmware.ino.bin`. Expect chunk
acks all `0x00` (~520 chunks, several seconds) and COMMIT verified. This proves
the entire pipeline except the swap, and changes nothing about the running
firmware. If interrupted, just start over with BEGIN.

**Stage 3 — the swap (MCU reboots; Jetson may power-cycle):** prepare for a
possible Jetson reboot (§1), then either continue with `apply=True` (state is
still VERIFIED after stage 2) by sending the APPLY frame, or rerun
`dfu_update(bus, image, apply=True)`. After the MCU comes back (~1–2 s, longer
if the Jetson itself rebooted):
- `0x09` version → still `7,1` (we flashed the same firmware to itself)
- `0x0C` STATUS → **active bank flipped** vs stage 0 — this is the proof the
  swap executed and the MCU is genuinely running from the other bank.

**Stage 4 — rollback (optional, same reboot caveat):** APPLY with sub `0x02`
→ bank flips back. Both banks hold identical firmware after this test, so
behavior is unchanged either way.

Report the bank values and any non-zero ack at each stage.

## 6. Failure & recovery

- Anything failing **before APPLY** leaves the running firmware untouched:
  ABORT and retry from BEGIN.
- New firmware misbehaves **after APPLY** but I2C still works: APPLY/rollback
  (`0x02`) swaps back to the previous firmware.
- New firmware dead on arrival (no I2C): USB recovery — double-tap the MCU
  reset button for the UF2 bootloader (the staged bootloader copy makes this
  work from either bank) and flash over USB.
- The bank swap persists across power cycles; a power cut mid-transfer is
  harmless (the swap only happens at APPLY, atomically in hardware).
- Run updates with the robot stationary: BEGIN blocks the MCU's control loop
  for up to ~3 s, and motor commands are frozen at their last value during it.
