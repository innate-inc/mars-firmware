// src/FirmwareUpdate.h
//
// I2C firmware update (DFU) for the SAMD51 using A/B flash bank swap.
//
// The SAMD51 has two 256 KB flash banks; the inactive bank is always mapped
// at 0x40000 and can be erased/written while code runs from the active bank
// (read-while-write). The update flow:
//   1. BEGIN: erase the staged region and copy the running bootloader
//      (0x0000-0x3FFF) into the staged bank so address 0 is valid post-swap.
//   2. DATA chunks: application image (linked at 0x4000) streamed 128 bytes
//      at a time, written at 0x44000+ in the staged bank.
//   3. COMMIT: CRC32 verify of the staged application against the sender's.
//   4. APPLY: NVMCTRL bank swap + reset (BKSWRST). The old firmware stays in
//      the now-inactive bank; APPLY with the rollback sub-command swaps back.
//
// Safety gates: APPLY requires a passed COMMIT, magic bytes in the command,
// and a sanity check of the staged bank's stack-pointer/reset vectors (both
// bootloader and application) before the swap is issued.

#ifndef FIRMWAREUPDATE_H
#define FIRMWAREUPDATE_H

#include <Arduino.h>

// DFU protocol constants (shared with the I2C command definitions)
#define DFU_CHUNK_SIZE 128
#define DFU_MAX_FRAME (4 + DFU_CHUNK_SIZE + 1) // header + data + CRC8

// DFU session states (reported in status/ack responses)
enum DfuState : uint8_t {
  DFU_IDLE = 0,
  DFU_RECEIVING = 1,
  DFU_VERIFIED = 2,
  DFU_ERROR = 3,
};

// Fills resp[0..7] (including CRC8) for each handled command.
// cmd points at the received 8-byte command frame (CRC already checked).
void dfuHandleBegin(const uint8_t *cmd, uint8_t *resp);
void dfuHandleStatus(uint8_t *resp);
void dfuHandleCommit(const uint8_t *cmd, uint8_t *resp);
void dfuHandleApply(const uint8_t *cmd, uint8_t *resp);
void dfuHandleAbort(uint8_t *resp);

// Process a raw DATA chunk frame (variable length, CRC checked inside).
// Fills resp[0..7] with the ack frame.
void dfuHandleDataChunk(const uint8_t *frame, uint16_t frameLen, uint8_t *resp);

// Call from loop(): executes a pending bank swap ~150 ms after APPLY was
// acknowledged, giving the leader time to read the ack before reset.
void dfuUpdate();

#endif // FIRMWAREUPDATE_H
