// src/FirmwareUpdate.cpp
#include "FirmwareUpdate.h"
#include "Utils.h"

// ---------------------------------------------------------------------
// Flash geometry (SAMD51G19A: 512 KB, two 256 KB banks, 8 KB erase blocks)
#define FLASH_BANK_SIZE 0x40000UL          // 256 KB per bank
#define STAGED_BANK_BASE 0x40000UL         // Inactive bank is always mapped here
#define BOOTLOADER_SIZE 0x4000UL           // 16 KB UF2 bootloader
#define APP_OFFSET BOOTLOADER_SIZE         // Application runs at 0x4000
#define DFU_MAX_APP_SIZE (FLASH_BANK_SIZE - BOOTLOADER_SIZE)
#define FLASH_BLOCK_SIZE 8192UL            // NVMCTRL erase block
#define FLASH_QW_SIZE 16UL                 // NVMCTRL quad-word write

// Response IDs (mirror main-firmware.ino definitions)
#define RESP_FWUP_BEGIN 0x8A
#define RESP_FWUP_DATA 0x8B
#define RESP_FWUP_STATUS 0x8C
#define RESP_FWUP_COMMIT 0x8D
#define RESP_FWUP_APPLY 0x8E
#define RESP_FWUP_ABORT 0x8F

#define DFU_APPLY_MAGIC1 0xB5
#define DFU_APPLY_MAGIC2 0x77

// ---------------------------------------------------------------------
// Session state
static DfuState dfu_state = DFU_IDLE;
static uint8_t dfu_error = 0;        // Last error code (see .md spec)
static uint32_t dfu_app_size = 0;    // Expected application image size
static uint16_t dfu_total_chunks = 0;
static uint16_t dfu_expected_seq = 0;
static bool apply_pending = false;
static unsigned long apply_time = 0;

// ---------------------------------------------------------------------
// NVMCTRL helpers (manual write mode, operating on the inactive bank only)

static void nvmWait() {
  while (!NVMCTRL->STATUS.bit.READY) {}
}

static void nvmCommand(uint16_t cmd) {
  nvmWait();
  NVMCTRL->INTFLAG.reg = NVMCTRL_INTFLAG_DONE;
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | cmd;
  nvmWait();
}

static void nvmInitForDfu() {
  // Manual write mode; disable both NVM caches so verification reads see
  // freshly written data
  NVMCTRL->CTRLA.bit.WMODE = NVMCTRL_CTRLA_WMODE_MAN_Val;
  NVMCTRL->CTRLA.bit.CACHEDIS0 = 1;
  NVMCTRL->CTRLA.bit.CACHEDIS1 = 1;
}

static void nvmEraseBlock(uint32_t addr) {
  nvmWait();
  NVMCTRL->ADDR.reg = addr;
  nvmCommand(NVMCTRL_CTRLB_CMD_EB);
}

// Write 16 bytes (one quad word) at a 16-byte-aligned flash address
static void nvmWriteQuadWord(uint32_t addr, const uint8_t *data) {
  uint32_t words[4];
  memcpy(words, data, FLASH_QW_SIZE);

  nvmCommand(NVMCTRL_CTRLB_CMD_PBC); // Clear page buffer

  volatile uint32_t *dst = (volatile uint32_t *)addr;
  for (int i = 0; i < 4; i++) {
    dst[i] = words[i];
  }

  nvmWait();
  NVMCTRL->ADDR.reg = addr;
  nvmCommand(NVMCTRL_CTRLB_CMD_WQW);
}

// Write an arbitrary-length buffer (16-byte-aligned address, padded with 0xFF)
static void nvmWriteBuffer(uint32_t addr, const uint8_t *data, uint32_t len) {
  uint8_t qw[FLASH_QW_SIZE];
  uint32_t off = 0;
  while (off < len) {
    uint32_t n = min((uint32_t)FLASH_QW_SIZE, len - off);
    memset(qw, 0xFF, sizeof(qw));
    memcpy(qw, data + off, n);
    nvmWriteQuadWord(addr + off, qw);
    off += FLASH_QW_SIZE;
  }
}

// ---------------------------------------------------------------------
// CRC32 (standard reflected 0xEDB88320, init/final 0xFFFFFFFF — matches
// Python's binascii.crc32 / zlib.crc32)
static uint32_t crc32Flash(uint32_t addr, uint32_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  const uint8_t *p = (const uint8_t *)addr;
  for (uint32_t i = 0; i < len; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1)));
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

// ---------------------------------------------------------------------
// Sanity check a bank image before swapping to it: both the bootloader
// vector table (bank base) and the application vector table (bank base +
// 0x4000) must have a stack pointer in RAM and a thumb reset vector within
// flash. Guards against swapping to a blank or half-written bank, which
// would brick the board beyond USB recovery.
static bool bankImageLooksValid(uint32_t bankBase) {
  const uint32_t offsets[2] = {0, APP_OFFSET};
  for (int i = 0; i < 2; i++) {
    uint32_t sp = *(const uint32_t *)(bankBase + offsets[i]);
    uint32_t rv = *(const uint32_t *)(bankBase + offsets[i] + 4);
    bool spOk = (sp >= 0x20000000UL) && (sp <= 0x20040000UL); // 192 KB RAM + margin
    bool rvOk = (rv & 1) && ((rv & ~1UL) < 2 * FLASH_BANK_SIZE);
    if (!spOk || !rvOk) return false;
  }
  return true;
}

static void finishResponse(uint8_t *resp) {
  resp[7] = calculateCRC8Maxim(resp, 7);
}

static uint8_t activeBank() {
  return NVMCTRL->STATUS.bit.AFIRST ? 0 : 1;
}

// ---------------------------------------------------------------------
// Command handlers

void dfuHandleBegin(const uint8_t *cmd, uint8_t *resp) {
  uint32_t size = ((uint32_t)cmd[1] << 24) | ((uint32_t)cmd[2] << 16) |
                  ((uint32_t)cmd[3] << 8) | cmd[4];

  memset(resp, 0, 8);
  resp[0] = RESP_FWUP_BEGIN;
  resp[2] = DFU_CHUNK_SIZE;
  resp[3] = activeBank();

  if (size == 0 || size > DFU_MAX_APP_SIZE) {
    resp[1] = 0x01; // Bad size
    dfu_state = DFU_ERROR;
    dfu_error = 0x01;
    finishResponse(resp);
    return;
  }

  Serial.print("DFU begin - app size: ");
  Serial.println(size);

  nvmInitForDfu();

  // Erase bootloader copy region + application region (rounded up to blocks)
  uint32_t total = APP_OFFSET + size;
  uint32_t blocks = (total + FLASH_BLOCK_SIZE - 1) / FLASH_BLOCK_SIZE;
  for (uint32_t b = 0; b < blocks; b++) {
    nvmEraseBlock(STAGED_BANK_BASE + b * FLASH_BLOCK_SIZE);
  }

  // Copy the running bootloader into the staged bank so address 0 remains
  // valid after the swap
  nvmWriteBuffer(STAGED_BANK_BASE, (const uint8_t *)0x0UL, BOOTLOADER_SIZE);

  dfu_app_size = size;
  dfu_total_chunks = (uint16_t)((size + DFU_CHUNK_SIZE - 1) / DFU_CHUNK_SIZE);
  dfu_expected_seq = 0;
  dfu_state = DFU_RECEIVING;
  dfu_error = 0;
  apply_pending = false;

  resp[1] = 0x00; // OK
  resp[4] = (dfu_total_chunks >> 8) & 0xFF;
  resp[5] = dfu_total_chunks & 0xFF;
  finishResponse(resp);
  Serial.println("DFU staged bank erased, bootloader copied");
}

void dfuHandleDataChunk(const uint8_t *frame, uint16_t frameLen, uint8_t *resp) {
  memset(resp, 0, 8);
  resp[0] = RESP_FWUP_DATA;

  uint8_t status = 0x00;

  if (dfu_state != DFU_RECEIVING) {
    status = 0x03; // No session in progress
  } else if (frameLen < 6 || frameLen > DFU_MAX_FRAME) {
    status = 0x01; // Malformed frame
  } else {
    uint16_t seq = ((uint16_t)frame[1] << 8) | frame[2];
    uint8_t len = frame[3];

    if ((uint16_t)(4 + len + 1) != frameLen) {
      status = 0x01; // Length mismatch
    } else if (calculateCRC8Maxim(frame, 4 + len) != frame[4 + len]) {
      status = 0x01; // CRC error
    } else if (seq == (uint16_t)(dfu_expected_seq - 1) && dfu_expected_seq > 0) {
      status = 0x00; // Duplicate of last chunk (leader retry) - ack, don't rewrite
    } else if (seq != dfu_expected_seq) {
      status = 0x02; // Out-of-order chunk
    } else {
      uint32_t offset = (uint32_t)seq * DFU_CHUNK_SIZE;
      bool isLast = (seq == dfu_total_chunks - 1);
      uint8_t expectLen = isLast ? (uint8_t)(dfu_app_size - offset) : DFU_CHUNK_SIZE;

      if (offset + len > dfu_app_size || len != expectLen) {
        status = 0x04; // Range/length error
      } else {
        nvmWriteBuffer(STAGED_BANK_BASE + APP_OFFSET + offset, &frame[4], len);
        dfu_expected_seq++;
      }
    }
  }

  if (status != 0x00 && status != 0x02 && dfu_state == DFU_RECEIVING && status != 0x01) {
    // 0x03/0x04 are session-fatal; 0x01 (bad frame) and 0x02 (bad seq) are
    // retryable without killing the session
    dfu_state = DFU_ERROR;
    dfu_error = status;
  }

  resp[1] = status;
  resp[2] = (dfu_expected_seq >> 8) & 0xFF;
  resp[3] = dfu_expected_seq & 0xFF;
  resp[4] = dfu_state;
  finishResponse(resp);
}

void dfuHandleStatus(uint8_t *resp) {
  memset(resp, 0, 8);
  resp[0] = RESP_FWUP_STATUS;
  resp[1] = dfu_state;
  resp[2] = (dfu_expected_seq >> 8) & 0xFF;
  resp[3] = dfu_expected_seq & 0xFF;
  resp[4] = (dfu_total_chunks >> 8) & 0xFF;
  resp[5] = dfu_total_chunks & 0xFF;
  resp[6] = (activeBank() << 4) | (dfu_error & 0x0F);
  finishResponse(resp);
}

void dfuHandleCommit(const uint8_t *cmd, uint8_t *resp) {
  uint32_t expected_crc = ((uint32_t)cmd[1] << 24) | ((uint32_t)cmd[2] << 16) |
                          ((uint32_t)cmd[3] << 8) | cmd[4];

  memset(resp, 0, 8);
  resp[0] = RESP_FWUP_COMMIT;

  if (dfu_state != DFU_RECEIVING || dfu_expected_seq != dfu_total_chunks) {
    resp[1] = 0x02; // No session / transfer incomplete
    finishResponse(resp);
    return;
  }

  uint32_t actual = crc32Flash(STAGED_BANK_BASE + APP_OFFSET, dfu_app_size);
  resp[2] = (actual >> 24) & 0xFF;
  resp[3] = (actual >> 16) & 0xFF;
  resp[4] = (actual >> 8) & 0xFF;
  resp[5] = actual & 0xFF;

  if (actual == expected_crc) {
    dfu_state = DFU_VERIFIED;
    resp[1] = 0x00;
    Serial.println("DFU image verified");
  } else {
    dfu_state = DFU_ERROR;
    dfu_error = 0x05;
    resp[1] = 0x01; // CRC mismatch
    Serial.println("DFU CRC mismatch");
  }
  finishResponse(resp);
}

void dfuHandleApply(const uint8_t *cmd, uint8_t *resp) {
  uint8_t sub = cmd[1];

  memset(resp, 0, 8);
  resp[0] = RESP_FWUP_APPLY;
  resp[3] = activeBank();

  if (cmd[2] != DFU_APPLY_MAGIC1 || cmd[3] != DFU_APPLY_MAGIC2) {
    resp[1] = 0x02; // Bad magic
    finishResponse(resp);
    return;
  }

  if (sub == 0x01) { // Apply staged image
    if (dfu_state != DFU_VERIFIED) {
      resp[1] = 0x01; // Not verified
      finishResponse(resp);
      return;
    }
  } else if (sub == 0x02) { // Rollback: swap to whatever is in the other bank
    // No verified session required; sanity check below is the gate
  } else {
    resp[1] = 0x04; // Unknown sub-command
    finishResponse(resp);
    return;
  }

  if (!bankImageLooksValid(STAGED_BANK_BASE)) {
    resp[1] = 0x03; // Target bank fails sanity check - refusing to swap
    finishResponse(resp);
    return;
  }

  resp[1] = 0x00; // Accepted: swapping banks and resetting in ~150 ms
  finishResponse(resp);

  apply_pending = true;
  apply_time = millis();
  Serial.println(sub == 0x02 ? "DFU rollback pending" : "DFU apply pending");
}

void dfuHandleAbort(uint8_t *resp) {
  dfu_state = DFU_IDLE;
  dfu_error = 0;
  dfu_expected_seq = 0;
  dfu_total_chunks = 0;
  dfu_app_size = 0;
  apply_pending = false;

  memset(resp, 0, 8);
  resp[0] = RESP_FWUP_ABORT;
  finishResponse(resp);
  Serial.println("DFU aborted");
}

void dfuUpdate() {
  if (!apply_pending) return;
  if (millis() - apply_time < 150) return; // Let the leader read the ack first

  Serial.println("DFU: swapping banks and resetting NOW");
  Serial.flush();
  delay(10);

  __disable_irq();
  nvmWait();
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMDEX_KEY | NVMCTRL_CTRLB_CMD_BKSWRST;
  while (true) {} // Device resets
}
