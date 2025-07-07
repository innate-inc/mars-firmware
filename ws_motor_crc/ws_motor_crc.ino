#include <Arduino.h>
#include "wiring_private.h"    // for pinPeripheral()

// ——— Motor UART on SERCOM4 ———
// RX = A3, TX = A2
Uart SerialLeft(&sercom4,
               PIN_A3,      // RX → PAD1
               PIN_A2,      // TX → PAD0
               SERCOM_RX_PAD_1,
               UART_TX_PAD_0);

// Maxim/Dallas CRC-8 (poly=0x31 reflected as 0x8C)
uint8_t crc8_maxim(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x01)
        crc = (crc >> 1) ^ 0x8C;
      else
        crc >>= 1;
    }
  }
  return crc;
}

// Build and send a velocity command (rawVel = RPM × 10)
void sendVelocity(int16_t rawVel) {
  uint8_t cmd[10];
  cmd[0] = 0x01;                    // ID
  cmd[1] = 0x64;                    // 0x64 = velocity-loop command
  cmd[2] = (rawVel >> 8) & 0xFF;    // speed high byte
  cmd[3] = rawVel & 0xFF;           // speed low byte
  cmd[4] = 0x01;                    // feedback 1: speed
  cmd[5] = 0x02;                    // feedback 2: current
  cmd[6] = 0x00;                    // accel time (default)
  cmd[7] = 0x00;                    // brake (0=no brake)
  cmd[8] = 0x00;                    // reserved
  cmd[9] = crc8_maxim(cmd, 9);      // CRC over bytes 0–8

  Serial.print(F("Sending → "));
  for (int i = 0; i < 10; i++) {
    if (cmd[i] < 0x10) Serial.print('0');
    Serial.print(cmd[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
  SerialLeft.write(cmd, 10);
}

int readPacket(uint8_t *buf) {
  int idx = 0;
  unsigned long t0 = millis();
  while (idx < 10 && millis() - t0 < 200) {
    if (SerialLeft.available()) {
      buf[idx++] = SerialLeft.read();
    }
  }
  return idx;
}

void printHex(const uint8_t *data, int len) {
  for (int i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // --- Initialize SERCOM4 UART for Motor 1 ---
  SerialLeft.begin(115200);
  pinPeripheral(PIN_A2, PIO_SERCOM_ALT);  // A2 → SERCOM4 TX pad 0
  pinPeripheral(PIN_A3, PIO_SERCOM_ALT);  // A3 → SERCOM4 RX pad 1
  delay(100);

  // Switch into velocity-loop mode
  const uint8_t velLoopCmd[10] = {
    0x01,0xA0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0xE4
  };
  Serial.print(F("Switch to velocity-loop: "));
  printHex(velLoopCmd, 10);
  Serial.println();
  SerialLeft.write(velLoopCmd, 10);
  delay(200);

  Serial.println(F("Setup complete. Will send 100 RPM…"));
}

void loop() {
  // Send 100 RPM → raw value = 100 × 10 = 1000
  sendVelocity(100 * 10);
  delay(1000);

  // Read back and display
  uint8_t buf[10];
  int len = readPacket(buf);

  Serial.print(F("Received: "));
  if (len > 0) {
    printHex(buf, len);
    Serial.print(F("  (")); Serial.print(len); Serial.println(F(" bytes)"));

    if (len >= 6) {
      int16_t rawVel     = (int16_t(buf[2]) << 8) | buf[3];
      int16_t rawCurrent = (int16_t(buf[4]) << 8) | buf[5];
      float velRpm = rawVel / 10.0;
      float currA  = rawCurrent / 100.0;
      Serial.print(F("  → Vel: "));
      Serial.print(velRpm, 1);
      Serial.print(F(" RPM, Current: "));
      Serial.print(currA, 2);
      Serial.println(F(" A"));
    }
  } else {
    Serial.println(F("<no data>"));
  }

  Serial.println();
  delay(1000);
}

// ——— SERCOM4 interrupt handlers ———
void SERCOM4_0_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_1_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_2_Handler() { SerialLeft.IrqHandler(); }
void SERCOM4_3_Handler() { SerialLeft.IrqHandler(); }


