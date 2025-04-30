// src/MotorControl.cpp
#include "MotorControl.h"

// Static array: index by sercom number
MotorControl* MotorControl::_instances[8] = { nullptr };

MotorControl::MotorControl(uint8_t sercomNum,
                           uint8_t rxPin, uint8_t txPin,
                           uint8_t rxPad, uint8_t txPad)
  : _serial(nullptr),
    _id(1), _lastSpeed(0), _lastRPM(0.0f), _newRPM(false), _bufIndex(0),
    _sercomNum(sercomNum)
{
  // register this instance
  if (sercomNum < 8) _instances[sercomNum] = this;

  // select the SERCOM peripheral
  SERCOM* sercomPtr = nullptr;
  if (sercomNum == 0) sercomPtr = &sercom0;
  else if (sercomNum == 4) sercomPtr = &sercom4;
  // add more cases if needed

  // allocate and configure Uart
  _serial = new Uart(sercomPtr, rxPin, txPin, 
                    static_cast<SercomRXPad>(rxPad), 
                    static_cast<SercomUartTXPad>(txPad));
  pinPeripheral(txPin, PIO_SERCOM_ALT);
  pinPeripheral(rxPin, PIO_SERCOM_ALT);
}

void MotorControl::begin(uint32_t baud) {
  _serial->begin(baud);
  delay(20);

  // set ID packet
  uint8_t cmdID[10] = { 0xAA, 0x55, 0x53, _id, 0,0,0,0,0, 0 };
  cmdID[9] = calculateCRC(cmdID, 9);
  for (int i = 0; i < 5; i++) {
    _serial->write(cmdID, 10);
    delay(20);
  }

  // set speed-loop mode
  uint8_t cmdMode[10] = { _id, 0xA0, 0x02, 0,0,0,0,0,0, 0 };
  cmdMode[9] = calculateCRC(cmdMode, 9);
  _serial->write(cmdMode, 10);
  delay(20);
}

void MotorControl::sendCommand(int16_t speed) {
  _lastSpeed = speed;
  uint8_t cmd[10] = { _id, 0x64,
                      uint8_t(speed >> 8), uint8_t(speed & 0xFF),
                      0,0, 0x01, 0, 0, 0 };
  cmd[9] = calculateCRC(cmd, 9);
  _serial->write(cmd, 10);
}

void MotorControl::maintain() {
  sendCommand(_lastSpeed);
}

void MotorControl::processFeedback() {
  while (_serial->available()) {
    uint8_t b = _serial->read();
    _buffer[_bufIndex++] = b;

    if (_bufIndex >= 10) {
      if (calculateCRC(_buffer, 9) == _buffer[9]) {
        int16_t sp = (int16_t(_buffer[2]) << 8) | _buffer[3];
        _lastRPM = sp / 10.0f;
        _newRPM = true;
      }
      // slide buffer
      memmove(_buffer, _buffer + 1, --_bufIndex);
    }
  }
}

bool MotorControl::available() {
  if (_newRPM) { _newRPM = false; return true; }
  return false;
}

float MotorControl::getRPM() const {
  return _lastRPM;
}

uint8_t MotorControl::calculateCRC(const uint8_t* data, int len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 1) ? (crc >> 1) ^ 0x8C : (crc >> 1);
  }
  return crc;
}

// C-linkage interrupt handlers
extern "C" {
  void SERCOM0_0_Handler() { if (MotorControl::_instances[0]) MotorControl::_instances[0]->_serial->IrqHandler(); }
  void SERCOM0_1_Handler() { if (MotorControl::_instances[0]) MotorControl::_instances[0]->_serial->IrqHandler(); }
  void SERCOM0_2_Handler() { if (MotorControl::_instances[0]) MotorControl::_instances[0]->_serial->IrqHandler(); }
  void SERCOM0_3_Handler() { if (MotorControl::_instances[0]) MotorControl::_instances[0]->_serial->IrqHandler(); }

  void SERCOM4_0_Handler() { if (MotorControl::_instances[4]) MotorControl::_instances[4]->_serial->IrqHandler(); }
  void SERCOM4_1_Handler() { if (MotorControl::_instances[4]) MotorControl::_instances[4]->_serial->IrqHandler(); }
  void SERCOM4_2_Handler() { if (MotorControl::_instances[4]) MotorControl::_instances[4]->_serial->IrqHandler(); }
  void SERCOM4_3_Handler() { if (MotorControl::_instances[4]) MotorControl::_instances[4]->_serial->IrqHandler(); }
}