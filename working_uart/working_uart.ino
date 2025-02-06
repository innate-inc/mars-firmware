#include <Arduino.h>
#include "wiring_private.h" // Required for pinPeripheral()
#include <HardwareSerial.h>

// Define the new Serial instance on SERCOM4
Uart Serial2(&sercom4, PIN_A2, PIN_A3, SERCOM_RX_PAD_1, UART_TX_PAD_0);

// Interrupt handlers for SERCOM4 (required on SAMD51)
void SERCOM4_0_Handler() {
  Serial2.IrqHandler();
}
void SERCOM4_1_Handler() {
  Serial2.IrqHandler();
}
void SERCOM4_2_Handler() {
  Serial2.IrqHandler();
}
void SERCOM4_3_Handler() {
  Serial2.IrqHandler();
}

void setup() {
  Serial.begin(115200);  // USB Serial for debugging
  Serial2.begin(115200); // Initialize the new UART on SERCOM4

  // Set pins to SERCOM functionality
  pinPeripheral(A2, PIO_SERCOM_ALT); // PB09 (RX) -> SERCOM4 Pad 1
  pinPeripheral(A3, PIO_SERCOM_ALT); // PA04 (TX) -> SERCOM4 Pad 0

  Serial.println("SERCOM4 UART on A3 (TX) and A2 (RX) initialized.");
}

void loop() {
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.print("Received: ");
    Serial.println(c);
  }

  Serial2.println("Hello from SERCOM4 UART!");
  delay(1000);
}

