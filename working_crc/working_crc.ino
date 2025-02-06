byte calculate_crc8_maxim(byte *data, int len) {
    byte crc = 0x00;  // Initial CRC value

    while (len--) {
        crc ^= *data++;  // XOR byte into the current CRC
        for (byte i = 0; i < 8; i++) {
            if (crc & 1)  // If LSB is 1, shift and XOR with polynomial
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {  // Wait for Serial Monitor to connect
        delay(10);
    }
    
    Serial.println("Starting CRC-8/MAXIM Verification...");

    // Example motor response (all 10 bytes)
    byte response[10] = {0x01, 0x64, 0xFE, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16};

    // Compute CRC for first 9 bytes
    byte computed_crc = calculate_crc8_maxim(response, 9);

    // Check if computed CRC matches the received CRC (10th byte)
    if (computed_crc == response[9]) {
        Serial.println("✅ CRC MATCHES! Data is valid.");
    } else {
        Serial.print("❌ CRC MISMATCH! Computed: 0x");
        Serial.print(computed_crc, HEX);
        Serial.print(", Expected: 0x");
        Serial.println(response[9], HEX);
    }
}

void loop() {
    delay(1000);  // Keep the loop idle
}
