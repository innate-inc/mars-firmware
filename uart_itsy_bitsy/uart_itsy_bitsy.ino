#define UART Serial1  // Use Serial1 for hardware UART

const uint8_t SOM[] = {0x69, 0x69};  // Start-of-Message marker

// CRC-8/MAXIM function (Polynomial 0x8C, Initial Value 0x00)
uint8_t calculate_crc8_maxim(uint8_t *data, int len) {
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8C;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void setup() {
    Serial.begin(115200);  // USB Serial for debugging
    UART.begin(115200);    // UART communication
}

void loop() {
    // Step 1: Look for SOM (0x69 0x69)
    while (UART.available() >= 10) {  // Ensure we have a full message
        uint8_t firstByte = UART.read();
        if (firstByte == SOM[0] && UART.peek() == SOM[1]) {  // Check for 0x69 0x69
            UART.read(); // Consume second SOM byte

            // Step 2: Read the next 8 bytes
            uint8_t message[8];
            UART.readBytes(message, 8);

            // Step 3: Extract Payload & CRC
            uint8_t received_crc = message[7];  // Last byte is CRC
            uint8_t calculated_crc = calculate_crc8_maxim(message, 7);  // CRC over bytes 2-8

            // Step 4: Validate CRC
            if (received_crc == calculated_crc) {
                decodeMessage(message);  // Process the message
            }
        }
    }
}

// Step 5: Decode and print **only LED and Status** messages
void decodeMessage(uint8_t *msg) {
    uint8_t commandID = msg[0];

    // Declare all variables at the beginning
    uint8_t ledMode, red, green, blue;
    uint16_t batteryVoltage;
    int16_t motorTemp;
    uint8_t faultCode;

    switch (commandID) {
        case 0x02:  // LED Command
            ledMode = msg[1];
            red = msg[3];
            green = msg[4];
            blue = msg[5];
            Serial.print("[LED Command] Mode = ");
            Serial.print(ledMode);
            Serial.print(", Color = RGB(");
            Serial.print(red); Serial.print(", ");
            Serial.print(green); Serial.print(", ");
            Serial.print(blue); Serial.println(")");
            break;

        case 0x03:  // Status Request
            Serial.println("[Status Request] Command Received.");
            break;

        case 0x82:  // LED Response
            ledMode = msg[1];
            red = msg[2];
            green = msg[3];
            blue = msg[4];
            Serial.print("[LED Response] Mode = ");
            Serial.print(ledMode);
            Serial.print(", Color = RGB(");
            Serial.print(red); Serial.print(", ");
            Serial.print(green); Serial.print(", ");
            Serial.print(blue); Serial.println(")");
            break;

        case 0x83:  // Status Response
            batteryVoltage = (msg[1] << 8) | msg[2];
            motorTemp = (msg[3] << 8) | msg[4];
            faultCode = msg[5];
            Serial.print("[Status Response] Battery = ");
            Serial.print(batteryVoltage);
            Serial.print("V, Motor Temp = ");
            Serial.print(motorTemp);
            Serial.print("°C, Fault Code = ");
            Serial.println(faultCode, HEX);
            break;

        default:
            // Ignore movement (0x01, 0x81) and unknown commands
            break;
    }
}

