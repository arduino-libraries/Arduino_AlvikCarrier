#include <Wire.h>

#define ALVIK_I2C_ADDRESS 0x2B

// Command codes (from Alvik firmware)
#define CMD_SET_RPM 'J'           // Set wheel RPM (left, right)
#define CMD_SET_DRIVE 'V'         // Set linear/angular velocity
#define CMD_SET_WHEEL 'W'         // Set individual wheel control
#define CMD_SET_POSITION 'A'      // Set wheel positions
#define CMD_SET_SERVO 'S'         // Set servo angles
#define CMD_SET_LEDS 'L'          // Set LED colors
#define CMD_SET_PID 'P'           // Set PID parameters
#define CMD_ROTATE 'R'            // Rotate by angle
#define CMD_MOVE 'G'              // Move distance
#define CMD_RESET_POSE 'Z'        // Reset pose (x, y, theta)
#define CMD_ACK_CONFIRM 'X'       // ACK confirmation
#define CMD_SET_BEHAVIOUR 'B'     // Set behaviour
#define CMD_REQUEST '#'           // Request data publication

// Request sub-commands
#define REQ_VERSION 'v'
#define REQ_BATTERY 'b'
#define REQ_IMU 'i'
#define REQ_MOTORS 'm'
#define REQ_ACK 'k'
#define REQ_SENSORS 's'

uint8_t txBuffer[128];

// Statistics
struct Statistics {
    uint32_t packetsSent = 0;
    uint32_t packetsReceived = 0;
    uint32_t i2cErrors = 0;
    uint32_t checksumErrors = 0;
    unsigned long lastResponseTime = 0;
};

Statistics stats;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n\n======================================");
    Serial.println("Alvik Protocol I2C Tester");
    Serial.println("======================================\n");

    Wire1.begin();

    Serial.println("I2C Master initialized");
    Serial.print("Alvik slave address: 0x");
    Serial.println(ALVIK_I2C_ADDRESS, HEX);

    // Scan for Alvik
    Serial.println("\nScanning for Alvik...");
    if (scanForAlvik()) {
        Serial.println("✅ Alvik found!");
    } else {
        Serial.println("❌ Alvik not found!");
        Serial.println("Check I2C connections and address");
    }

    Serial.println("\n======================================");
    Serial.println("Test Menu:");
    Serial.println("  1 - Request Version");
    Serial.println("  2 - Request Battery");
    Serial.println("  3 - Request IMU");
    Serial.println("  4 - Request Line Sensor");
    Serial.println("  5 - Request Touch Sensor");
    Serial.println("  6 - Request Color Sensor");
    Serial.println("  7 - Request ToF Sensors");
    Serial.println("  8 - Set LEDs (cycle colors)");
    Serial.println("  9 - Set Motor RPM (test)");
    Serial.println("  0 - Stop Motors");
    Serial.println("  s - Show statistics");
    Serial.println("  r - Reset statistics");
    Serial.println("======================================\n");

}

void loop() {
    // Check for serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
        receiveI2CData(8);
    }

    delay(10);
}

bool scanForAlvik() {
    Wire1.beginTransmission(ALVIK_I2C_ADDRESS);
    uint8_t error = Wire1.endTransmission();
    return (error == 0);
}

void receiveI2CData(int size) {

    for (int i = 0; i < size; i++) {
        Wire1.requestFrom(ALVIK_I2C_ADDRESS, 1);

        if (Wire1.available()) {
            Serial.print("📥 Received ");

            //while (Wire1.available()) {
            uint8_t b = Wire1.read();
            Serial.print("0x");
            Serial.print(b, HEX);
            Serial.print(" ");

            //}
            Serial.println();

            stats.packetsReceived++;
            stats.lastResponseTime = millis();
        }
    }

}

bool sendI2CData(const uint8_t* data, uint8_t length) {
    Wire1.beginTransmission(ALVIK_I2C_ADDRESS);
    Wire1.write(data, length);
    uint8_t error = Wire1.endTransmission();

    if (error == 0) {
        stats.packetsSent++;
        Serial.print("📤 Sent ");
        Serial.print(length);
        Serial.print(" bytes: ");
        printHex(data, length);
        return true;
    } else {
        stats.i2cErrors++;
        Serial.print("❌ I2C Error: ");
        Serial.println(error);
        return false;
    }
}

uint8_t createRequestPacket(uint8_t* buf, uint8_t cmd, uint8_t id = 0) {
    buf[0] = CMD_REQUEST;
    buf[1] = cmd;
    buf[2] = id;
    return 3;
}

void handleCommand(char cmd) {
    uint8_t length = 0;
    static uint8_t ledColor = 0;

    switch (cmd) {
        case '1':  // Request Version
            Serial.println("\n>>> Requesting version...");
            length = createRequestPacket(txBuffer, REQ_VERSION);
            sendI2CData(txBuffer, length);
            break;

        // case '2':  // Request Battery
        //     Serial.println("\n>>> Requesting battery...");
        //     length = createRequestPacket(txBuffer, REQ_BATTERY);
        //     sendI2CData(txBuffer, length);
        //     break;
        //
        // case '3':  // Request IMU
        //     Serial.println("\n>>> Requesting IMU...");
        //     length = createRequestPacket(txBuffer, REQ_IMU);
        //     sendI2CData(txBuffer, length);
        //     break;
        //
        // case '4':  // Request Line Sensor
        //     Serial.println("\n>>> Requesting line sensor...");
        //     length = createRequestPacket(txBuffer, REQ_SENSORS, SENSOR_LINE);
        //     sendI2CData(txBuffer, length);
        //     break;
        //
        // case '5':  // Request Touch Sensor
        //     Serial.println("\n>>> Requesting touch sensor...");
        //     length = createRequestPacket(txBuffer, REQ_SENSORS, SENSOR_TOUCH);
        //     sendI2CData(txBuffer, length);
        //     break;
        //
        // case '6':  // Request Color Sensor
        //     Serial.println("\n>>> Requesting color sensor...");
        //     length = createRequestPacket(txBuffer, REQ_SENSORS, SENSOR_COLOR);
        //     sendI2CData(txBuffer, length);
        //     break;
        //
        // case '7':  // Request ToF Sensors
        //     Serial.println("\n>>> Requesting ToF sensors...");
        //     length = createRequestPacket(txBuffer, REQ_SENSORS, SENSOR_TOF);
        //     sendI2CData(txBuffer, length);
        //     break;
        //
        // case '8':  // Set LEDs (cycle colors)
        //     Serial.println("\n>>> Setting LEDs...");
        //     ledColor = (ledColor + 1) % 8;
        //     length = createLEDPacket(txBuffer, ledColor);
        //     sendI2CData(txBuffer, length);
        //     Serial.print("LED color: ");
        //     Serial.println(ledColor);
        //     break;
        //
        // case '9':  // Test motor RPM
        //     Serial.println("\n>>> Setting motor RPM (50, 50)...");
        //     length = createRPMPacket(txBuffer, 50.0f, 50.0f);
        //     sendI2CData(txBuffer, length);
        //     break;
        //
        // case '0':  // Stop motors
        //     Serial.println("\n>>> Stopping motors...");
        //     length = createRPMPacket(txBuffer, 0.0f, 0.0f);
        //     sendI2CData(txBuffer, length);
        //     break;

        case 's':  // Show statistics
        case 'S':
            printStatistics();
            break;

        case 'r':  // Reset statistics
        case 'R':
            resetStatistics();
            break;

        default:
            break;
    }
}

void printHex(const uint8_t* data, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        Serial.print("0x");
        if (data[i] < 16) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void printStatistics() {
    Serial.println("\n======================================");
    Serial.println("STATISTICS");
    Serial.println("======================================");
    Serial.print("Packets sent: ");
    Serial.println(stats.packetsSent);
    Serial.print("Packets received: ");
    Serial.println(stats.packetsReceived);
    Serial.print("I2C errors: ");
    Serial.println(stats.i2cErrors);
    Serial.print("Checksum errors: ");
    Serial.println(stats.checksumErrors);
    Serial.print("Last response: ");
    if (stats.lastResponseTime > 0) {
        Serial.print((millis() - stats.lastResponseTime) / 1000);
        Serial.println(" seconds ago");
    } else {
        Serial.println("never");
    }
    Serial.println("======================================\n");
}

void resetStatistics() {
    stats.packetsSent = 0;
    stats.packetsReceived = 0;
    stats.i2cErrors = 0;
    stats.checksumErrors = 0;
    Serial.println("\n✅ Statistics reset\n");
}