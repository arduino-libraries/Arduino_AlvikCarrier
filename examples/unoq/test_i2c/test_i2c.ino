/*
 * Arduino Alvik Protocol I2C Tester
 *
 * Tests the Alvik carrier board protocol by acting as an I2C master
 * that communicates with the Alvik carrier (I2C slave at address 0x2B)
 *
 * Hardware:
 * - Any Arduino or STM32 board
 * - Connected to Alvik carrier board via I2C
 *
 * I2C Connection:
 * - SDA and SCL lines connected
 * - Common ground
 * - Pull-up resistors on SDA/SCL (usually 4.7kΩ)
 *
 * Alvik Slave Address: 0x2B
 */

#include <Wire.h>

// ==================== CONFIGURATION ====================
#define ALVIK_I2C_ADDRESS 0x2B
#define I2C_TIMEOUT 100  // ms

// Test interval
#define TEST_INTERVAL 1000  // ms

// ==================== UCPACK PROTOCOL ====================
// Simple packet structure for Alvik protocol
// Format: [START_BYTE, LENGTH, PAYLOAD..., CHECKSUM]

#define PACKET_START 0x5A
#define MAX_PAYLOAD_SIZE 64

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

// Motor request IDs
#define MOTOR_SPEED 'j'
#define MOTOR_POSITION 'w'
#define MOTOR_VELOCITY 'v'
#define MOTOR_POSE 'z'

// Sensor IDs
#define SENSOR_LINE 0
#define SENSOR_TOUCH 1
#define SENSOR_COLOR 2
#define SENSOR_TOF 3
#define SENSOR_ORIENTATION 4

// LED colors
#define COLOR_BLACK 0x00
#define COLOR_RED 0x01
#define COLOR_GREEN 0x02
#define COLOR_BLUE 0x04
#define COLOR_YELLOW 0x03
#define COLOR_CYAN 0x06
#define COLOR_MAGENTA 0x05
#define COLOR_WHITE 0x07
#define COLOR_ORANGE 0x09

// ==================== GLOBAL VARIABLES ====================
uint8_t rxBuffer[128];
uint8_t txBuffer[128];
uint16_t rxIndex = 0;

// Statistics
struct Statistics {
    uint32_t packetsSent = 0;
    uint32_t packetsReceived = 0;
    uint32_t i2cErrors = 0;
    uint32_t checksumErrors = 0;
    unsigned long lastResponseTime = 0;
};

Statistics stats;

// ==================== SETUP ====================
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n\n======================================");
    Serial.println("Alvik Protocol I2C Tester");
    Serial.println("======================================\n");

    // Initialize I2C as master
    Wire1.begin();
    //Wire1.setClock(100000);  // 100kHz standard mode

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

// ==================== MAIN LOOP ====================
void loop() {
    // Check for incoming I2C data

    // Check for serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        handleCommand(cmd);
        receiveI2CData(8);
    }

    delay(10);
}

// ==================== I2C FUNCTIONS ====================

/**
 * Scan for Alvik on I2C bus
 */
bool scanForAlvik() {
    Wire1.beginTransmission(ALVIK_I2C_ADDRESS);
    uint8_t error = Wire1.endTransmission();
    return (error == 0);
}

/**
 * Send data to Alvik via I2C
 */
bool sendI2CData(const uint8_t* data, uint8_t length) {
    Wire1.beginTransmission(ALVIK_I2C_ADDRESS);
    Wire1.write(data, length);
    uint8_t error = Wire1.endTransmission(false);

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

/**
 * Receive data from Alvik via I2C
 */
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

            Serial.println();

            stats.packetsReceived++;
            stats.lastResponseTime = millis();
        }
    }

}

/**
 * Parse received data for complete packets
 */
void parseReceivedData() {
    // Simple parsing - look for packet patterns
    // Alvik uses ucPack protocol with format: [CODE, DATA...]

    if (rxIndex > 0) {
        uint8_t code = rxBuffer[0];

        switch (code) {
            case 0x7E:  // Version
                if (rxIndex >= 4) {
                    Serial.print("  Version: ");
                    Serial.print(rxBuffer[1]);
                    Serial.print(".");
                    Serial.print(rxBuffer[2]);
                    Serial.print(".");
                    Serial.println(rxBuffer[3]);
                    rxIndex = 0;
                }
                break;

            case 'p':  // Battery
                Serial.println("  Battery data received");
                rxIndex = 0;
                break;

            case 'i':  // IMU
                Serial.println("  IMU data received");
                rxIndex = 0;
                break;

            case 'l':  // Line sensor
                Serial.println("  Line sensor data received");
                rxIndex = 0;
                break;

            case 't':  // Touch
                Serial.println("  Touch sensor data received");
                rxIndex = 0;
                break;

            case 'c':  // Color
                Serial.println("  Color sensor data received");
                rxIndex = 0;
                break;

            case 'f':  // ToF
                Serial.println("  ToF sensor data received");
                rxIndex = 0;
                break;

            case 'x':  // ACK
                Serial.println("  ACK received");
                rxIndex = 0;
                break;

            default:
                // Unknown packet, wait for more data or timeout
                if (rxIndex > 32) {
                    Serial.println("  Buffer full, clearing");
                    rxIndex = 0;
                }
                break;
        }
    }
}

// ==================== PACKET CREATION ====================

/**
 * Create a simple request packet
 */
uint8_t createRequestPacket(uint8_t* buf, uint8_t cmd, uint8_t id = 0) {
    buf[0] = CMD_REQUEST;
    buf[1] = cmd;
    buf[2] = id;
    return 3;
}

/**
 * Create LED control packet
 */
uint8_t createLEDPacket(uint8_t* buf, uint8_t color) {
    buf[0] = CMD_SET_LEDS;
    buf[1] = color;
    return 2;
}

/**
 * Create motor RPM packet
 */
uint8_t createRPMPacket(uint8_t* buf, float left, float right) {
    buf[0] = CMD_SET_RPM;
    memcpy(&buf[1], &left, sizeof(float));
    memcpy(&buf[5], &right, sizeof(float));
    return 9;
}

/**
 * Create drive velocity packet
 */
uint8_t createDrivePacket(uint8_t* buf, float linear, float angular) {
    buf[0] = CMD_SET_DRIVE;
    memcpy(&buf[1], &linear, sizeof(float));
    memcpy(&buf[5], &angular, sizeof(float));
    return 9;
}

// ==================== COMMAND HANDLERS ====================

void handleCommand(char cmd) {
    uint8_t length = 0;
    static uint8_t ledColor = 0;

    switch (cmd) {
        case '1':  // Request Version
            Serial.println("\n>>> Requesting version...");
            length = createRequestPacket(txBuffer, REQ_VERSION);
            sendI2CData(txBuffer, length);
            break;

        case '2':  // Request Battery
            Serial.println("\n>>> Requesting battery...");
            length = createRequestPacket(txBuffer, REQ_BATTERY);
            sendI2CData(txBuffer, length);
            break;

        case '3':  // Request IMU
            Serial.println("\n>>> Requesting IMU...");
            length = createRequestPacket(txBuffer, REQ_IMU);
            sendI2CData(txBuffer, length);
            break;

        case '4':  // Request Line Sensor
            Serial.println("\n>>> Requesting line sensor...");
            length = createRequestPacket(txBuffer, REQ_SENSORS, SENSOR_LINE);
            sendI2CData(txBuffer, length);
            break;

        case '5':  // Request Touch Sensor
            Serial.println("\n>>> Requesting touch sensor...");
            length = createRequestPacket(txBuffer, REQ_SENSORS, SENSOR_TOUCH);
            sendI2CData(txBuffer, length);
            break;

        case '6':  // Request Color Sensor
            Serial.println("\n>>> Requesting color sensor...");
            length = createRequestPacket(txBuffer, REQ_SENSORS, SENSOR_COLOR);
            sendI2CData(txBuffer, length);
            break;

        case '7':  // Request ToF Sensors
            Serial.println("\n>>> Requesting ToF sensors...");
            length = createRequestPacket(txBuffer, REQ_SENSORS, SENSOR_TOF);
            sendI2CData(txBuffer, length);
            break;

        case '8':  // Set LEDs (cycle colors)
            Serial.println("\n>>> Setting LEDs...");
            ledColor = (ledColor + 1) % 8;
            length = createLEDPacket(txBuffer, ledColor);
            sendI2CData(txBuffer, length);
            Serial.print("LED color: ");
            Serial.println(ledColor);
            break;

        case '9':  // Test motor RPM
            Serial.println("\n>>> Setting motor RPM (50, 50)...");
            length = createRPMPacket(txBuffer, 50.0f, 50.0f);
            sendI2CData(txBuffer, length);
            break;

        case '0':  // Stop motors
            Serial.println("\n>>> Stopping motors...");
            length = createRPMPacket(txBuffer, 0.0f, 0.0f);
            sendI2CData(txBuffer, length);
            break;

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

// ==================== UTILITY FUNCTIONS ====================

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