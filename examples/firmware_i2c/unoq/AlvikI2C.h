
#ifndef ARDUINO_ALVIKCARRIER_ALVIKI2C_H
#define ARDUINO_ALVIKCARRIER_ALVIKI2C_H

#include <Wire.h>
#define wire Wire1

#define ALVIK_I2C_ADDRESS 0x2B

class AlvikI2C{

    int address{};
    uint8_t version[3] = {0xFF, 0xFF, 0xFF};

    public:
    AlvikI2C(int addr=ALVIK_I2C_ADDRESS){
        address = addr;
    }

    void begin() {
        wire.begin();
    }

    bool isOnline() {
        wire.beginTransmission(address);
        uint8_t error = wire.endTransmission();
        return (error == 0);
    }

    void getVersion() {
        wire.beginTransmission(address);
        wire.write('V');
        wire.endTransmission();
        wire.requestFrom((uint8_t)address, (uint8_t)3);
        wire.readBytes(version, 3);
    }

    void rotate(float angle) {
        int msg_size = sizeof(float);
        uint8_t msg[msg_size];

        wire.beginTransmission(address);
        wire.write('R');
        memcpy(msg, &angle, msg_size);
        wire.write(msg, msg_size);
        wire.endTransmission();
    }

    void move(float distance) {
        int msg_size = sizeof(float);
        uint8_t msg[msg_size];

        wire.beginTransmission(address);
        wire.write('G');
        memcpy(msg, &distance, msg_size);
        wire.write(msg, msg_size);
        wire.endTransmission();
    }

    String getVersionString() {
        getVersion();
        String versionStr = String(version[0]) + "." +
                            String(version[1]) + "." +
                            String(version[2]);
        return versionStr;
    }

    void getImu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
        size_t arr_size = sizeof(float) * 6;
        float data[6];

        wire.beginTransmission(address);
        wire.write('I');
        wire.endTransmission();
        wire.requestFrom((uint8_t)address, (uint8_t)arr_size);
        wire.readBytes((uint8_t*)data, (uint8_t)arr_size);

        ax = data[0];
        ay = data[1];
        az = data[2];
        gx = data[3];
        gy = data[4];
        gz = data[5];
    }

};



#endif //ARDUINO_ALVIKCARRIER_ALVIKI2C_H
