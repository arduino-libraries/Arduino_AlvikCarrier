
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
        getVersion();
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
        int msg_size = sizeof(angle);
        uint8_t msg[msg_size];

        wire.beginTransmission(address);
        wire.write('R');
        memcpy(msg, &angle, sizeof(angle));
        wire.write(msg, sizeof(angle));
        wire.endTransmission();
    }

    String getVersionString() {
        String versionStr = String(version[0]) + "." +
                            String(version[1]) + "." +
                            String(version[2]);
        return versionStr;
    }


};



#endif //ARDUINO_ALVIKCARRIER_ALVIKI2C_H
