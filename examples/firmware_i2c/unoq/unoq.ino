#include "Wire.h"
#include "Arduino_RouterBridge.h"
#include "ucPack.h"
#define wire Wire1

#define ALVIK_I2C_ADDRESS 0x2B

ucPack packeter(200);
uint8_t version[3];
int address = ALVIK_I2C_ADDRESS;
int msg_size;

float f = 90;
uint8_t msg[100];

void setup() {
    wire.begin();
    Bridge.begin();
    Monitor.begin();

    while (!scanForAlvik()) {
        Monitor.println("Alvik not found");
        delay(1000);
    }

    Monitor.println("Alvik online");
    delay(1000);

}

void print_msg(size_t size) {

    for (size_t i = 0; i < size; i++) {
        Monitor.print(msg[i], HEX);
        Monitor.print(" ");
    }
    Monitor.println("");

}

void loop() {

    wire.beginTransmission(address);
    wire.write('V');
    wire.endTransmission();
    wire.requestFrom((uint8_t)address,(uint8_t)3);
    wire.readBytes(version,3);

    Monitor.print(version[0], HEX);
    Monitor.print("\t");
    Monitor.print(version[1], HEX);
    Monitor.print("\t");
    Monitor.println(version[2], HEX);
    delay(1000);

    f = 90.0;
    wire.beginTransmission(address);
    wire.write('R');
    //msg_size = packeter.packetC1F('R', f);
    memcpy(msg, &f, sizeof(f));
    //wire.write(packeter.msg, msg_size);
    wire.write(msg, sizeof(f));
    wire.endTransmission();
    delay(1000);

    f = -90.0;
    wire.beginTransmission(address);
    wire.write('R');
    //msg_size = packeter.packetC1F('R', f);
    memcpy(msg, &f, sizeof(f));
    //wire.write(packeter.msg, msg_size);
    wire.write(msg, sizeof(f));
    wire.endTransmission();
    delay(1000);

    wire.beginTransmission(address);
    wire.write('I');
    wire.endTransmission();
    //delay(20);
    wire.requestFrom((uint8_t)address,(uint8_t)29);
    wire.readBytes(msg,29);
    print_msg(29);
    delay(1000);
}

bool scanForAlvik() {
    wire.beginTransmission(address);
    uint8_t error = wire.endTransmission();
    return (error == 0);
}
