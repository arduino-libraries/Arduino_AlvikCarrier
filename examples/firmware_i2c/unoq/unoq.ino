#include "Arduino_RouterBridge.h"
#include "AlvikI2C.h"


AlvikI2C alvik;

int msg_size;

float f = 90;
uint8_t msg[100];

void setup() {

    alvik.begin();
    Bridge.begin();
    Monitor.begin();

    while (!alvik.isOnline()) {
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

    Monitor.println(alvik.getVersionString());
    delay(1000);

//    f = 90.0;
//    wire.beginTransmission(address);
//    wire.write('R');
//    //msg_size = packeter.packetC1F('R', f);
//    memcpy(msg, &f, sizeof(f));
//    //wire.write(packeter.msg, msg_size);
//    wire.write(msg, sizeof(f));
//    wire.endTransmission();
//    delay(1000);
//
//    f = -90.0;
//    wire.beginTransmission(address);
//    wire.write('R');
//    //msg_size = packeter.packetC1F('R', f);
//    memcpy(msg, &f, sizeof(f));
//    //wire.write(packeter.msg, msg_size);
//    wire.write(msg, sizeof(f));
//    wire.endTransmission();
//    delay(1000);
//
//    wire.beginTransmission(address);
//    wire.write('I');
//    wire.endTransmission();
//    //delay(20);
//    wire.requestFrom((uint8_t)address,(uint8_t)29);
//    wire.readBytes(msg,29);
//    print_msg(29);
//    delay(1000);
}

