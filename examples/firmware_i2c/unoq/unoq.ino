#include "Arduino_RouterBridge.h"
#include "AlvikI2C.h"


AlvikI2C alvik;

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

void loop() {

    Monitor.println(alvik.getVersionString());
    delay(1000);

    alvik.rotate(10.0);
    delay(1000);

    alvik.rotate(-10.0);
    delay(1000);

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

