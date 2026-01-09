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

    Monitor.print("VER = ");
    Monitor.println(alvik.getVersionString());
    delay(1000);
    //
    // alvik.rotate(10.0);
    // delay(1000);
    //
    // alvik.rotate(-10.0);
    // delay(1000);
    //
    // alvik.move(10.0);
    // delay(1000);
    //
    // alvik.move(-10.0);
    // delay(1000);

    // alvik.drive(10, 10);
    // delay(1000);

    alvik.setRpm(10, -10);
    delay(1000);

    float ax, ay, az, gx, gy, gz;
    alvik.getImu(ax, ay, az, gx, gy, gz);

    Monitor.print("accX: ");
    Monitor.print(ax);
    Monitor.print(" | accY: ");
    Monitor.print(ay);
    Monitor.print(" | accZ: ");
    Monitor.println(az);
    delay(100);

}

