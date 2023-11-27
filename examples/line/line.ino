#include "Arduino_Robot_Firmware.h"
#include "sensor_line.h"

Arduino_Robot_Firmware robot;
SensorLine line(EXT_A2,EXT_A1,EXT_A0);

void setup(){
    robot.begin();
    line.begin();
    Serial.begin(115200);
}

void loop(){
    line.update();
    line.updateCentroid();
    Serial.print(line.getLeft());
    Serial.print("\t");
    Serial.print(line.getCenter());
    Serial.print("\t");
    Serial.print(line.getRight());
    Serial.print("\t");
    Serial.print(line.getCentroid());
    Serial.print("\n");
    delay(10);
}