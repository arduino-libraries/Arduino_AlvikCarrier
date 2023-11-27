#include "Arduino_Robot_Firmware.h"

Arduino_Robot_Firmware robot;

void setup(){
    Serial.begin(115200);
    robot.begin();
}

void loop(){
    robot.updateBMS();
    Serial.print(robot.getBatteryVoltage(),4);
    Serial.print("\t");
    Serial.println(robot.getBatteryChargePercentage(),4);
    delay(1000);
}