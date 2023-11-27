#include "Arduino_Robot_Firmware.h"


Arduino_Robot_Firmware robot;

void setup(){
    Serial.begin(115200);
    robot.begin();
}

void loop(){
    robot.setLedBuiltin(HIGH);
    delay(100);
    robot.setLedBuiltin(LOW);
    delay(1000);
}