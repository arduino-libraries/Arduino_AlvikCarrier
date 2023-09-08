#include "Arduino_Robot_Firmware.h"

Arduino_Robot_Firmware robot;

void setup(){
    robot.begin();
}

void loop(){
    robot.led1->setRed(1);
    delay(1000);
    robot.led1->setRed(0);
    delay(1000);
}