#include "Arduino_Robot_Firmware.h"

Arduino_Robot_Firmware robot;

void setup(){
    robot.begin();
}

void loop(){
    for(int i=0; i<180; i++){
        robot.setServoA(i);
        robot.setServoB(i);
        delay(15);
    }
    delay(5000);
}