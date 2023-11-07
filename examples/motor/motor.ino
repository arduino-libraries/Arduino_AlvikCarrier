#include "Arduino_Robot_Firmware.h"

Arduino_Robot_Firmware robot;


MotorControl right(robot.motor_right,robot.encoder_right,100.0,0.01,0,20.0);

unsigned long t=0;

void setup(){
   
  Serial.begin(115200);
  robot.begin();
  right.begin();
  t=millis();
  right.setReference(30.0);
}

void loop(){
  if (millis()-t>20){
    t=millis();
    right.update();
  }
}