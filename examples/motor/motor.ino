#include "Arduino_Robot_Firmware.h"

Arduino_Robot_Firmware robot;

unsigned long t=0;
unsigned long t_change = 0;

int status=0;
float reference = 0.0;

void setup(){ 
  Serial.begin(115200);
  robot.begin();
  t=millis();
  t_change=millis();
  robot.setRpmRight(reference);
  robot.setKPidRight(150, 0.1, 0);
  Serial.print("reference");
  Serial.print(" ");
  Serial.println("measure");
}

void loop(){
  if (millis()-t_change>2000){
    t_change=millis();
    switch (status){
      case 0:
          reference=0.0;
          break;
      case 1:
          reference=30.0;
          break;
      case 2:
          reference=60.0;
          break;
      case 3:
          reference=-10.0;
          break;
      case 4:
          reference=-60.0;
          break;
      case 5:
          reference=20.0;
          break;
    }
    status++;
    if (status>1){
      status=0;
    }
    robot.setRpmRight(reference);
  }
  if (millis()-t>20){
    t=millis();
    robot.updateMotors();
    Serial.print(reference);
    Serial.print(" ");
    Serial.println(robot.getRpmRight());
  }
}