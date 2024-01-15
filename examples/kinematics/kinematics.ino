#include "Arduino_Alvik_Firmware.h"

Arduino_Alvik_Firmware alvik;

unsigned long tmotor=0;
unsigned long ttask=0;
uint8_t task=0;

void setup() {
  Serial.begin(115200);
  alvik.begin();
  ttask=millis();
  tmotor=millis();
  task=0;
}

void loop() {
  if (millis()-tmotor>20){
    tmotor=millis();
    alvik.updateMotors();
    alvik.kinematics->updatePose(alvik.motor_control_left->getTravel(), alvik.motor_control_right->getTravel());
    Serial.print("\t");
    Serial.print(alvik.kinematics->getDeltaX());
    Serial.print("\t");
    Serial.print(alvik.kinematics->getDeltaY());
    Serial.print("\t");
    Serial.print(alvik.kinematics->getTheta());
    Serial.print("\n");
  }

  if (millis()-ttask>2000){
    ttask=millis();
    switch (task){
      case 0:
        alvik.drive(50,0);
        break;
      case 1:
        alvik.drive(0,0);
        break;
      case 2:
        alvik.drive(0,-90);
        break;
      case 3:
        alvik.drive(0,0);
        break;
    }
    task++;
    if (task>3){
      task=0;
    }
  }
  
}
