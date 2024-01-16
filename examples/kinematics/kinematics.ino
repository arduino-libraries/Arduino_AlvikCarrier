/*
    This file is part of the Arduino Alvik library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


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
