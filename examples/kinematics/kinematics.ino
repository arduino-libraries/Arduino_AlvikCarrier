/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

// WIP

#include "Arduino_AlvikCarrier.h"

Arduino_AlvikCarrier alvik;

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
    alvik.updateKinematics();
    Serial.print("\t");
    Serial.print(alvik.kinematics->getLinearVelocity());
    Serial.print("\t");
    Serial.print(alvik.kinematics->getAngularVelocity());
    Serial.print("\t");
    Serial.print(alvik.kinematics->getX());
    Serial.print("\t");
    Serial.print(alvik.kinematics->getY());
    Serial.print("\t");
    Serial.print(alvik.kinematics->getTheta());
    Serial.print("\n");
  }

  if (millis()-ttask>5000){
    ttask=millis();
    switch (task){
      case 0:
        alvik.rotate(90);
        break;
      case 1:
        alvik.disableKinematicsMovement();
        alvik.drive(0,0);
        break;
      case 2:
        alvik.rotate(-90);
        break;
      case 3:
        alvik.disableKinematicsMovement();
        alvik.drive(0,0);
        break;
    }
    task++;
    if (task>3){
      task=0;
    }
  }
}
