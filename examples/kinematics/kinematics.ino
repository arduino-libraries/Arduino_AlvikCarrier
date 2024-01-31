/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

// This example shows how to implement kinematics commands and retrieve odometry data from Arduino_AlvikCarrier class

#include "Arduino_AlvikCarrier.h"

Arduino_AlvikCarrier alvik;

unsigned long tmotor=0;
unsigned long ttask=0;
unsigned long tled=0;
uint8_t task=0;
bool led_value = false;

void setup() {
  Serial.begin(115200);
  alvik.begin();
  ttask=millis();
  tmotor=millis();
  tled=millis();
  task=0;
}

void loop() {
  if (millis()-tmotor>20){
    tmotor=millis();
    alvik.updateMotors();
    alvik.updateKinematics();
    Serial.print("\t");
    Serial.print(alvik.getLinearVelocity());
    Serial.print("\t");
    Serial.print(alvik.getAngularVelocity());
    Serial.print("\t");
    Serial.print(alvik.getX());
    Serial.print("\t");
    Serial.print(alvik.getY());
    Serial.print("\t");
    Serial.print(alvik.getTheta());
    Serial.print("\n");
  }

  if ((millis()-ttask>5000)||alvik.isTargetReached()){
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
        alvik.move(100);
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

  if (millis()-tled>200){
    tled=millis();
    led_value=!led_value;
    if (task==1){
      alvik.setLedLeftGreen(false);
      alvik.setLedLeftRed(led_value);
    }
    if (task==3){
      alvik.setLedLeftRed(false);
      alvik.setLedLeftGreen(led_value);
    }
  }
}
