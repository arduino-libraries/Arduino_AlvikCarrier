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

PidController pid(1.2,0,1.0,20,50);


unsigned long tmotor=0;
unsigned long ttask=0;
uint8_t task=0;
float reference;
float position;
float error;

void setup() {
  Serial.begin(115200);
  alvik.begin();
  ttask=millis();
  tmotor=millis();
  task=0;
  reference=0;
  position=0;
  error=0;
}

void loop() {
  if (millis()-tmotor>20){
    tmotor=millis();
    alvik.updateMotors();
    position=alvik.motor_control_left->getTravel()*360.0;
    pid.update(position);
    alvik.motor_control_left->setRPM(pid.getControlOutput());
    Serial.print(reference);
    Serial.print("\t");
    Serial.println(position);
  }

  if (millis()-ttask>5000){
    ttask=millis();
    switch (task){
      case 0:
        reference=90;
        break;
      case 1:
        reference=0;
        break;
      case 2:
        reference=-90;
        break;
      case 3:
        reference=0;
        break;
    }
    pid.setReference(reference);
    task++;
    if (task>3){
      task=0;
    }
  }
  
}
