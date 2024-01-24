/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#include "Arduino_AlvikCarrier.h"

Arduino_AlvikCarrier alvik;

unsigned long tmotor=0;
unsigned long ttask=0;
uint8_t task=0;
float reference;

void setup() {
  Serial.begin(115200);
  alvik.begin();
  task = 0;
  reference = 0;
  ttask = millis();
  tmotor = millis();
}

void loop() {
  if (millis()-tmotor>20){
    tmotor = millis();
    alvik.updateMotors();
    Serial.print(reference);
    Serial.print("\t");
    Serial.print(alvik.getPositionLeft());
    Serial.print("\n");
  }

  if (millis()-ttask>5000){
    ttask = millis();
    switch (task){
      case 0:
        reference = 90;
        break;
      case 1:
        reference = 0;
        break;
      case 2:
        reference = -90;
        break;
      case 3:
        reference = 0;
        break;
      case 4:
        reference = 360;
        break;
      case 5:
        reference = 45;
        break;
      case 6:
        reference = -270;
        break;
      case 7:
        reference = 0;
        break;
      case 8:
        reference = 5;
        break;
      case 9:
        reference = 10;
        break;
    }
    alvik.setPositionLeft(reference);
    task++;
    if (task>9){
      task = 0;
    }
  }
  
}

