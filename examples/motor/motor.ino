/*
    This file is part of the Arduino Alvik library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#include "Arduino_Alvik_Firmware.h"

Arduino_Alvik_Firmware robot;

unsigned long t=0;
unsigned long t_change = 0;

int status=0;
float reference = 0.0;

void setup(){ 
  Serial.begin(115200);
  robot.begin();
  t=millis();
  t_change=millis();
  robot.setRpm(reference,reference);

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
          reference=70.0;
          break;
      case 3:
          reference=-70.0;
          break;
      case 4:
          reference=-10.0;
          break;
      case 5:
          reference=20.0;
          break;
    }
    status++;
    if (status>5){
      status=0;
    }
    robot.setRpm(reference,reference);
  }
  if (millis()-t>20){
    t=millis();
    robot.updateMotors();
    
    Serial.print(reference);
    Serial.print(" ");
    Serial.print(robot.getRpmRight());
    Serial.print(" ");
    Serial.println(robot.getRpmLeft());
  }
}