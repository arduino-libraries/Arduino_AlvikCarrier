/*
  This file is part of the Arduino Alvik library.
  Copyright (c) 2023 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

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
  robot.setKPidRight(30.0, 0.1, 0.4);
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