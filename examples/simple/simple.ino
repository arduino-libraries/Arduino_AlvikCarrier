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

void setup(){
  Serial.begin(115200);
  robot.begin();
  pinMode(PD2,OUTPUT);
  digitalWrite(PD2,HIGH);
}

void loop(){
  robot.motor_left->setSpeed(4095);
  robot.motor_right->setSpeed(4095);
  Serial.print(robot.encoder_left->getCount());
  Serial.print("\t");
  Serial.print(robot.encoder_right->getCount());
  Serial.print("\n"); 

  robot.led1->set(HIGH,LOW,LOW);
  robot.led2->set(LOW,LOW,HIGH);
  delay(1000);

  robot.motor_left->setSpeed(-4095);
  robot.motor_right->setSpeed(4095);
  Serial.print(robot.encoder_left->getCount());
  Serial.print("\t");
  Serial.print(robot.encoder_right->getCount());
  Serial.print("\n"); 
  
  robot.led1->set(LOW,LOW,HIGH);
  robot.led2->set(HIGH,LOW,LOW);
  delay(1000);
}