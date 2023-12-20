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

void setup() {
  Serial.begin(115200);
  robot.begin();
}

void loop() {
  robot.updateTouch();
  leds();
  Serial.print(robot.getTouchUp());
  Serial.print("\t");
  Serial.print(robot.getTouchRight());
  Serial.print("\t");
  Serial.print(robot.getTouchDown());
  Serial.print("\t");
  Serial.print(robot.getTouchLeft());
  Serial.print("\t");
  Serial.print(robot.getTouchEnter());
  Serial.print("\t");
  Serial.print(robot.getTouchOk());
  Serial.print("\t");
  Serial.print(robot.getTouchDelete());
  Serial.print("\n");
}

void leds(){
  if (robot.getTouchEnter()){
    robot.setIlluminator(true);
  }
  else{
    robot.setIlluminator(false);
  }

  if (robot.getTouchOk()){
    robot.led1->setGreen(true);
    robot.led2->setGreen(true);
  }
  else{
    robot.led1->setGreen(false);
    robot.led2->setGreen(false);
  }

  if (robot.getTouchDelete()){
    robot.led1->setRed(true);
    robot.led2->setRed(true);
  }
  else{
    robot.led1->setRed(false);
    robot.led2->setRed(false);
  }

  if (robot.getTouchLeft()){
    robot.led1->setBlue(true);
  }
  else{
    robot.led1->setBlue(false);
  }

  if (robot.getTouchRight()){
    robot.led2->setBlue(true);
  }
  else{
    robot.led2->setBlue(false);
  }

  if (robot.getTouchUp()){
    robot.setLedBuiltin(HIGH);
  }

  if (robot.getTouchDown()){
    robot.setLedBuiltin(LOW);
  }
}
