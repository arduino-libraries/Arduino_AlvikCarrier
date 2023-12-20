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
#include "sensor_line.h"

Arduino_Robot_Firmware robot;
SensorLine line(EXT_A2,EXT_A1,EXT_A0);

void setup() {
  robot.begin();
  line.begin();
  robot.disableIlluminator();
}

void loop() {
  robot.updateTouch();
  if (robot.getTouchEnter()){
    simpleMotors();
  }
  if (robot.getTouchOk()){
    line_follower();
  }
}

void simpleMotors(){
  int status=0;
  unsigned long tmotor=millis();
  while(!robot.getTouchDelete()){
    switch (status){
      case 0:
        robot.motor_left->setSpeed(4095);
        robot.motor_right->setSpeed(4095);
        robot.setLedLeft(COLOR_BLUE);
        robot.setLedRight(COLOR_RED);
        if (millis()-tmotor>1000){
          status++;
          tmotor=millis();
        }
        break;
      case 1:
        robot.motor_left->setSpeed(4095);
        robot.motor_right->setSpeed(-4095);
        robot.setLedLeft(COLOR_RED);
        robot.setLedRight(COLOR_BLUE);
        if (millis()-tmotor>1000){
          status++;
          status=0;
          tmotor=millis();
        }
        break;
    }
    robot.updateTouch();
  }
  robot.motor_left->setSpeed(0);
  robot.motor_right->setSpeed(0);
  robot.setLeds(COLOR_BLACK);
}

void line_follower(){
  float e=0;
  float sum_e=0;
  float kp = 100.0;
  float ki = 0.1;
  float control;
  unsigned long tline = millis();
  
  while(!robot.getTouchDelete()){
    robot.updateTouch();
    if (millis()-tline>10){
      tline=millis();
      line.update();
      line.updateCentroid();
      control = kp*line.getCentroid();

      if ((control<1)&&(control>-1)){
        robot.setLedLeft(COLOR_GREEN);
        robot.setLedRight(COLOR_GREEN);
      }
      else{
        if (control>=1){
          robot.setLedLeft(COLOR_ORANGE);
          robot.setLedRight(COLOR_BLACK);
        }
        else{
          robot.setLedLeft(COLOR_BLACK);
          robot.setLedRight(COLOR_ORANGE);
        }
      }

      float control_left=2000-control*400;
      float control_right=2000+control*400;

      if (control_left>4095){
        control_left=4095;
      }

      if (control_left<-4095){
        control_left=-4095;
      }

      if (control_right>4095){
        control_right=4095;
      }

      if (control_right<-4095){
        control_right=-4095;
      }

      robot.motor_left->setSpeed(control_left);
      robot.motor_right->setSpeed(control_right);
    }
  }
  robot.motor_left->setSpeed(0);
  robot.motor_right->setSpeed(0);
  robot.setLeds(COLOR_BLACK);
}
