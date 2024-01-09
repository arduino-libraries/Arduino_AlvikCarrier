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
#include "sensor_tof_matrix.h"
#include "ucPack.h"

Arduino_Robot_Firmware robot;
SensorLine line(EXT_A2,EXT_A1,EXT_A0);
SensorTofMatrix tof(robot.wire, EXT_GPIO3, EXT_GPIO2);


ucPack packeter(200);

uint8_t code;
uint8_t msg_size;

unsigned long tmotor=0;
unsigned long tupdate=0;
unsigned long tsensor=0;


float left, right;
uint8_t leds;

uint8_t sensor_id = 0;


uint8_t pid;
float kp, ki, kd;


float reference=30.0;
float interpolation=0.0;
float trip=0.0;
float iterations = 10.0;
float start_value= 0.0;
float end_value = 0.0;
float step_size = 5.0;
float step= 1.0/iterations;


int c=0;



void setup(){
  Serial.begin(115200);
  robot.begin();
  robot.disableIlluminator();
  robot.setLedBuiltin(HIGH);
  robot.setKPidLeft(30, 450, 0.0);  //120 800 10
  tmotor=millis();
  tupdate=millis();

}
int i=0;
void loop(){
  start_value=interpolation;
  if (millis()-tupdate>1000){
    tupdate=millis();
    //reference=-reference;
    switch (c){
      case 0:
        reference = 0;
        break;
      case 1:
        reference = 30;
        break;
      case 2:
        reference = -30;
        break;
      case 3:
        reference = -60;
        break;
      case 4:
        reference = 80;
        break;
      case 5:
        reference = 10;
        break;
    }
    c++;
    if (c>5){
      c=0;
    }
    //start_value=robot.getRpmLeft();
    end_value=reference; 
    trip=0.0;
    iterations=abs(end_value-start_value)/step_size;
    step=1.0/iterations;
    i=0;
  }

  if (millis()-tmotor>20){
    if (i<iterations){
      i++;
      trip+=step;
      interpolation=trip*(end_value-start_value)+start_value;
    }
    tmotor=millis();
    robot.updateMotors();
    Serial.print("\t");
    Serial.print(reference);
    Serial.print("\t");
    Serial.print(interpolation);
    Serial.print("\t");
    Serial.print(robot.getRpmLeft());
    Serial.print("\n");
    robot.setRpmLeft(interpolation);
  }
}