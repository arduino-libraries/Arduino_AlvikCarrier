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
unsigned long tsend=0;
unsigned long tsensor=0;


float left, right;
uint8_t leds;

uint8_t sensor_id = 0;


uint8_t pid;
float kp, ki, kd;



void setup(){
  Serial.begin(115200);
  robot.begin();
  robot.setLedBuiltin(HIGH);
  line.begin();
  tof.begin();

  uint8_t version[3];
  robot.getVersion(version[0], version[1], version[2]);
  msg_size = packeter.packetC3B(0x7E, version[0], version[1], version[2]);
  robot.serial->write(packeter.msg,msg_size);

  robot.setLedBuiltin(LOW);


  code=0;
  tmotor=millis();
  tsend=millis();
  tsensor=millis();
}

void loop(){
  while(robot.serial->available() > 0) {
    packeter.buffer.push(robot.serial->read());
  }
  if (packeter.checkPayload()) {
    code = packeter.payloadTop();
    switch (code){
      case 'J':
        packeter.unpacketC2F(code,left,right);
        robot.setRpm(left, right);
        break;
      /*
      case 'S':
        robot.setRpm(0,0);
        break;
      */
      case 'L':
        packeter.unpacketC1B(code,leds);
        robot.setAllLeds(leds);
        break;
      
      case 'P':
        packeter.unpacketC1B3F(code,pid,kp,ki,kd);
        if (pid=='L'){
          robot.setKPidLeft(kp,ki,kd);
        }
        if (pid=='R'){
          robot.setKPidRight(kp,ki,kd);
        }
        break;
    }
  }

  if (millis()-tsensor>10){
    tsensor=millis();
    switch(sensor_id){
      case 0:
        line.update();
        msg_size = packeter.packetC3I('l', line.getLeft(), line.getCenter(), line.getRight());
        robot.serial->write(packeter.msg,msg_size);
        break;
      case 1:
        robot.updateTouch();
        msg_size = packeter.packetC1B('t', robot.getTouchKeys());
        robot.serial->write(packeter.msg,msg_size);
        break;
      case 2:
        robot.updateAPDS();
        msg_size = packeter.packetC3I('c', robot.getRed(), robot.getGreen(), robot.getBlue());
        robot.serial->write(packeter.msg,msg_size);
        break;
      case 3:
        if (tof.update_rois()){
          msg_size = packeter.packetC7I('f', tof.getLeft(), tof.getCenterLeft(), tof.getCenter(), tof.getCenterRight(), tof.getRight(), tof.get_min_range_top_mm(), tof.get_max_range_bottom_mm());
          robot.serial->write(packeter.msg,msg_size);
        }
        break;
      case 4:
        msg_size = packeter.packetC3F('q', robot.getRoll(), robot.getPitch(), robot.getYaw());
        robot.serial->write(packeter.msg,msg_size);
        break;
    }
    sensor_id++;
    if (sensor_id>4){
      sensor_id=0;
    }
  } 

  if (millis()-tmotor>10){
    tmotor=millis();
    robot.updateMotors();
    robot.updateImu();
    msg_size = packeter.packetC2F('j', robot.getRpmLeft(),robot.getRpmRight());
    robot.serial->write(packeter.msg,msg_size);
    msg_size = packeter.packetC6F('i', robot.getAccelerationX(), robot.getAccelerationY(), robot.getAccelerationZ(), robot.getAngularVelocityX(), robot.getAngularVelocityY(), robot.getAngularVelocityZ());
    robot.serial->write(packeter.msg,msg_size);
  }
}