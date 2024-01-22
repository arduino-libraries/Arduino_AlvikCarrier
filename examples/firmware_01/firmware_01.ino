/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#include "Arduino_AlvikCarrier.h"
#include "sensor_line.h"
#include "sensor_tof_matrix.h"
#include "ucPack.h"

Arduino_AlvikCarrier alvik;
SensorLine line(EXT_A2,EXT_A1,EXT_A0);
SensorTofMatrix tof(alvik.wire, EXT_GPIO3, EXT_GPIO2);


ucPack packeter(200);

uint8_t code;
uint8_t label;
uint8_t control_type;
uint8_t msg_size;

unsigned long tmotor=0;
unsigned long tsend=0;
unsigned long tsensor=0;
unsigned long timu=0;


float left, right, value;
uint8_t leds;

uint8_t sensor_id = 0;


uint8_t pid;
float kp, ki, kd;



void setup(){
  Serial.begin(115200);
  alvik.begin();
  alvik.setLedBuiltin(HIGH);
  line.begin();
  tof.begin();

  uint8_t version[3];
  alvik.getVersion(version[0], version[1], version[2]);
  msg_size = packeter.packetC3B(0x7E, version[0], version[1], version[2]);
  alvik.serial->write(packeter.msg,msg_size);

  alvik.setLedBuiltin(LOW);


  code=0;
  tmotor=millis();
  tsend=millis();
  tsensor=millis();
  timu=millis();
}

void loop(){
  while(alvik.serial->available() > 0) {
    packeter.buffer.push(alvik.serial->read());
  }
  if (packeter.checkPayload()) {
    code = packeter.payloadTop();
    switch (code){
      case 'J':
        packeter.unpacketC2F(code,left,right);
        alvik.setRpm(left, right);
        break;
      case 'W':
        packeter.unpacketC2B1F(code,label,control_type,value);
        if ((label == 'L') && (control_type == 'V')) {
          alvik.motor_control_left->setRPM(value);
        }
        else if ((label == 'R') && (control_type == 'V'))
        {
          alvik.motor_control_right->setRPM(value);
        }
        
        break;
      /*
      case 'S':
        alvik.setRpm(0,0);
        break;
      */
      case 'L':
        packeter.unpacketC1B(code,leds);
        alvik.setAllLeds(leds);
        break;
      
      case 'P':
        packeter.unpacketC1B3F(code,pid,kp,ki,kd);
        if (pid=='L'){
          alvik.setKPidLeft(kp,ki,kd);
        }
        if (pid=='R'){
          alvik.setKPidRight(kp,ki,kd);
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
        alvik.serial->write(packeter.msg,msg_size);
        break;
      case 1:
        alvik.updateTouch();
        msg_size = packeter.packetC1B('t', alvik.getTouchKeys());
        alvik.serial->write(packeter.msg,msg_size);
        break;
      case 2:
        alvik.updateAPDS();
        msg_size = packeter.packetC3I('c', alvik.getRed(), alvik.getGreen(), alvik.getBlue());
        alvik.serial->write(packeter.msg,msg_size);
        break;
      case 3:
        if (tof.update_rois()){
          msg_size = packeter.packetC7I('f', tof.getLeft(), tof.getCenterLeft(), tof.getCenter(), tof.getCenterRight(), tof.getRight(), tof.get_min_range_top_mm(), tof.get_max_range_bottom_mm());
          alvik.serial->write(packeter.msg,msg_size);
        }
        break;
      case 4:
        msg_size = packeter.packetC3F('q', alvik.getRoll(), alvik.getPitch(), alvik.getYaw());
        alvik.serial->write(packeter.msg,msg_size);
        break;
    }
    sensor_id++;
    if (sensor_id>4){
      sensor_id=0;
    }
  } 

  if (millis()-tmotor>20){
    tmotor=millis();
    alvik.updateMotors();
    alvik.updateImu();
    msg_size = packeter.packetC2F('j', alvik.getRpmLeft(),alvik.getRpmRight());
    alvik.serial->write(packeter.msg,msg_size);
   
  }

  if (millis()-timu>10){
    timu=millis();
    alvik.updateImu();
    msg_size = packeter.packetC6F('i', alvik.getAccelerationX(), alvik.getAccelerationY(), alvik.getAccelerationZ(), alvik.getAngularVelocityX(), alvik.getAngularVelocityY(), alvik.getAngularVelocityZ());
    alvik.serial->write(packeter.msg,msg_size);
  }
}