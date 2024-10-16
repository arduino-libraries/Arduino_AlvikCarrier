/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

// This is the stock firmware of Arduino Alvik robot.


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
uint8_t ack_required = 0;
bool ack_check = false;
uint8_t ack_code = 0;
uint8_t behaviours;

unsigned long tmotor = 0;
unsigned long tsend = 0;
unsigned long tsensor = 0;
unsigned long timu = 0;
unsigned long tack = 0;
unsigned long tbehaviours = 0;
unsigned long tbattery = 0;


float left, right, value;
float linear, angular;
uint8_t leds;

uint8_t sensor_id = 0;


uint8_t pid;
float kp, ki, kd;
float x, y, theta;

uint8_t servo_A, servo_B;
float position_left, position_right;

int counter_version = 9;
uint8_t version[3];


void setup(){
  alvik.begin();
  alvik.disableIlluminator();
  alvik.setLeds(COLOR_ORANGE);
  alvik.setLedBuiltin(HIGH);
  line.begin();
  tof.begin();


  alvik.getVersion(version[0], version[1], version[2]);
  msg_size = packeter.packetC3B(0x7E, version[0], version[1], version[2]);
  alvik.serial->write(packeter.msg,msg_size);

  alvik.updateBMS();
  msg_size = packeter.packetC1F('p', alvik.getBatteryChargePercentage());
  alvik.serial->write(packeter.msg,msg_size);

  alvik.setLedBuiltin(LOW);
  alvik.setLeds(COLOR_BLACK);


  code=0;
  tmotor=millis();
  tsend=millis();
  tsensor=millis();
  timu=millis();
  tack=millis();
  tbehaviours=millis();
}

void loop(){
  while(alvik.serial->available() > 0) {
    packeter.buffer.push(alvik.serial->read());
  }
  if (packeter.checkPayload()) {
    code = packeter.payloadTop();
    if (!alvik.isBatteryAlert()){
      switch (code){
        case 'J':
          packeter.unpacketC2F(code,left,right);
          alvik.disableKinematicsMovement();
          alvik.disablePositionControl();
          alvik.setRpm(left, right);
          break;

        case 'V':
          packeter.unpacketC2F(code,linear,angular);
          alvik.disableKinematicsMovement();
          alvik.disablePositionControl();
          alvik.drive(linear,angular);
          break;

        case 'W':
          packeter.unpacketC2B1F(code,label,control_type,value);
          alvik.disableKinematicsMovement();
          if (label=='L'){
            switch (control_type){
              case 'V':
                alvik.disablePositionControlLeft();
                alvik.setRpmLeft(value);
                break;
              case 'P':
                alvik.setPositionLeft(value);
                ack_required=MOVEMENT_LEFT;
                ack_check=true;
                break;
              case 'Z':
                alvik.resetPositionLeft(value);
                break;
            }
          }
          if (label=='R'){
            switch (control_type){
              case 'V':
                alvik.disablePositionControlRight();
                alvik.setRpmRight(value);
                break;
              case 'P':
                alvik.setPositionRight(value);
                ack_required=MOVEMENT_RIGHT;
                ack_check=true;
                break;
              case 'Z':
                alvik.resetPositionRight(value);
                break;
            }
          }
          break;


        case 'A':
          packeter.unpacketC2F(code,position_left, position_right);
          alvik.disableKinematicsMovement();
          alvik.setPosition(position_left, position_right);
          ack_required=MOVEMENT_POSITION;
          ack_check=true;
          break;

        
        case 'S':
          packeter.unpacketC2B(code,servo_A,servo_B);
          alvik.setServoA(servo_A);
          alvik.setServoB(servo_B);
          break;
        
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

        case 'R':
          packeter.unpacketC1F(code, value);
          alvik.disablePositionControl();
          alvik.rotate(value);
          ack_required=MOVEMENT_ROTATE;
          ack_check=true;
          break;
        
        case 'G':
          packeter.unpacketC1F(code, value);
          alvik.disablePositionControl();
          alvik.move(value);
          ack_required=MOVEMENT_MOVE;
          ack_check=true;
          break;

        case 'Z':
          packeter.unpacketC3F(code, x, y, theta);
          alvik.resetPose(x, y, theta);
          break;
      }
    }
    switch (code){
      case 'X':
        packeter.unpacketC1B(code, ack_code);
        if (ack_code == 'K') {
          ack_check = false;
        }
        break;

      case 'B':
        packeter.unpacketC1B(code, behaviours);
        switch (behaviours){
          case 1: 
            alvik.setBehaviour(LIFT_ILLUMINATOR, true);
            break;
          case 2:
            alvik.setBehaviour(BATTERY_ALERT, true);
            break;
          default:
            alvik.setBehaviour(ALL_BEHAVIOURS, false);
        }
        break;
    }
  }

  // sensors publish
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
        msg_size = packeter.packetC1B('m', alvik.getMotion());
        alvik.serial->write(packeter.msg,msg_size);
        break;
      case 2:
        alvik.updateAPDS();
        msg_size = packeter.packetC3I('c', alvik.getRed(), alvik.getGreen(), alvik.getBlue());
        alvik.serial->write(packeter.msg,msg_size);
        break;
      case 3:
        if (tof.update_rois()){
          msg_size = packeter.packetC7I('f', tof.getLeft(), tof.getCenterLeft(), tof.getCenter(), tof.getCenterRight(), tof.getRight(), tof.getTop(), tof.getBottom());
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

  // motors update & publish
  if (millis()-tmotor>=20){
    tmotor=millis();
    alvik.updateMotors();
    alvik.updateKinematics();
    // joint speed
    msg_size = packeter.packetC2F('j', alvik.getRpmLeft(),alvik.getRpmRight());
    alvik.serial->write(packeter.msg,msg_size);
    // joint position
    msg_size = packeter.packetC2F('w', alvik.getPositionLeft(),alvik.getPositionRight());
    alvik.serial->write(packeter.msg, msg_size);
    // robot speed
    msg_size = packeter.packetC2F('v', alvik.getLinearVelocity(), alvik.getAngularVelocity());
    alvik.serial->write(packeter.msg, msg_size);
    // pose
    msg_size = packeter.packetC3F('z', alvik.getX(), alvik.getY(), alvik.getTheta());
    alvik.serial->write(packeter.msg, msg_size);
  }

  // acknowledge
  if (millis()-tack > 100){
    tack = millis();
    if (counter_version>0){
      counter_version--;
      alvik.getVersion(version[0], version[1], version[2]);
      msg_size = packeter.packetC3B(0x7E, version[0], version[1], version[2]);
      alvik.serial->write(packeter.msg,msg_size);
    }
    if (ack_check && (alvik.isTargetReached() || alvik.isPositionReached() || alvik.isPositionLeftReached() || alvik.isPositionRightReached())){
      if (ack_required == MOVEMENT_ROTATE){
        msg_size = packeter.packetC1B('x', 'R');
      }
      if (ack_required == MOVEMENT_MOVE){
        msg_size = packeter.packetC1B('x', 'M');
      }
      if (ack_required == MOVEMENT_POSITION){
        msg_size = packeter.packetC1B('x', 'P');
      }
      if (ack_required == MOVEMENT_LEFT){
        msg_size = packeter.packetC1B('x', 'P');
      }
      if (ack_required == MOVEMENT_RIGHT){
        msg_size = packeter.packetC1B('x', 'P');
      }
    }
    else{
      msg_size = packeter.packetC1B('x', 0);
    }
    alvik.serial->write(packeter.msg, msg_size);
  }

  if (millis()-tbehaviours > 100){
    tbehaviours = millis();
    alvik.updateBehaviours();
  }

  // imu update
  if (millis()-timu>10){
    timu=millis();
    alvik.updateImu();
    msg_size = packeter.packetC6F('i', alvik.getAccelerationX(), alvik.getAccelerationY(), alvik.getAccelerationZ(), alvik.getAngularVelocityX(), alvik.getAngularVelocityY(), alvik.getAngularVelocityZ());
    alvik.serial->write(packeter.msg,msg_size);
  }

  // battery update
  if (millis()-tbattery>1000){
    tbattery = millis();
    alvik.updateBMS();
    msg_size = packeter.packetC1F('p', alvik.isBatteryCharging()*alvik.getBatteryChargePercentage());
    alvik.serial->write(packeter.msg,msg_size);
  }
}
