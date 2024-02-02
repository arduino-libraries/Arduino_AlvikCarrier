/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

// WIP -> preliminary firmware


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
uint8_t ack_required=0;
int ack_counter=0;
bool ack_check=false;

unsigned long tmotor=0;
unsigned long tsend=0;
unsigned long tsensor=0;
unsigned long timu=0;
unsigned long tack=0;


float left, right, value;
float linear, angular;
uint8_t leds;

uint8_t sensor_id = 0;


uint8_t pid;
float kp, ki, kd;
float x, y, theta;

uint8_t servo_A, servo_B;


void setup(){
  Serial.begin(115200);
  alvik.begin();
  alvik.disableIlluminator();
  alvik.setLeds(COLOR_ORANGE);
  alvik.setLedBuiltin(HIGH);
  line.begin();
  tof.begin();

  uint8_t version[3];
  alvik.getVersion(version[0], version[1], version[2]);
  msg_size = packeter.packetC3B(0x7E, version[0], version[1], version[2]);
  alvik.serial->write(packeter.msg,msg_size);

  alvik.setLedBuiltin(LOW);
  alvik.setLeds(COLOR_BLACK);


  code=0;
  tmotor=millis();
  tsend=millis();
  tsensor=millis();
  timu=millis();
  tack=millis();
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
              break;
            case 'Z':
              alvik.resetPositionRight(value);
              break;
          }
        }
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
        ack_counter=5;
        ack_check=true;
        break;
      
      case 'G':
        packeter.unpacketC1F(code, value);
        alvik.disablePositionControl();
        alvik.move(value);
        ack_required=MOVEMENT_MOVE;
        ack_counter=5;
        ack_check=true;
        break;

      case 'Z':
        packeter.unpacketC3F(code, x, y, theta);
        alvik.resetPose(x, y, theta);
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
/*
    if (ack_required!=0){
    //if (alvik.getKinematicsMovement()!=MOVEMENT_DISABLED){
      if (alvik.isTargetReached()){
        Serial.print(alvik.isTargetReached());
        Serial.print("\t");
        

        if (ack_required==MOVEMENT_ROTATE){
          msg_size = packeter.packetC1B('x', 'R');
          Serial.println("R");
        }
        if (ack_required==MOVEMENT_MOVE){
          msg_size = packeter.packetC1B('x', 'M');
          Serial.println("M");
        }
        alvik.serial->write(packeter.msg, msg_size);
        //alvik.disableKinematicsMovement();
        ack_required=0;
      }

    }
    */
  }

  if (millis()-tack>100){
    tack=millis();
    msg_size = packeter.packetC1B('x', 0);

    if (ack_check&&alvik.isTargetReached()){
      if (ack_required==MOVEMENT_ROTATE){
        msg_size = packeter.packetC1B('x', 'R');
        //ack_counter--;
        Serial.println("R");
      }
      if (ack_required==MOVEMENT_MOVE){
        msg_size = packeter.packetC1B('x', 'M');
        //ack_counter--;
        Serial.println("M");
      }
      ack_check=false;
    }


    if (ack_counter<=0){
      ack_counter=0;
      ack_required=MOVEMENT_DISABLED;
    }

    alvik.serial->write(packeter.msg, msg_size);
  }

  if (millis()-timu>10){
    timu=millis();
    alvik.updateImu();
    msg_size = packeter.packetC6F('i', alvik.getAccelerationX(), alvik.getAccelerationY(), alvik.getAccelerationZ(), alvik.getAngularVelocityX(), alvik.getAngularVelocityY(), alvik.getAngularVelocityZ());
    alvik.serial->write(packeter.msg,msg_size);
  }
}
