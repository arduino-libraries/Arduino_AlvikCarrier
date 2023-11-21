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
        robot.led1->set(HIGH,LOW,LOW);
        robot.led2->set(LOW,LOW,HIGH);
        if (millis()-tmotor>2000){
          status++;
          tmotor=millis();
        }
        break;
      case 1:
        robot.motor_left->setSpeed(4095);
        robot.motor_right->setSpeed(-4095);
        robot.led1->set(LOW,LOW,HIGH);
        robot.led2->set(HIGH,LOW,LOW);
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
  robot.led1->set(LOW,LOW,LOW);
  robot.led2->set(LOW,LOW,LOW);
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
        robot.led1->set(LOW,HIGH,LOW);
        robot.led2->set(LOW,HIGH,LOW);
      }
      else{
        if (control>=1){
          robot.led1->set(HIGH,HIGH,LOW);
          robot.led2->set(LOW,LOW,LOW);
        }
        else{
          robot.led1->set(LOW,LOW,LOW);
          robot.led2->set(HIGH,HIGH,LOW);
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
  robot.led1->set(LOW,LOW,LOW);
  robot.led2->set(LOW,LOW,LOW);
}
