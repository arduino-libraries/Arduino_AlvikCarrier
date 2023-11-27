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