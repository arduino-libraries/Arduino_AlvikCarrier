#include "Arduino_Robot_Firmware.h"

Encoder enc_right(TIM5);

DCmotor motor_right(MOTOR_RIGHT_A,MOTOR_RIGHT_A_CH,MOTOR_RIGHT_B,MOTOR_RIGHT_B_CH);

MotorControl right(&motor_right,&enc_right,1.0,0.01,0,10.0);

unsigned long t=0;

void setup(){
    Serial.begin(115200);
    motor_right.begin();
    enc_right.begin();
    right.begin();
    t=millis();
}

void loop(){
    if (millis()-t>10){
        Serial.println(enc_right.getCount());
        t=millis();
        right.update();
    }
}