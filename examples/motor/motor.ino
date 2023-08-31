#include "Arduino_Robot_Firmware.h"

Encoder enc_right(TIM5);

DCmotor motor_right(MOTOR_RIGHT_A,MOTOR_RIGHT_A_CH,MOTOR_RIGHT_B,MOTOR_RIGHT_B_CH);

MotorControl right(&motor_right,0,0,0);

void setup(){
    enc_right.begin();
    motor_right.begin();
}

void loop(){
    right.test();
}