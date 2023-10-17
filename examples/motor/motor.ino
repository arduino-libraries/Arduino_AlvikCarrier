#include "Arduino_Robot_Firmware.h"

Encoder enc_right(TIM5);

DCmotor motor_right(MOTOR_RIGHT_A,MOTOR_RIGHT_A_CH,MOTOR_RIGHT_B,MOTOR_RIGHT_B_CH);
DCmotor motor_left(MOTOR_LEFT_A,MOTOR_LEFT_A_CH, MOTOR_LEFT_B, MOTOR_LEFT_B_CH);

//MotorControl right(&motor_right,&enc_right,0,0,0);

void setup(){
    motor_right.begin();
    motor_left.begin();
}

void loop(){
    for (int i=0; i<4096; i++){
        motor_right.setSpeed(i);
        motor_left.setSpeed(i);
        delay(1);
    }
    for (int i=4095; i>0; i--){
        motor_right.setSpeed(i);
        motor_left.setSpeed(i);
        delay(1);
    }
    delay(1000);
    for (int i=0; i<4096; i++){
        motor_right.setSpeed(i);
        motor_left.setSpeed(-i);
        delay(1);
    }
    for (int i=4095; i>0; i--){
        motor_right.setSpeed(i);
        motor_left.setSpeed(-i);
        delay(1);
    }
    delay(1000);
}