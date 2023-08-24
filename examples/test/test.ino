#include "Arduino_Robot_Firmware.h"

Encoder enc_right(TIM5);
//Encoder enc_left(TIM3);

DCmotor motor_right(MOTOR_RIGHT_A,MOTOR_RIGHT_A_CH,MOTOR_RIGHT_B,MOTOR_RIGHT_B_CH);
DCmotor motor_left(MOTOR_LEFT_A,MOTOR_LEFT_A_CH,MOTOR_LEFT_B,MOTOR_LEFT_B_CH);


void setup(){
    Serial.begin(115200);
    Serial.println("Start");
    Serial.println("af2");
    AF_Tim2_pwm();
    Serial.println("af5");
    AF_Tim5_pins_encoder();
    //AF_Tim3_pins_encoder();
    Serial.println("encoder");
    enc_right.begin();
    //enc_left.begin();
    Serial.println("motor right");
    motor_right.begin();
    Serial.println("motor left");
    motor_left.begin();

}

void loop(){
  int k=-4096;
  while(k<4096){
    Serial.print(enc_right.getCount());
    Serial.print("  ");
    //Serial.print(enc_left.getCount());
    Serial.print("  ");
    Serial.print(k);
    Serial.print("\n");
    motor_right.setSpeed(k);
    motor_left.setSpeed(k);
    delay(10);
    k+=10;
  }
}