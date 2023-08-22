#include "Arduino_Robot_Firmware.h"

Encoder enc_right(TIM5);
Encoder enc_left(TIM3);
RGBled led1(LED_1_RED, LED_1_GREEN, LED_1_BLUE);


void setup(){
    Serial.begin(115200);
    Serial.println("Start");
    AF_Tim5_pins_encoder();
    AF_Tim3_pins_encoder();
    enc_right.begin();
    enc_left.begin();

}

void loop(){
  Serial.print(enc_right.getCount());
  if (enc_right.getCount()>0){
    led1.setBlue(HIGH);
    led1.setRed(LOW);
  }
  else{
    led1.setBlue(LOW);
    led1.setRed(HIGH);
  }
  Serial.print("  ");
  Serial.println(enc_left.getCount());
  delay(10);
}