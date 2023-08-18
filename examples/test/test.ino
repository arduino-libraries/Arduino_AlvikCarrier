#include "Arduino_Robot_Firmware.h"

Encoder enc_right(TIM2);
Encoder enc_left(TIM3);


void setup(){
    Serial.begin(115200);
    Serial.println("Start");
    AF_Tim2_pins_encoder();
    AF_Tim3_pins_encoder();
    enc_right.begin();
    enc_left.begin();

}

void loop(){
    Serial.print(enc_right.getCount());
    Serial.print("  ");
    Serial.println(enc_left.getCount());
    delay(100);
}