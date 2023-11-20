#include "Arduino_Robot_Firmware.h"


Arduino_Robot_Firmware robot;

void setup(){
    Serial.begin(115200);
    pinMode(LED_BUILTIN,OUTPUT);
}

void loop(){
    digitalWrite(LED_BUILTIN,HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN,LOW);
    delay(1000);
}