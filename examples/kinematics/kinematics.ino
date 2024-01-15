#include "Arduino_Alvik_Firmware.h"

Arduino_Alvik_Firmware alvik;

unsigned long tmotor=0;
unsigned long ttask=0;
uint8_t task=0;

void setup() {
  alvik.begin();
  ttask=millis();
  tmotor=millis();
  task=0;
}

void loop() {
  if (millis()-tmotor>20){
    tmotor=millis();
    alvik.updateMotors();
  }

  if (millis()-ttask>2000){
    ttask=millis();
    switch (task){
      case 0:
        alvik.drive(0,90);
        break;
      case 1:
        alvik.drive(0,0);
        break;
      case 2:
        alvik.drive(0,-90);
        break;
      case 3:
        alvik.drive(0,0);
        break;
    }
    task++;
    if (task>3){
      task=0;
    }
  }
}
