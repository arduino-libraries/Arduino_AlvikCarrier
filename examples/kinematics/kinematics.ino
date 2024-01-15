#include "Arduino_Alvik_Firmware.h"

Arduino_Alvik_Firmware alvik;

void setup() {
  alvik.begin();
}

void loop() {
  alvik.drive(1,1);
  
}
