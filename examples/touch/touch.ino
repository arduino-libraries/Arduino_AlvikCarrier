#include "Arduino_Robot_Firmware.h"

Arduino_Robot_Firmware robot;

void setup() {
  Serial.begin(115200);
  robot.begin();
}

void loop() {
  robot.updateTouch();
  Serial.print(robot.getTouchUp());
  Serial.print("\t");
  Serial.print(robot.getTouchRight());
  Serial.print("\t");
  Serial.print(robot.getTouchDown());
  Serial.print("\t");
  Serial.print(robot.getTouchLeft());
  Serial.print("\t");
  Serial.print(robot.getTouchEnter());
  Serial.print("\t");
  Serial.print(robot.getTouchOk());
  Serial.print("\t");
  Serial.print(robot.getTouchDelete());
  Serial.print("\n");
}
