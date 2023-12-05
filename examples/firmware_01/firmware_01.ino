#include "Arduino_Robot_Firmware.h"
#include "ucPack.h"

Arduino_Robot_Firmware robot;
ucPack packeter(200);

uint8_t code;
uint8_t msg_size;

unsigned long tmotor=0;
unsigned long tsend=0;


void setup(){
  robot.begin();
  code=0;
  tmotor=millis();
  tsend=millis();
}

void loop(){
  while(robot.serial->available() > 0) {
    packeter.buffer.push(robot.serial->read());
  }
  if (packeter.checkPayload()) {
    code = packeter.payloadTop();
    switch (code){
      case 'J':
        float left, right;
        packeter.unpacketC2F(code,left,right);
        robot.setRpm(left, right);
        break;
    }
  }
  if (millis()-tmotor>10){
    tmotor=millis();
    robot.updateMotors();
  }
  if (millis()-tsend>10){
    tsend=millis();
    msg_size = packeter.packetC2F('j', robot.getRpmLeft(),robot.getRpmRight());
    robot.serial->write(packeter.msg,msg_size);
  }
}