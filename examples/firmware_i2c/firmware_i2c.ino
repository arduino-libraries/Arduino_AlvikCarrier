#include "Arduino_AlvikCarrier.h"
#include "sensor_line.h"
#include "sensor_tof_matrix.h"
#include "ucPack.h"
#define OUT_BUF_SIZE 512


Arduino_AlvikCarrier alvik;
SensorLine line(EXT_A2,EXT_A1,EXT_A0);
SensorTofMatrix tof(alvik.wire, EXT_GPIO3, EXT_GPIO2);
bool blink;

uint8_t command = 0x00;
uint8_t data[200];
uint8_t version[3] = {0xff, 0xff, 0xff};

ucPack packeter(200);
uint8_t out_buffer[OUT_BUF_SIZE];
size_t message_len = 0;

float tmp_float;

unsigned long tmotor = 0;
unsigned long tsend = 0;
unsigned long tsensor = 0;
unsigned long timu = 0;
unsigned long tack = 0;
unsigned long tbehaviours = 0;
unsigned long tbattery = 0;


void setup() {
    //Serial.begin(115200);
    alvik.begin();
    alvik.disableIlluminator();
    alvik.setLeds(COLOR_ORANGE);
    alvik.setLedBuiltin(HIGH);
    line.begin();
    tof.begin();
    alvik.setLedBuiltin(LOW);
    alvik.setLeds(COLOR_BLACK);

    alvik.getVersion(version[0], version[1], version[2]);

    alvik.ext_wire->onReceive(receiveEvent);
    alvik.ext_wire->onRequest(requestEvent);
}

void loop() {

    // motors update
    if (millis()-tmotor>=20){
        tmotor=millis();
        alvik.updateMotors();
        alvik.updateKinematics();
    }

    if (millis()-tbehaviours > 100){
        tbehaviours = millis();
        alvik.updateBehaviours();
    }

    // imu update
    if (millis()-timu>10){
        timu=millis();
        alvik.updateImu();
    }

    // battery update
    if (millis()-tbattery>1000){
        tbattery = millis();
        //alvik.updateBMS();
        if (blink) {
            alvik.setLedLeft(COLOR_GREEN);
        } else {
            alvik.setLedLeft(COLOR_RED);
        }
        blink=!blink;
    }

}

void sendMessage(const uint8_t * buf, const size_t length) {
    memcpy(out_buffer, buf, length);
    message_len = length;
}

void publishVersion() {
    sendMessage(version, 3);
}

void publishImu() {
    uint8_t msg_size = packeter.packetC6F('i', alvik.getAccelerationX(), alvik.getAccelerationY(), alvik.getAccelerationZ(), alvik.getAngularVelocityX(), alvik.getAngularVelocityY(), alvik.getAngularVelocityZ());
    sendMessage(packeter.msg, msg_size);
}

void parseMessage() {

    float angle;
    uint8_t code;

    switch (command){
        case 'R':
            // for (size_t i = 0; i < 9; i++) {
            //     packeter.buffer.push(alvik.ext_wire->read());
            // }
            data[0]=alvik.ext_wire->read();
            data[1]=alvik.ext_wire->read();
            data[2]=alvik.ext_wire->read();
            data[3]=alvik.ext_wire->read();
            memcpy(&angle, data, 4);

            // packeter.unpacketC1F(code, angle);
            // alvik.rotate(angle);
            alvik.serial->println(angle);
            if (angle > 0) {
                alvik.setLedRight(COLOR_VIOLET);
            } else {
                alvik.setLedRight(COLOR_RED);
            }
            //Serial.println("rotate");
            break;
    }

}

void receiveEvent(int event){
    command=alvik.ext_wire->read();
    //Serial.println("requested\t"+String(command));
    parseMessage();

}

void requestEvent(){
    switch(command){
        case 'V':
            publishVersion();
            alvik.ext_wire->write(out_buffer, message_len);
            break;
        case 'I':
            publishImu();
            alvik.ext_wire->write(out_buffer, message_len);
            break;
        case 'D':
            data[0]=0x01;
            data[1]=0x02;
            data[2]=0x03;
            data[3]=0x04;
            data[4]=0x05;
            data[5]=0x06;
            alvik.ext_wire->write(data,6);
            break;
            /*
            case 'T':
              test_flag=true;
              break;
            case 'N':
              test_flag=false;
              break;
            */
    }
}