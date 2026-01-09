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

unsigned long tmotor = 0;
unsigned long tsend = 0;
unsigned long tsensor = 0;
unsigned long timu = 0;
unsigned long tack = 0;
unsigned long tbehaviours = 0;
unsigned long tbattery = 0;

bool tof_available = false;

uint8_t sensor_id = 0;


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

    // sensors publish
    if (millis()-tsensor>10){
        tsensor=millis();
        switch(sensor_id){
            case 0:
                line.update();
                // msg_size = packeter.packetC3I('l', line.getLeft(), line.getCenter(), line.getRight());
                // alvik.serial->write(packeter.msg, msg_size);
                break;
            case 1:
                // alvik.updateTouch();
                // msg_size = packeter.packetC1B('t', alvik.getTouchKeys());
                // alvik.serial->write(packeter.msg,msg_size);
                // msg_size = packeter.packetC1B('m', alvik.getMotion());
                // alvik.serial->write(packeter.msg, msg_size);
                break;
            case 2:
                // alvik.updateAPDS();
                // msg_size = packeter.packetC3I('c', alvik.getRed(), alvik.getGreen(), alvik.getBlue());
                // alvik.serial->write(packeter.msg, msg_size);
                break;
            case 3:
                tof_available = tof.update_rois();
                // if (tof.update_rois()){
                //     msg_size = packeter.packetC7I('f', tof.getLeft(), tof.getCenterLeft(), tof.getCenter(), tof.getCenterRight(), tof.getRight(), tof.getTop(), tof.getBottom());
                //     alvik.serial->write(packeter.msg,msg_size);
                // }
                break;
            case 4:
                // msg_size = packeter.packetC3F('q', alvik.getRoll(), alvik.getPitch(), alvik.getYaw());
                // alvik.serial->write(packeter.msg,msg_size);
                break;
        }
        sensor_id++;
        if (sensor_id>4){
            sensor_id=0;
        }
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
    float acc_x = alvik.getAccelerationX();
    float acc_y = alvik.getAccelerationY();
    float acc_z = alvik.getAccelerationZ();
    float gyr_x = alvik.getAngularVelocityX();
    float gyr_y = alvik.getAngularVelocityY();
    float gyr_z = alvik.getAngularVelocityZ();

    size_t msg_size = sizeof(float)*6;

    float buf[6] = {acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z};
    sendMessage((uint8_t*)buf, msg_size);
}

void publishTofMatrix() {
    int buf[7] = {0,0,0,0,0,0,0};

    if (tof_available) {
        buf[0] = tof.getLeft();
        buf[1] = tof.getCenterLeft();
        buf[2] = tof.getCenter();
        buf[3] = tof.getCenterRight();
        buf[4] = tof.getRight();
        buf[5] = tof.getTop();
        buf[6] = tof.getBottom();
    }

    size_t msg_size = sizeof(int)*7;
    sendMessage((uint8_t*)buf, msg_size);
}

void getData(size_t size) {

    for (size_t i=0; i<size; i++){
        data[i]=alvik.ext_wire->read();
    }

}

void rotateCmd() {
    float angle;
    size_t sz = sizeof(float);
    getData(sz);
    memcpy(&angle, data, sz);

    alvik.rotate(angle);
}

void moveCmd() {
    float distance;
    size_t sz = sizeof(float);
    getData(sz);
    memcpy(&distance, data, sz);

    alvik.move(distance);
}

void driveCmd() {
    float values[2];
    getData(sizeof(values));
    memcpy(values, data, sizeof(values));

    alvik.disableKinematicsMovement();
    alvik.disablePositionControl();
    alvik.drive(values[0], values[1]);  // linear, angular
}

void parseMessage() {

    switch (command){
        case 'R':
            rotateCmd();
            break;
        case 'G':
            moveCmd();
            break;
        case 'V':
            driveCmd();
            break;
        default:
            break;
    }

}

void receiveEvent(int event){
    command=alvik.ext_wire->read();
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
        case 'T':
            publishTofMatrix();
            alvik.ext_wire->write(out_buffer, message_len);
            break;
        default:
            break;
    }
}