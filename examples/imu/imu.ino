/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/



// This example shows IMU data, accelerations are in Gs, angular velocities in deg/s and orientation is in degrees.

#include "Arduino_AlvikCarrier.h"

unsigned long time_imu_update=0;

Arduino_AlvikCarrier alvik;

void setup(){
    alvik.begin();
    time_imu_update=millis();
}

void loop(){
    if (millis()-time_imu_update>100){
        alvik.updateImu();
        alvik.serial->print("\t");
        alvik.serial->print(alvik.getAccelerationX());
        alvik.serial->print("\t");
        alvik.serial->print(alvik.getAccelerationY()); 
        alvik.serial->print("\t");
        alvik.serial->print(alvik.getAccelerationZ()); 
        alvik.serial->print("\t");
        alvik.serial->print(alvik.getAngularVelocityX());
        alvik.serial->print("\t");
        alvik.serial->print(alvik.getAngularVelocityY()); 
        alvik.serial->print("\t");
        alvik.serial->print(alvik.getAngularVelocityZ()); 
        alvik.serial->print("\t");
        alvik.serial->print(alvik.getRoll());
        alvik.serial->print("\t");
        alvik.serial->print(alvik.getPitch());  
        alvik.serial->print("\t");
        alvik.serial->println(alvik.getYaw()); 
    }
    delay(1);
}