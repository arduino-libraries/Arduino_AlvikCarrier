/*
    This file is part of the Arduino Alvik library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/



// This example shows IMU data, accelerations are in Gs, angular velocities in deg/s and orientation is in degrees.

#include "Arduino_Alvik_Firmware.h"

unsigned long time_imu_update=0;

Arduino_Alvik_Firmware alvik;

void setup(){
    Serial.begin(115200);
    alvik.begin();
    time_imu_update=millis();
}

void loop(){
    if (millis()-time_imu_update>100){
        alvik.updateImu();
        Serial.print("\t");
        Serial.print(alvik.getAccelerationX());
        Serial.print("\t");
        Serial.print(alvik.getAccelerationY()); 
        Serial.print("\t");
        Serial.print(alvik.getAccelerationZ()); 
        Serial.print("\t");
        Serial.print(alvik.getAngularVelocityX());
        Serial.print("\t");
        Serial.print(alvik.getAngularVelocityY()); 
        Serial.print("\t");
        Serial.print(alvik.getAngularVelocityZ()); 
        Serial.print("\t");
        Serial.print(alvik.getRoll());
        Serial.print("\t");
        Serial.print(alvik.getPitch());  
        Serial.print("\t");
        Serial.println(alvik.getYaw()); 
    }
    delay(1);
}