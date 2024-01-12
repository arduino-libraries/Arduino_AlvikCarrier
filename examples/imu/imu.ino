/*
  This file is part of the Arduino Alvik library.
  Copyright (c) 2023 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
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