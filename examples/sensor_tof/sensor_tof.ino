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

#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>
#include "sensor_tof_matrix.h"
#include "platform_config_custom.h"

TwoWire wire(PB7, PB8);

#define LPN_PIN PB1
#define I2C_RST_PIN PB0

SensorTofMatrix tof(&wire, LPN_PIN, I2C_RST_PIN);

void setup() {
    Serial.begin(9600);
    tof.begin();
}

void loop() {
    bool updated = false;
    updated = tof.update_rois();
    
    if (updated) {
      /*
      Serial.println(" ");
      Serial.print("TOP=");
      Serial.println(tof.getTop());
      Serial.print("BOT=");
      Serial.println(tof.getBottom());
      Serial.print("LEFT=");
      Serial.println(tof.getLeft());
      Serial.print("RIGHT=");
      Serial.println(tof.getRight());
      Serial.print("CLEFT=");
      Serial.println(tof.getCenterLeft());
      Serial.print("CRIGHT=");
      Serial.println(tof.getCenterRight());
      Serial.print("CENTER=");
      Serial.println(tof.getCenter());
      Serial.print("LEFT_MIN=");
      Serial.println(tof.get_min_range_left_mm());
      Serial.print("RIGHT_MIN=");
      Serial.println(tof.get_min_range_right_mm());
      Serial.print("CLEFT_MIN=");
      Serial.println(tof.get_min_range_center_left_mm());
      Serial.print("CRIGHT_MIN=");
      Serial.println(tof.get_min_range_center_right_mm());
      Serial.print("CENTER_MIN=");
      Serial.println(tof.get_min_range_center_mm());
      */
      Serial.print("\t");
      Serial.print("L");
      Serial.print("\t");
      Serial.print("CL");
      Serial.print("\t");
      Serial.print("C");
      Serial.print("\t");
      Serial.print("CR");
      Serial.print("\t");
      Serial.print("R");
      Serial.print("\t");
      Serial.print("T");
      Serial.print("\t");
      Serial.print("B");
      Serial.println("");

      Serial.print("\t");
      Serial.print(tof.getLeft());
      Serial.print("\t");
      Serial.print(tof.getCenterLeft());
      Serial.print("\t");
      Serial.print(tof.getCenter());
      Serial.print("\t");
      Serial.print(tof.getCenterRight());
      Serial.print("\t");
      Serial.print(tof.getRight());
      Serial.print("\t");
      Serial.print(tof.getTop());
      Serial.print("\t");
      Serial.print(tof.getBottom());
      Serial.print("\n");
    }
    

    delay(10);
}