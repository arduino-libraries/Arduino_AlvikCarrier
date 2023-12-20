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
#include "platform_config_custom.h"
#include <vl53l7cx_class.h>
#include "sensor_tof_matrix.h"

TwoWire wire(PB7, PB8);
#define DEV_I2C wire

#define LPN_PIN PB1
#define I2C_RST_PIN PB0

SensorTofMatrix tof(&wire, LPN_PIN, I2C_RST_PIN);

void setup() {
    tof.begin();
}

void loop() {
    Serial.print("TOP MIN: ");
    Serial.println(tof.get_min_range_top_mm());
    delay(10);
}