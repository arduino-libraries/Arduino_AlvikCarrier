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

#include "Arduino_Robot_Firmware.h"
#include "sensor_line.h"

Arduino_Robot_Firmware robot;
SensorLine line(EXT_A2,EXT_A1,EXT_A0);

void setup(){
    robot.begin();
    line.begin();
    Serial.begin(115200);
}

void loop(){
    line.update();
    line.updateCentroid();
    Serial.print(line.getLeft());
    Serial.print("\t");
    Serial.print(line.getCenter());
    Serial.print("\t");
    Serial.print(line.getRight());
    Serial.print("\t");
    Serial.print(line.getCentroid());
    Serial.print("\n");
    delay(10);
}