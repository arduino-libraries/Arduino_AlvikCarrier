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

#include <Wire.h>
#include <VL53L1X.h>
#include <Wire.h>

TwoWire wire(PB7, PB8);
const uint8_t sensorCount = 3;

const uint8_t xshutPins[sensorCount] = { PC5, PB0, PB1 }; //Right, Center, Left

VL53L1X sensors[sensorCount];

void setup(){
  while (!Serial);
  Serial.begin(115200);
  wire.begin();
  wire.setClock(400000);

  for (uint8_t i = 0; i < sensorCount; i++){
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  for (uint8_t i = 0; i < sensorCount; i++){
    pinMode(xshutPins[i], INPUT);
    delay(10);
    sensors[i].setBus(&wire);
    sensors[i].setTimeout(500);
    if (!sensors[i].init()){
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    sensors[i].setAddress(0x2A + i);
    sensors[i].startContinuous(50);
  }
}

void loop(){
  for (uint8_t i = 0; i < sensorCount; i++){
    Serial.print(sensors[i].read());
    if (sensors[i].timeoutOccurred()){
        Serial.print(" TIMEOUT");
    }
    Serial.print('\t');
  }
  Serial.print("\n");
}
