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


// This example makes the rear orange led blink. It is also called LED_BUILTIN

#include "Arduino_Alvik_Firmware.h"


Arduino_Alvik_Firmware alvik;

void setup(){
    alvik.begin();
}

void loop(){
    alvik.setLedBuiltin(HIGH);
    delay(100);
    alvik.setLedBuiltin(LOW);
    delay(1000);
}