/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/



// This example makes the rear orange led blink. It is also called LED_BUILTIN

#include "Arduino_AlvikCarrier.h"


Arduino_AlvikCarrier alvik;

void setup(){
    alvik.begin();
}

void loop(){
    alvik.setLedBuiltin(HIGH);
    delay(100);
    alvik.setLedBuiltin(LOW);
    delay(1000);
}