/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "Arduino.h"

class Encoder{
    private:
        TIM_HandleTypeDef htimX;
        bool flip;
    public:
        Encoder(TIM_TypeDef * _tim, bool _flip=false);

        void begin();

        int getCount();

        void reset();

};

#endif