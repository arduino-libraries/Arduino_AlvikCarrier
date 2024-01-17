/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#ifndef __RGB_LED_H__
#define __RGB_LED_H__

#include "Arduino.h"

#define COLOR_BLACK 0
#define COLOR_RED 1
#define COLOR_GREEN 2
#define COLOR_ORANGE 3
#define COLOR_BLUE 4
#define COLOR_VIOLET 5
#define COLOR_TEAL 6
#define COLOR_WHITE 7

class RGBled{
    private:
        uint8_t R;
        uint8_t G;
        uint8_t B;
    public:
        RGBled(const uint8_t _Red, const uint8_t _Green, const uint8_t _Blue);

        void setRed(const uint32_t red);
        void setGreen(const uint32_t green);
        void setBlue(const uint32_t blue);

        void set(const uint32_t red, const uint32_t green, const uint32_t blue);
        void set(const uint32_t color);

        void clear();
};



#endif