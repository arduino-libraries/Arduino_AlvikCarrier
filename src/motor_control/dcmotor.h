/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#ifndef __DC_MOTOR_H__
#define __DC_MOTOR_H__

#include "Arduino.h"
#include "../definitions/pinout_definitions.h"

class DCmotor{
    private:
        uint32_t pinA;
        uint32_t chA;
        uint32_t pinB;
        uint32_t chB;
        TIM_HandleTypeDef htimX;
        HardwareTimer * timX;
        uint32_t frequency;

        void pwmWrite(uint32_t ch, uint32_t value){
            timX->setCaptureCompare(ch, value, RESOLUTION_12B_COMPARE_FORMAT);
        }

    public:
        DCmotor(const uint32_t _pinA, const uint32_t _chA, const uint32_t _pinB, const uint32_t _chB, 
                const bool flip=false, TIM_TypeDef * _tim = TIM2, const uint32_t _frequency=20000);

        void begin();

        void disable();

        void setSpeed(const int speed);

        void stop();
};

#endif