/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#include "dcmotor.h"


DCmotor::DCmotor(const uint32_t _pinA, const uint32_t _chA, const uint32_t _pinB, const uint32_t _chB, 
                const bool flip, TIM_TypeDef * _tim, const uint32_t _frequency){
    if (!flip){
        pinA=_pinA;
        chA=_chA;
        pinB=_pinB;
        chB=_chB; 
    }
    else{
        pinA=_pinB;
        chA=_chB;
        pinB=_pinA;
        chB=_chA;                
    }
    htimX.Instance = _tim; 
    frequency=_frequency;
    pinMode(MOTORS_ENABLE,OUTPUT);
}

void DCmotor::begin(){
    timX = new HardwareTimer(htimX.Instance);
    timX->setPWM(chA, pinA, frequency, 0);  
    timX->setPWM(chB, pinB, frequency, 0);
    digitalWrite(MOTORS_ENABLE,HIGH);
}

void DCmotor::disable(){
    digitalWrite(MOTORS_ENABLE,LOW);
}

void DCmotor::setSpeed(const int speed){
    if (speed>=0){
        pwmWrite(chA,speed);
        pwmWrite(chB,0);
    }
    else{
        pwmWrite(chA,0);
        pwmWrite(chB,-speed);
    } 
}

void DCmotor::stop(){
    setSpeed(0);
}