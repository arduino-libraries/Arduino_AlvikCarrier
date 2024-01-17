/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#include "encoder.h"

Encoder::Encoder(TIM_TypeDef * _tim, bool _flip){
    memset(&htimX, 0, sizeof(htimX));
    htimX.Instance = _tim;
    flip = _flip;
}

void Encoder::begin(){
    
    //htimX.Instance= TIM2;
    htimX.Init.Prescaler=0;
    htimX.Init.CounterMode= TIM_COUNTERMODE_UP;
    htimX.Init.Period = 65535;
    htimX.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;


    TIM_Encoder_InitTypeDef encoder_config;
    encoder_config.EncoderMode = TIM_ENCODERMODE_TI2;
    
    if (flip){
        encoder_config.IC1Polarity=TIM_ICPOLARITY_FALLING;
    }
    else{
        encoder_config.IC1Polarity=TIM_ICPOLARITY_RISING;
    }

    encoder_config.IC1Selection=TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC1Prescaler=TIM_ICPSC_DIV1;
    encoder_config.IC1Filter=0;
    encoder_config.IC2Polarity=TIM_ICPOLARITY_RISING;
    encoder_config.IC2Selection=TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC2Prescaler=TIM_ICPSC_DIV1;
    encoder_config.IC2Filter=0;
    

    HAL_TIM_Encoder_Init(&htimX, &encoder_config);
    HAL_TIM_Encoder_Start(&htimX, TIM_CHANNEL_ALL);
}

int Encoder::getCount(){
    return int16_t(__HAL_TIM_GET_COUNTER(&htimX));  // it gives the sign
}

void Encoder::reset(){
    htimX.Instance->CNT=0;
}