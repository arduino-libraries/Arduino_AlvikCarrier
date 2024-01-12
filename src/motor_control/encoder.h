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

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "Arduino.h"

class Encoder{
    private:
        TIM_HandleTypeDef htimX;
        bool flip;
    public:
        Encoder(TIM_TypeDef * _tim, bool _flip=false){
            memset(&htimX, 0, sizeof(htimX));
            htimX.Instance = _tim;
            flip = _flip;
        }

        void begin(){
            
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

        int getCount(){
            return int16_t(__HAL_TIM_GET_COUNTER(&htimX));  // it gives the sign
        }

        void reset(){
            htimX.Instance->CNT=0;
        }

};

#endif