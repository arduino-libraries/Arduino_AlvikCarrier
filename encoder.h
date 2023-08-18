#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "Arduino.h"

class Encoder{
    private:
        TIM_HandleTypeDef htimX;
    public:
        Encoder(TIM_TypeDef * tim){
            htimX.Instance = tim;
        }

        void begin(){
            TIM_Encoder_InitTypeDef encoder_config;
            //htimX.Instance= TIM2;
            htimX.Init.Prescaler=0;
            htimX.Init.CounterMode= TIM_COUNTERMODE_UP;
            htimX.Init.Period = 65535;
            htimX.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

            encoder_config.EncoderMode = TIM_ENCODERMODE_TI2;

            encoder_config.IC1Polarity=TIM_ICPOLARITY_RISING;
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
            return __HAL_TIM_GET_COUNTER(&htimX);
        }

};

#endif