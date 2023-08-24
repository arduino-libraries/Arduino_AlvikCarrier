#ifndef __DC_MOTOR_H__
#define __DC_MOTOR_H__

#include "Arduino.h"

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
                const bool flip=false,TIM_TypeDef * _tim = TIM2, const uint32_t _frequency=20000){
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
        }

        void begin(){
            timX = new HardwareTimer(htimX.Instance);
            timX->setPWM(chA, pinA, frequency, 0);  
            timX->setPWM(chB, pinB, frequency, 0);  
        }

        void setSpeed(const int speed){
            if (speed>=0){
                pwmWrite(chA,speed);
                pwmWrite(chB,0);
            }
            else{
                pwmWrite(chA,0);
                pwmWrite(chB,-speed);
            } 
        }

        void stop(){
            setSpeed(0);
        }
};

#endif