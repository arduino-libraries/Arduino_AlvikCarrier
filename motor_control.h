#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "Arduino.h"
#include "dcmotor.h"
#include "encoder.h"
#include "robot_definitions.h"

#define CONTROL_LIMIT 4095

class MotorControl{
    private:
        float kp;
        float ki;
        float kd;
        
        float travel;
        float reference;

        float measure;
        float error;

        float ctrl_p;
        float ctrl_i;
        float ctrl_d;
        float actuation;

        DCmotor * motor;
        Encoder * encoder;
    public:
        MotorControl(DCmotor * _motor, Encoder * _encoder, const float _kp, const float _ki, const float _kd){
            motor = _motor;
            encoder = _encoder;
            
            kp = _kp;
            ki = _ki;
            kd = _kd;

            travel=0.0;
            reference = 0.0;

            measure = 0.0;
            error = 0.0;

            ctrl_p = 0.0;
            ctrl_i = 0.0;
            ctrl_d = 0.0;

            actuation = 0.0;
        }

        void begin(){
            motor->begin();
            encoder->begin();
            encoder->reset();
        }

        bool setReference(const float ref){
            if ((ref<MOTOR_LIMIT)&&(ref>-MOTOR_LIMIT)){
                reference = ref;
                return true;
            }
            return false;
        }

        float checkLimits(float value){
            if (value>CONTROL_LIMIT){
                return CONTROL_LIMIT;
            }
            if (value<-CONTROL_LIMIT){
                return -CONTROL_LIMIT;
            }
            return value;
        }

        

        void test(){
            motor->setSpeed(2000);
            delay(1000);
            motor->setSpeed(-2000);
            delay(1000);
            
        }



        void update(){
            measure = encoder->getCount();
            encoder->reset();
            error = reference - measure;

            ctrl_p = kp * error;
            ctrl_i = checkLimits(ctrl_i+ki*error);
            ctrl_d = kd * (error-prev_error);

            prev_error = error;

            actuation = checkLimits(ctrl_p+ctrl_i+ctrl_d);

            motor->setSpeed(actuation);
        }

        
};

#endif