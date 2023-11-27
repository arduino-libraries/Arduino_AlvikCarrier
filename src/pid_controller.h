#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#include "Arduino.h"

class PidController{
    private:
        float kp;
        float ki;
        float kd;

        float reference;
        float error;
        float error_rate;
        float previous_error;
        float error_sum;
        float last_measure;

        float ctrl_p;
        float ctrl_i;
        float ctrl_d;
        float ctrl_output;

        float ctrl_period;
        float ctrl_limit;


    public:
        PidController(const float _kp, const float _ki, const float _kd, const float _ctrl_period, const int _ctrl_limit = 4095){
            kp=_kp;
            ki=_ki;
            kd=_kd;
            ctrl_period=_ctrl_period;
            ctrl_limit=_ctrl_limit;
            
            reference=0.0;
            error=0.0;
            error_rate=0.0;
            previous_error=0.0;
            error_sum=0.0;
            last_measure=0.0;

            ctrl_p=0.0;
            ctrl_i=0.0;
            ctrl_d=0.0;
            ctrl_output=0.0;
        }

        void setReference(const float _reference){
            reference=_reference;
        }

        float getControlOutput(){
            return ctrl_output;
        }

        void setKPid(const float _kp, const float _ki, const float _kd){
            kp=_kp;
            ki=_ki;
            kd=_kd;
        }

        void setKp(const float _k){
            kp=_k;
        }

        void setKi(const float _k){
            ki=_k;
        }

        void setKd(const float _k){
            kd=_k;
        }

        void update(const float measure){
            error = reference - measure;
            error_sum += error*ctrl_period;
            error_rate = (measure-last_measure)/ctrl_period;
            last_measure=measure;
            ctrl_p = error*kp;
            ctrl_i = checkLimits(error_sum*ki);
            ctrl_d = error_rate*kd;
            ctrl_output = checkLimits(ctrl_p+ctrl_i-ctrl_d);
        }

        float checkLimits(float value){
            if (value>ctrl_limit){
                return ctrl_limit;
            }
            if (value<-ctrl_limit){
                return -ctrl_limit;
            }
            return value;
        }

        void reset(){
            ctrl_output=0.0;
            error=0.0;
            error_sum=0.0;
            previous_error=0.0;
            reference=0.0;
            last_measure=0.0;
        }
};



#endif