/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


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
        PidController(const float _kp, const float _ki, const float _kd, const float _ctrl_period, const int _ctrl_limit = 4095);

        void setReference(const float _reference);

        float getControlOutput();
        float getError();

        void setKPid(const float _kp, const float _ki, const float _kd);
        void setKp(const float _k);
        void setKi(const float _k);
        void setKd(const float _k);

        void update(const float measure);

        float checkLimits(float value);

        void reset();
};



#endif