/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "Arduino.h"
#include "dcmotor.h"
#include "encoder.h"
#include "../definitions/robot_definitions.h"
#include "pid_controller.h"

#define CONTROL_LIMIT 4095
#define MEM_SIZE 5
#define CONTROL_MODE_NORMAL 0
#define CONTROL_MODE_LINEAR 1


class MotorControl{
    private:

        uint8_t control_mode;

        float kp;
        float ki;
        float kd;
        
        float travel;
        float angle;
        float reference;

        float trip;
        float iterations;
        float start_value;
        float end_value;
        float step_size;
        float step;
        int step_index;
        float interpolation;

        float controller_period;
        float conversion_factor;
        float conversion_factor_travel;
        float measure;
        float last_measure;


        float mean;
        int i;
        int id_memory;
        float measure_memory[MEM_SIZE];


        DCmotor * motor;
        Encoder * encoder;
    public:

        PidController * vel_pid;

        MotorControl(DCmotor * _motor, Encoder * _encoder, const float _kp, const float _ki, const float _kd,
                                       const float _controller_period,
                                       const uint8_t _control_mode = CONTROL_MODE_LINEAR, const float _step_size=5.0);

        void begin();

        float checkLimits(float value);

        void addMemory(float _val);

        float meanMemory();               

        void clearMemory(const float reset_value=0.0);

        float getRPM();

        bool setRPM(const float ref);

        void update();

        void setKP(const float _kp);
        void setKI(const float _ki);
        void setKD(const float _kd);

        float getError();

        void brake();

        void resetTravel();

        float getTravel();

        float getAngle();

        
};

#endif