/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#include "pid_controller.h"

PidController::PidController(const float _kp, const float _ki, const float _kd, const float _ctrl_period, const int _ctrl_limit){
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

void PidController::setReference(const float _reference){
    reference=_reference;
}

float PidController::getControlOutput(){
    return ctrl_output;
}

float PidController::getError(){
    return error;
}

void PidController::setKPid(const float _kp, const float _ki, const float _kd){
    kp=_kp;
    ki=_ki;
    kd=_kd;
}

void PidController::setKp(const float _k){
    kp=_k;
}

void PidController::setKi(const float _k){
    ki=_k;
}

void PidController::setKd(const float _k){
    kd=_k;
}

void PidController::update(const float measure){
    error = reference - measure;
    error_sum += error*ctrl_period;
    error_rate = (measure-last_measure)/ctrl_period;
    last_measure=measure;
    ctrl_p = error*kp;
    ctrl_i = checkLimits(error_sum*ki);
    ctrl_d = error_rate*kd;
    ctrl_output = checkLimits(ctrl_p+ctrl_i-ctrl_d);
}

float PidController::checkLimits(float value){
    if (value>ctrl_limit){
        return ctrl_limit;
    }
    if (value<-ctrl_limit){
        return -ctrl_limit;
    }
    return value;
}

void PidController::reset(){
    error=0.0;
    error_sum=0.0;
    previous_error=0.0;
    error_rate=0.0;
    reference=0.0;
    last_measure=0.0;
    ctrl_p=0.0;
    ctrl_i=0.0;
    ctrl_d=0.0;
    ctrl_output=0.0;
}