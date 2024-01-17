/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#include "motor_control.h"

MotorControl::MotorControl(DCmotor * _motor, Encoder * _encoder, const float _kp, const float _ki, const float _kd,
                                       const float _controller_period,
                                       const uint8_t _control_mode, const float _step_size){
    motor = _motor;
    encoder = _encoder;
    
    control_mode = _control_mode;


    kp = _kp;
    ki = _ki;
    kd = _kd;

    controller_period = _controller_period;

    travel=0.0;
    angle=0.0;
    reference = 0.0;

    trip=0.0;
    iterations=0.0;
    start_value=0.0;
    end_value=0.0;
    step_size=_step_size;
    step=0.0;
    step_index=0;
    interpolation=0.0;

    measure = 0.0;
    last_measure = 0.0;

    id_memory=0;
    mean=0.0;

    conversion_factor_travel = (1.0/MOTOR_RATIO);
    conversion_factor = 60.0*(1.0/MOTOR_RATIO)/(controller_period);
    vel_pid = new PidController(kp,ki,kd,controller_period,CONTROL_LIMIT);
}

void MotorControl::begin(){
    motor->begin();
    encoder->begin();
    encoder->reset();
    vel_pid->reset();
    clearMemory();
}

float MotorControl::checkLimits(float value){
    if (value>CONTROL_LIMIT){
        return CONTROL_LIMIT;
    }
    if (value<-CONTROL_LIMIT){
        return -CONTROL_LIMIT;
    }
    return value;
}

void MotorControl::addMemory(float _val){
    if (id_memory>=MEM_SIZE){
        id_memory=0;
    }
    measure_memory[id_memory]=_val;
    id_memory++;
}

float MotorControl::meanMemory(){
    mean=0.0;
    for (i=0; i<MEM_SIZE; i++){
        mean=mean+measure_memory[i];
    }
    return mean/float(MEM_SIZE);
}                  

void MotorControl::clearMemory(const float reset_value){
    for (i=0; i<MEM_SIZE; i++){
        measure_memory[i]=reset_value;
    }
}

float MotorControl::getRPM(){
    return measure;
}


bool MotorControl::setRPM(const float ref){
    if ((ref<=MOTOR_LIMIT)&&(ref>=-MOTOR_LIMIT)){
        reference = ref;
        if (control_mode==CONTROL_MODE_LINEAR){
            start_value=interpolation;
            end_value=reference;
            trip=0.0;
            iterations=abs(end_value-start_value)/step_size;
            step=1.0/iterations;
            step_index=0;
        }
        else if(control_mode==CONTROL_MODE_NORMAL){
            vel_pid->setReference(reference);
        }
        return true;
    }
    return false;
}

void MotorControl::update(){

    measure = encoder->getCount();
    encoder->reset();
    angle = measure*conversion_factor_travel;
    travel += angle;
    measure = measure*conversion_factor;

    /* experimental
    if (abs(measure)-abs(reference)>5){
        clearMemory(reference);  
    }
    end */

    addMemory(measure);

    /*
    if (abs(reference)<1.0){
        vel_pid->reset();
        motor->setSpeed(0);
        clearMemory();
    }
    */

    measure = meanMemory();

    //vel_pid->update(measure);
    //motor->setSpeed(vel_pid->getControlOutput());

    if (control_mode==CONTROL_MODE_LINEAR){
        if (step_index<iterations){
            step_index++;
            trip+=step;
            interpolation=trip*(end_value-start_value)+start_value;
            if (abs(interpolation)>abs(reference)){
                interpolation=reference;
            }
        }
        vel_pid->setReference(interpolation);
    }            
    else if(control_mode==CONTROL_MODE_NORMAL){
        vel_pid->setReference(reference);
    }

    vel_pid->update(measure);
    motor->setSpeed(vel_pid->getControlOutput());
}

void MotorControl::setKP(const float _kp){
    vel_pid->setKp(_kp);
}

void MotorControl::setKI(const float _ki){
    vel_pid->setKi(_ki);
}

void MotorControl::setKD(const float _kd){
    vel_pid->setKd(_kd);
}

float MotorControl::getError(){
    return vel_pid->getError();
}

void MotorControl::brake(){
    reference=0.0;
    trip=0.0;
    iterations=0.0;
    start_value=0.0;
    end_value=0.0;
    step=0.0;
    step_index=0;
    interpolation=0.0;
    clearMemory();

    motor->setSpeed(0);
    vel_pid->setReference(0.0);
    vel_pid->reset();
}

void MotorControl::resetTravel(){
    travel=0;
}

float MotorControl::getTravel(){
    return travel;
}

float MotorControl::getAngle(){
    return angle;
}