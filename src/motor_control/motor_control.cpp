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
                                       const uint8_t _control_mode, const float _step_size,
                                       const float _kp_pos, const float _ki_pos, const float _kd_pos,
                                       const float _pos_controller_period,
                                       const float _pos_max_velocity){
    motor = _motor;
    encoder = _encoder;
    
    control_mode = _control_mode;


    kp = _kp;
    ki = _ki;
    kd = _kd;

    controller_period = _controller_period;

    kp_pos = _kp_pos;
    ki_pos = _ki_pos;
    kd_pos = _kd_pos;
    pos_controller_period = _pos_controller_period;
    pos_max_velocity = _pos_max_velocity;
    position_control_enabled = false;
    is_position_reached = false;



    position = 0.0;
    angle = 0.0;
    reference = 0.0;

    return_flag = false;

    trip = 0.0;
    iterations = 0.0;
    start_value = 0.0;
    end_value = 0.0;
    step_size = _step_size;
    step = 0.0;
    step_index = 0;
    interpolation = 0.0;

    measure = 0.0;
    last_measure = 0.0;

    id_memory = 0;
    mean = 0.0;

    conversion_factor_angle = (1.0/MOTOR_RATIO)*360.0;
    conversion_factor = 60.0*(1.0/MOTOR_RATIO)/(controller_period);

    vel_pid = new PidController(kp,ki,kd,controller_period,CONTROL_LIMIT);
    pos_pid = new PidController(kp_pos, ki_pos, kd_pos, pos_controller_period, pos_max_velocity);
}

void MotorControl::begin(){
    motor->begin();
    encoder->begin();
    encoder->reset();
    vel_pid->reset();
    pos_pid->reset();
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
    reference = ref;
    return_flag = true;
    if (ref>MOTOR_LIMIT){
        reference=MOTOR_LIMIT;
        return_flag = false;
    }
    if (ref<-MOTOR_LIMIT){
        reference=-MOTOR_LIMIT;
        return_flag = false;
    }
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
    return return_flag;
}

void MotorControl::update(){

    measure = encoder->getCount();
    encoder->reset();
    angle = measure*conversion_factor_angle;
    position+=angle;
    measure = measure*conversion_factor;
    addMemory(measure);
    measure = meanMemory();

    if (control_mode==CONTROL_MODE_LINEAR){
        if (step_index<iterations){
            step_index++;
            trip+=step;
            interpolation = trip*(end_value-start_value)+start_value;
            if (abs(interpolation)>abs(reference)){
                interpolation = reference;
            }
        }
        vel_pid->setReference(interpolation);
    }            
    else if(control_mode==CONTROL_MODE_NORMAL){
        vel_pid->setReference(reference);
    }

    vel_pid->update(measure);
    motor->setSpeed(vel_pid->getControlOutput());

    if (position_control_enabled){
        pos_pid->update(position);
        setRPM(round(pos_pid->getControlOutput()/10.0)*10);
        if (abs(pos_pid->getError())<POSITION_THRESHOLD){
            is_position_reached = true;
        }
    }
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
    reference = 0.0;
    trip = 0.0;
    iterations = 0.0;
    start_value = 0.0;
    end_value = 0.0;
    step = 0.0;
    step_index = 0;
    interpolation = 0.0;
    clearMemory();

    motor->setSpeed(0);
    vel_pid->setReference(0.0);
    vel_pid->reset();
}


void MotorControl::enablePositionControl(){
    position_control_enabled = true;
}

void MotorControl::disablePositionControl(){
    position_control_enabled = false;
    is_position_reached = false;
}

bool MotorControl::isPositionControlEnabled(){
    return position_control_enabled;
}

void MotorControl::resetPosition(const float p0){
    disablePositionControl();
    pos_pid->reset();
    position = p0;
    reference_position = p0;
}

float MotorControl::getPosition(){
    return position;
}

void MotorControl::setPosition(const float degree){
    pos_pid->setReference(degree);
    enablePositionControl();
    is_position_reached = false;
}

bool MotorControl::isPositionReached(){
    return is_position_reached;
}

