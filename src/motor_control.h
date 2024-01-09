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

#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "Arduino.h"
#include "dcmotor.h"
#include "encoder.h"
#include "robot_definitions.h"
#include "pid_controller.h"

#define CONTROL_LIMIT 4095
#define MEM_SIZE 5

class MotorControl{
    private:
        float kp;
        float ki;
        float kd;
        
        float travel;
        float reference;

        float controller_period;
        float conversion_factor;
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

        MotorControl(DCmotor * _motor, Encoder * _encoder, const float _kp, const float _ki, const float _kd, const float _controller_period){
            motor = _motor;
            encoder = _encoder;
            
            kp = _kp;
            ki = _ki;
            kd = _kd;

            controller_period = _controller_period;

            travel=0.0;
            reference = 0.0;

            measure = 0.0;
            last_measure = 0.0;

            id_memory=0;
            mean=0.0;

            conversion_factor = 60.0*(1/MOTOR_RATIO)/(controller_period);
            vel_pid = new PidController(kp,ki,kd,controller_period,CONTROL_LIMIT);
        }

        void begin(){
            motor->begin();
            encoder->begin();
            encoder->reset();
            vel_pid->reset();
            clearMemory();
        }

        bool setRPM(const float ref){
            if ((ref<MOTOR_LIMIT)&&(ref>-MOTOR_LIMIT)){
                reference = ref;
                vel_pid->setReference(reference);
                /*
                if ((reference)<0.1&&(reference>-0.1)){
                    vel_pid->reset();
                }
                */
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

        void addMemory(float _val){
            if (id_memory>=MEM_SIZE){
                id_memory=0;
            }
            measure_memory[id_memory]=_val;
            id_memory++;
        }

        float meanMemory(){
            mean=0.0;
            for (i=0; i<MEM_SIZE; i++){
                mean=mean+measure_memory[i];
            }
            return mean/float(MEM_SIZE);
        }                  

        void clearMemory(const float reset_value=0.0){
            for (i=0; i<MEM_SIZE; i++){
                measure_memory[i]=reset_value;
            }
        }

        float getRPM(){
            return measure;
        }


        void test(){
            motor->setSpeed(2000);
            delay(1000);
            motor->setSpeed(-2000);
            delay(1000); 
        }


/*
        void update(){
            measure = encoder->getCount();

            encoder->reset();
            measure = measure*conversion_factor;

            addMemory(measure);

            measure = meanMemory();

            error = reference - measure;

            ctrl_p = kp * error;
            ctrl_i = checkLimits(ctrl_i+ki*error);
            ctrl_d = kd * (error-prev_error);

            prev_error = error;

            actuation = checkLimits(ctrl_p+ctrl_i+ctrl_d);

            motor->setSpeed(-actuation);
        }

        */
    

    /*
       void update(){
            measure = encoder->getCount();
            encoder->reset();
            measure = measure*conversion_factor;
            error = reference-measure;
            error_sum+=error*controller_period;
            error_rate=(measure-last_measure)/controller_period;
            last_measure=measure;
            ctrl_p = error*kp;
            ctrl_i = checkLimits(error_sum*ki);
            ctrl_d = error_rate*kd;
            actuation = checkLimits(ctrl_p+ctrl_i-ctrl_d);
            motor->setSpeed(-actuation);
       }

       */

        void update(){

            measure = encoder->getCount();
            encoder->reset();
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

            /*
            measure = encoder->getCount();
            encoder->reset();
            measure = measure*conversion_factor;
            */

            


            vel_pid->update(measure);
            motor->setSpeed(vel_pid->getControlOutput());
        }

        void setKP(const float _kp){
            vel_pid->setKp(_kp);
        }

        void setKI(const float _ki){
            vel_pid->setKi(_ki);
        }

        void setKD(const float _kd){
            vel_pid->setKd(_kd);
        }

        float getError(){
            return vel_pid->getError();
        }

        
};

#endif