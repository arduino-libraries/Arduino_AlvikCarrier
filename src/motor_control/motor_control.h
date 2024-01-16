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
                                       const uint8_t _control_mode = CONTROL_MODE_LINEAR, const float _step_size=5.0){
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

        void begin(){
            motor->begin();
            encoder->begin();
            encoder->reset();
            vel_pid->reset();
            clearMemory();
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


        bool setRPM(const float ref){
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

        void update(){

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

        void brake(){
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

        void resetTravel(){
            travel=0;
        }

        float getTravel(){
            return travel;
        }

        float getAngle(){
            return angle;
        }

        
};

#endif