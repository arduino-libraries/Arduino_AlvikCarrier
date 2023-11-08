#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "Arduino.h"
#include "dcmotor.h"
#include "encoder.h"
#include "robot_definitions.h"

#define CONTROL_LIMIT 4095
#define MEM_SIZE 5

class MotorControl{
    private:
        float kp;
        float ki;
        float kd;
        
        float travel;
        float reference;

        float controller_frequency;

        float measure;
        float error;
        float conversion_factor;

        float ctrl_p;
        float ctrl_i;
        float ctrl_d;
        float actuation;

        float prev_error;

        float mean;
        int i;
        int id_memory;
        float measure_memory[MEM_SIZE];


        DCmotor * motor;
        Encoder * encoder;
    public:
        MotorControl(DCmotor * _motor, Encoder * _encoder, const float _kp, const float _ki, const float _kd, const float _controller_frequency){
            motor = _motor;
            encoder = _encoder;
            
            kp = _kp;
            ki = _ki;
            kd = _kd;

            controller_frequency = _controller_frequency;

            travel=0.0;
            reference = 0.0;

            measure = 0.0;
            error = 0.0;

            ctrl_p = 0.0;
            ctrl_i = 0.0;
            ctrl_d = 0.0;

            actuation = 0.0;

            id_memory=0;
            mean=0.0;

            conversion_factor = (1/MOTOR_RATIO)*(1000.0*60.0)/controller_frequency;
        }

        void begin(){
            motor->begin();
            encoder->begin();
            encoder->reset();
            clearMemory();
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

        void clearMemory(){
            for (i=0; i<MEM_SIZE; i++){
                measure_memory[i]=0.0;
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



        void update(){
            measure = encoder->getCount();
            Serial.print(measure);
            Serial.print("\t");

            encoder->reset();
            measure = measure*conversion_factor;
            Serial.print(measure);
            Serial.print("\t");

            addMemory(measure);

            measure = meanMemory();

            Serial.print(measure);
            Serial.print("\t");

            error = reference - measure;

            Serial.print(error);
            Serial.print("\t");

            ctrl_p = kp * error;
            ctrl_i = checkLimits(ctrl_i+ki*error);
            ctrl_d = kd * (error-prev_error);

            prev_error = error;

            actuation = checkLimits(ctrl_p+ctrl_i+ctrl_d);
            Serial.println(actuation);

            motor->setSpeed(-actuation);
        }

        
};

#endif