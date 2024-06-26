/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#ifndef __SENSOR_LINE_H__
#define __SENSOR_LINE_H__

#include "Arduino.h"

class SensorLine{
    private:
        uint8_t sensor_pins[3];
        int values[3];
        uint8_t index;

        float centroid;
        float sum_values;
        float sum_weight;

    
    public:
        const uint8_t LINE_SENSOR_NUMBER = 3;
        const uint8_t LINE_SENSOR_LEFT = 0;
        const uint8_t LINE_SENSOR_CENTER = 1;
        const uint8_t LINE_SENSOR_RIGHT = 2;
        
        SensorLine(const uint8_t _pin_left, const uint8_t _pin_center, const uint8_t _pin_right){
            sensor_pins[LINE_SENSOR_LEFT]=_pin_left;
            sensor_pins[LINE_SENSOR_CENTER]=_pin_center;
            sensor_pins[LINE_SENSOR_RIGHT]=_pin_right;
            index=0;
            for (index=0; index<LINE_SENSOR_NUMBER; index++){
                values[index]=0;
            }
            centroid=0.0;
            sum_values=0.0;
            sum_weight=0.0;
        }

        void begin(){
            //tbd
        }

        void update(){
            for (index=0;index<LINE_SENSOR_NUMBER;index++){
                values[index]=analogRead(sensor_pins[index]);
            }
        }

        void get(int & left, int & center, int & right){
            left=values[LINE_SENSOR_LEFT];
            center=values[LINE_SENSOR_CENTER];
            right=values[LINE_SENSOR_RIGHT];
        }

        int getLeft(){
            return values[LINE_SENSOR_LEFT];
        }

        int getCenter(){
            return values[LINE_SENSOR_CENTER];
        }

        int getRight(){
            return values[LINE_SENSOR_RIGHT];
        }

        void updateCentroid(){
            sum_values=0.0;
            sum_weight=0.0;
            for (index=0;index<LINE_SENSOR_NUMBER; index++){                                // weighted average
                sum_values+=values[index]*(index+1);
                sum_weight+=values[index];
            }
            if (sum_weight==0.0){                                                           // divide by zero protection
                centroid=0.0;
            }else{
                centroid=sum_values/sum_weight;
                centroid=-centroid+2.0;                                                     // so it is right on robot axis Y
            }
        }

        float getCentroid(){
            return centroid;
        }

};


#endif