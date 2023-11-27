#ifndef __SENSOR_TOF_MATRIX_H__
#define __SENSOR_TOF_MATRIX_H__

#include "Arduino.h"
#include "Wire.h"
#include "vl53l7cx_class.h"


class SensorTofMatrix{
    private:
        VL53L7CX * sensor;
        TwoWire * wire;
    public:
        SensorTofMatrix(TwoWire * _wire, const uint8_t lpn_pin, const uint8_t i2c_rst_pin){
            wire=_wire;
            sensor = new VL53L7CX(wire,lpn_pin,i2c_rst_pin);
        }

        int begin(){
            //wire->begin();
            sensor->begin();
            sensor->init_sensor();
            sensor->vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4);
            sensor->vl53l7cx_start_ranging();
        }

        void print(){
            int size=4;
            VL53L7CX_ResultsData Results;
            uint8_t NewDataReady = 0;
            uint8_t status;
            do {
                status = sensor->vl53l7cx_check_data_ready(&NewDataReady);
            } while (!NewDataReady);

            if ((!status) && (NewDataReady != 0)) {
                status = sensor->vl53l7cx_get_ranging_data(&Results);
                for (int y=0; y<size; y++){
                    for (int x=0; x<size; x++){
                        Serial.print((int)Results.distance_mm[x+y*size]);
                        Serial.print(" ");
                    }  
                    Serial.println();
                }
                Serial.println();
            }
        }

        /*void setResolution(){
            sensor->setRes
        }*/
};

#endif