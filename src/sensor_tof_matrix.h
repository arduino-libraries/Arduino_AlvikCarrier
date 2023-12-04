#ifndef __SENSOR_TOF_MATRIX_H__
#define __SENSOR_TOF_MATRIX_H__

#include "Arduino.h"
#include "Wire.h"
#include "vl53l7cx_class.h"


class SensorTofMatrix{
    private:
        VL53L7CX * sensor;
        TwoWire * wire;
        VL53L7CX_ResultsData results;
        int _size;
    public:
        SensorTofMatrix(TwoWire * _wire, const uint8_t lpn_pin, const uint8_t i2c_rst_pin, const int size=4){
            wire=_wire;
            sensor = new VL53L7CX(wire,lpn_pin,i2c_rst_pin);
            _size = size;
        }

        int begin(){
            int out = 0;
            wire->begin();
            out |= sensor->begin();
            out |= sensor->init_sensor();
            if (_size == 8){
                out |= sensor->vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_8X8);
            } else {
                out |= sensor->vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4);
            }
            out |= sensor->vl53l7cx_start_ranging();
            return out;
        }

        void print(){
            VL53L7CX_ResultsData Results;
            uint8_t NewDataReady = 0;
            uint8_t status;
            do {
                status = sensor->vl53l7cx_check_data_ready(&NewDataReady);
            } while (!NewDataReady);

            if ((!status) && (NewDataReady != 0)) {
                status = sensor->vl53l7cx_get_ranging_data(&Results);
                for (int y=0; y<_size; y++){
                    for (int x=0; x<_size; x++){
                        Serial.print((int)Results.distance_mm[x+y*_size]);
                        Serial.print(" ");
                    }  
                    Serial.println();
                }
                Serial.println();
            }
        }

        bool update() {
            uint8_t NewDataReady = 0;
            uint8_t status;

            status = sensor->vl53l7cx_check_data_ready(&NewDataReady);

            if ((!status) && (NewDataReady != 0)) {
                status = sensor->vl53l7cx_get_ranging_data(&results);
            } else {
                return false;
            }

            return true;
        }

        int get_min_range_top_mm() {
            update();

            int16_t top_min = results.distance_mm[0];

            for (int i=0; i < (_size==4?4:16) ;i++) {
                top_min = min(top_min, results.distance_mm[i]);
            }

            return top_min;
        }

        int get_max_range_bottom_mm() {
            update();

            int16_t bottom_max = results.distance_mm[0];

            for (int i=(_size==4?12:48); i < (_size==4?15:63) ;i++) {
                bottom_max = max(bottom_max, results.distance_mm[i]);
            }

            return bottom_max;
        }

        int get_min_range_right_mm() {
            update();

            int16_t top_min = results.distance_mm[0];

            for (int i=0; i < (_size==4?16:64) ;i+=_size) {
                top_min = min(top_min, results.distance_mm[i]);
                if (_size==8) {
                    top_min = min(top_min, results.distance_mm[i+1]);
                }
            }

            return top_min;
        }

        /*void setResolution(){
            sensor->setRes
        }*/
};

#endif