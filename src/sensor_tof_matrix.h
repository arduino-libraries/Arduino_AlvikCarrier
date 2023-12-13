#ifndef __SENSOR_TOF_MATRIX_H__
#define __SENSOR_TOF_MATRIX_H__

#include "Arduino.h"
#include "Wire.h"
#include "vl53l7cx_class.h"

#define DEFAULT_RANGING_FREQ_4X4    30
#define DEFAULT_RANGING_FREQ_8X8    10
#define WIRE_BASE_CLOCK 400000
#define WIRE_BOOST_CLOCK 1000000

class SensorTofMatrix{
    private:
        VL53L7CX * _sensor;
        TwoWire * _wire;
        VL53L7CX_ResultsData results;
        int _size;
        int _ranging_freq;
        bool _wire_boost;
        uint32_t _wire_base_clock;
        
    public:
        int top;
        int bottom;
        int left;
        int right;
        int center_left;
        int center_right;
        int center;

        SensorTofMatrix(TwoWire * wire, const uint8_t lpn_pin, const uint8_t i2c_rst_pin, const int size=4, const int ranging_freq=-1, const bool wire_boost = true, const uint32_t wire_base_clock=WIRE_BASE_CLOCK){
            _wire=wire;
            _sensor = new VL53L7CX(_wire,lpn_pin,i2c_rst_pin);
            _size = size;
            _ranging_freq = ranging_freq;
            _wire_boost=wire_boost;
            _wire_base_clock=wire_base_clock;
        }

        int begin(){
            int out = 0;
            _wire->begin();
            Serial.println("wire begin done");
            if (_wire_boost){
                _wire->setClock(WIRE_BOOST_CLOCK);
            }
            out |= _sensor->begin();
            Serial.println("sensor begin done");
            out |= _sensor->init_sensor();
            Serial.println("sensor begin done");
            if (_size == 8){
                out |= _sensor->vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_8X8);
                if (_ranging_freq > 0 ) {
                    out |= _sensor->vl53l7cx_set_ranging_frequency_hz(_ranging_freq);
                } else {
                    out |= _sensor->vl53l7cx_set_ranging_frequency_hz(DEFAULT_RANGING_FREQ_8X8);
                }
            } else {
                out |= _sensor->vl53l7cx_set_resolution(VL53L7CX_RESOLUTION_4X4);
                if (_ranging_freq > 0 ) {
                    out |= _sensor->vl53l7cx_set_ranging_frequency_hz(_ranging_freq);
                } else {
                    out |= _sensor->vl53l7cx_set_ranging_frequency_hz(DEFAULT_RANGING_FREQ_4X4);
                }
            }
            out |= _sensor->vl53l7cx_start_ranging();

            if (_wire_boost){
                _wire->setClock(_wire_base_clock);
            }
            return out;
        }

        void print(){
            VL53L7CX_ResultsData Results;
            uint8_t NewDataReady = 0;
            uint8_t status;
            do {
                status = _sensor->vl53l7cx_check_data_ready(&NewDataReady);
            } while (!NewDataReady);

            if ((!status) && (NewDataReady != 0)) {
                status = _sensor->vl53l7cx_get_ranging_data(&Results);
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
            bool return_value=true;

            if (_wire_boost){
                _wire->setClock(WIRE_BOOST_CLOCK);
            }

            status = _sensor->vl53l7cx_check_data_ready(&NewDataReady);

            if ((!status) && (NewDataReady != 0)) {
                status = _sensor->vl53l7cx_get_ranging_data(&results);
            } else {
                return_value = false;
            }

            if (_wire_boost){
                _wire->setClock(WIRE_BASE_CLOCK);
            }

            return return_value;
        }

        bool update_rois() {
            bool out = update();
            top = get_avg_range_top_mm();
            bottom = get_max_range_bottom_mm();
            left = get_avg_range_left_mm();
            right = get_avg_range_right_mm();
            center_left = get_avg_range_center_left_mm();
            center_right = get_avg_range_center_right_mm();
            center = get_avg_range_center_mm();
            return out;
        }

        int get_min_range_top_mm() {

            int16_t top_min = results.distance_mm[0];

            for (int i=0; i < (_size==4?4:16) ;i++) {
                top_min = min(top_min, results.distance_mm[i]);
            }

            return top_min;
        }

        int get_avg_range_top_mm() {

            int16_t _avg = 0;
            uint8_t n = 0;

            for (int i=0; i < (_size==4?4:16) ;i++) {
                _avg += results.distance_mm[i];
                n++;
            }

            return _avg/n;
        }

        int get_max_range_bottom_mm() {

            int16_t bottom_max = results.distance_mm[0];

            for (int i=(_size==4?12:48); i < (_size==4?15:63) ;i++) {
                bottom_max = max(bottom_max, results.distance_mm[i]);
            }

            return bottom_max;
        }

        int get_avg_range_bottom_mm() {

            int16_t _avg = 0;
            uint8_t n = 0;

            for (int i=(_size==4?12:48); i < (_size==4?15:63) ;i++) {
                _avg += results.distance_mm[i];
                n++;
            }

            return _avg/n;
        }

        int get_min_range_left_mm() {

            int16_t _min = results.distance_mm[0];

            for (int i=0; i < (_size==4?16:64) ;i+=_size) {
                _min = min(_min, results.distance_mm[i]);
                if (_size==8) {
                    _min = min(_min, results.distance_mm[i+1]);
                }
            }

            return _min;
        }

        int get_min_range_right_mm() {

            int16_t _min = results.distance_mm[0];

            for (int i=(_size==4?3:6); i < (_size==4?16:64) ;i+=_size) {
                _min = min(_min, results.distance_mm[i]);
                if (_size==8) {
                    _min = min(_min, results.distance_mm[i+1]);
                }
            }

            return _min;
        }

        int get_min_range_center_left_mm() {

            int16_t _min = results.distance_mm[0];

            for (int i=(_size==4?5:18); i < (_size==4?13:50) ;i+=_size) {
                _min = min(_min, results.distance_mm[i]);
                if (_size==8) {
                    _min = min(_min, results.distance_mm[i+1]);
                }
            }

            return _min;
        }

        int get_min_range_center_right_mm() {

            int16_t _min = results.distance_mm[0];

            for (int i=(_size==4?6:20); i < (_size==4?14:52) ;i+=_size) {
                _min = min(_min, results.distance_mm[i]);
                if (_size==8) {
                    _min = min(_min, results.distance_mm[i+1]);
                }
            }

            return _min;
        }

        int get_min_range_center_mm() {

            int16_t _min = results.distance_mm[0];

            for (int i=(_size==5?6:19); i < (_size==4?13:51) ;i+=_size) {
                _min = min(_min, results.distance_mm[i]);
                _min = min(_min, results.distance_mm[i+1]);               
            }

            return _min;
        }

        // avgs

        int get_avg_range_left_mm() {

            int16_t _avg = 0;
            uint8_t n = 0;

            for (int i=0; i < (_size==4?16:64) ;i+=_size) {
                _avg += results.distance_mm[i];
                n++;
                if (_size==8) {
                    _avg += results.distance_mm[i+1];
                    n++;
                }
            }

            return _avg/n;
        }

        int get_avg_range_right_mm() {

            int16_t _avg = 0;
            uint8_t n = 0;

            for (int i=(_size==4?3:6); i < (_size==4?16:64) ;i+=_size) {
                _avg += results.distance_mm[i];
                n++;
                if (_size==8) {
                    _avg += results.distance_mm[i+1];
                    n++;
                }
            }

            return _avg/n;
        }

        int get_avg_range_center_left_mm() {

            int16_t _avg = 0;
            uint8_t n = 0;

            for (int i=(_size==4?5:18); i < (_size==4?13:50) ;i+=_size) {
                _avg += results.distance_mm[i];
                n++;
                if (_size==8) {
                    _avg += results.distance_mm[i+1];
                    n++;
                }
            }

            return _avg/n;
        }

        int get_avg_range_center_right_mm() {

            int16_t _avg = 0;
            uint8_t n = 0;

            for (int i=(_size==4?6:20); i < (_size==4?14:52) ;i+=_size) {
                _avg += results.distance_mm[i];
                n++;
                if (_size==8) {
                    _avg += results.distance_mm[i+1];
                    n++;
                }
            }

            return _avg/n;
        }

        int get_avg_range_center_mm() {

            int16_t _avg = 0;
            uint8_t n = 0;

            for (int i=(_size==4?6:19); i < (_size==4?13:51) ;i+=_size) {
                _avg += results.distance_mm[i];
                n++;
                _avg += results.distance_mm[i+1];
                n++;
            }

            return _avg/n;
        }

};

#endif