#ifndef __ARDUINO_ROBOT_FIRMWARE_H__
#define __ARDUINO_ROBOT_FIRMWARE_H__

#include "Arduino.h"
#include "pinout_definitions.h"
#include "encoder.h"
#include "rgb_led.h"
#include "dcmotor.h"
#include "motor_control.h"
#include "Arduino_APDS9960.h"

class Arduino_Robot_Firmware{
    private:
        APDS9960 * apds9960;
        int bottom_red, bottom_green, bottom_blue, bottom_clear, bottom_proximity;

    public:
        RGBled * led1;
        RGBled * led2;
        DCmotor * motor_left;
        DCmotor * motor_right;
        Encoder * encoder_left;
        Encoder * encoder_right;
        TwoWire * wire;


        Arduino_Robot_Firmware();

        int begin();



        //custom class????

        // Color sensor, APDS9960
        int beginAPDS();                            // initialize all components required by color detection
        void updateAPDS();                          // refresh data
        void setIlluminator(uint8_t value);         // set white leds       
        void enableIlluminator();                   // white leds on
        void disableIlluminator();                  // white leds off
        int getRed();                               // red value 0-255
        int getGreen();                             // green value 0-255
        int getBlue();                              // blue value 0-255
        int getProximity();                         // proximity value 0-127
};

#endif