#ifndef __ARDUINO_ROBOT_FIRMWARE_H__
#define __ARDUINO_ROBOT_FIRMWARE_H__

#include "Arduino.h"
#include "pinout_definitions.h"
#include "HAL_custom_init.h"
#include "encoder.h"
#include "rgb_led.h"
#include "dcmotor.h"
#include "motor_control.h"

class Arduino_Robot_Firmware{
    private:
        
    public:
        RGBled * led1;
        RGBled * led2;
        DCmotor * motor_left;
        DCmotor * motor_right;
        Encoder * encoder_left;
        Encoder * encoder_right;


        Arduino_Robot_Firmware(){
            led1 = new RGBled(LED_1_RED,LED_1_GREEN,LED_1_BLUE);
            led2 = new RGBled(LED_2_RED,LED_2_GREEN,LED_2_BLUE);

            motor_left = new DCmotor(MOTOR_LEFT_A,MOTOR_LEFT_A_CH, MOTOR_LEFT_B, MOTOR_LEFT_B_CH,true);
            motor_right = new DCmotor(MOTOR_RIGHT_A,MOTOR_RIGHT_A_CH,MOTOR_RIGHT_B,MOTOR_RIGHT_B_CH);

            encoder_left = new Encoder(TIM3);
            encoder_right = new Encoder(TIM5);

        }

        int begin(){
            // setup alternate functions
            AF_Tim2_pwm();
            AF_Tim5_pins_encoder();
            AF_Tim3_pins_encoder();

            // turn off leds
            led1->clear();
            led2->clear();

            motor_left->begin();
            motor_right->begin();
            motor_left->stop();
            motor_right->stop();

            encoder_left->begin();
            encoder_right->begin();

            return 0;
        }
};

#endif