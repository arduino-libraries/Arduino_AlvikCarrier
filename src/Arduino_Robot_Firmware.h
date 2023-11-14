#ifndef __ARDUINO_ROBOT_FIRMWARE_H__
#define __ARDUINO_ROBOT_FIRMWARE_H__

#include "Arduino.h"
#include "pinout_definitions.h"
#include "encoder.h"
#include "rgb_led.h"
#include "dcmotor.h"
#include "motor_control.h"
#include "Arduino_APDS9960.h"
#include <Servo.h>
#include "Arduino_MAX17332.h"
#include "AT42QT2120.h"

class Arduino_Robot_Firmware{
    private:
        APDS9960 * apds9960;
        int bottom_red, bottom_green, bottom_blue, bottom_clear, bottom_proximity;

        Servo * servo_A;
        Servo * servo_B;


        MAX17332 * bms;
        float voltage, state_of_charge; 

        MotorControl * motor_control_right;

        AT42QT2120 * touch_sensor;
        AT42QT2120::Status touch_status;



    public:
        RGBled * led1;
        RGBled * led2;
        DCmotor * motor_left;
        DCmotor * motor_right;
        Encoder * encoder_left;
        Encoder * encoder_right;


        TwoWire * wire;
        TwoWire * ext_wire;



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


        // Servo
        int beginServo();                           // initialize Servo interfaces
        void setServoA(int position);               // 0°-180° servo position
        void setServoB(int position);               // 0°-180° servo position


        // I2C select
        int beginI2Cselect();                       // initialize I2C bus selector
        void setExternalI2C(uint8_t state);         // set A4,A5 connection on I2C bus 2
        void connectExternalI2C();                  // allow A4,A5 on nano connector to be attached to I2C bus 2
        void disconnectExternalI2C();               // disable the connection on A4,A5


        // BMS, MAX17332
        int beginBMS();
        void updateBMS();
        float getBatteryVoltage();
        float getBatteryChargePercentage();


        // Motors
        int beginMotors();
        void updateMotors();
        bool setRpmRight(const float rpm);
        float getRpmRight();
        void setKPidRight(const float kp, const float ki, const float kd);


        // Touch
        int beginTouch();
        void updateTouch();
        bool getTouchPressed();
        bool getTouchKey(const uint8_t key);
        bool getTouchUp();
        bool getTouchRight();
        bool getTouchDown();
        bool getTouchLeft();
        bool getTouchEnter();
        bool getTouchOk();
        bool getTouchDelete();


        // Leds
        int beginLeds();
        void setLedBuiltin(const uint8_t value);



        


};

#endif