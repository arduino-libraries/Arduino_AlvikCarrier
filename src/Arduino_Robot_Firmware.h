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
#include "LSM6DSOSensor.h"
#include "motion_fx.h"


class Arduino_Robot_Firmware{
    private:
        RGBled * led1;
        RGBled * led2;


        APDS9960 * apds9960;
        int bottom_red, bottom_green, bottom_blue, bottom_clear, bottom_proximity;


        Servo * servo_A;
        Servo * servo_B;


        MAX17332 * bms;
        float voltage, state_of_charge; 


        AT42QT2120 * touch_sensor;
        AT42QT2120::Status touch_status;
        uint8_t touch_value;


        LSM6DSOSensor * imu;
        int32_t accelerometer[3];
        int32_t gyroscope[3];

        float imu_delta_time;
        MFX_knobs_t iKnobs;
        MFX_knobs_t *ipKnobs;
        uint8_t mfxstate[STATE_SIZE];
        MFX_input_t imu_data;
        MFX_output_t filter_data;
        uint16_t sample_to_discard;

        uint8_t version_high;
        uint8_t version_mid;
        uint8_t version_low;


    public:        
    
        MotorControl * motor_control_right;
        MotorControl * motor_control_left;



        DCmotor * motor_left;
        DCmotor * motor_right;
        Encoder * encoder_left;
        Encoder * encoder_right;


        TwoWire * wire;
        TwoWire * ext_wire;
        HardwareSerial * serial;



        Arduino_Robot_Firmware();

        int begin();


        void getVersion(uint8_t &high_byte, uint8_t &mid_byte, uint8_t &low_byte);


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
        bool setRpmLeft(const float rpm);
        float getRpmLeft();
        bool setRpmRight(const float rpm);
        float getRpmRight();
        bool setRpm(const float left, const float right);
        void getRpm(float & left, float & right);
        void setKPidRight(const float kp, const float ki, const float kd);
        void setKPidLeft(const float kp, const float ki, const float kd);


        // Touch
        int beginTouch();
        void updateTouch();
        bool getAnyTouchPressed();
        bool getTouchKey(const uint8_t key);
        uint8_t getTouchKeys();
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
        void setLedLeft(const uint32_t color);
        void setLedLeft(const uint32_t red, const uint32_t green, const uint32_t blue);
        void setLedLeftRed(const uint32_t red);
        void setLedLeftGreen(const uint32_t green);
        void setLedLeftBlue(const uint32_t blue);
        void setLedRight(const uint32_t color);
        void setLedRight(const uint32_t red, const uint32_t green, const uint32_t blue);
        void setLedRightRed(const uint32_t red);
        void setLedRightGreen(const uint32_t green);
        void setLedRightBlue(const uint32_t blue);
        void setLeds(const uint32_t color);
        void setLeds(const uint32_t red, const uint32_t green, const uint32_t blue);
        void setAllLeds(const uint8_t value);


        // Imu
        int beginImu();
        void updateImu();
        float getAccelerationX();
        float getAccelerationY();
        float getAccelerationZ();
        float getAngularVelocityX();
        float getAngularVelocityY();
        float getAngularVelocityZ();
        float getRoll();
        float getPitch();
        float getYaw();

        void errorLed(const int error_code);




        


};

#endif