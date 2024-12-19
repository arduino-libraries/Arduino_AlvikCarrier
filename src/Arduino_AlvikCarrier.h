/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#ifndef __ARDUINO_ALVIKCARRIER_H__
#define __ARDUINO_ALVIKCARRIER_H__

#include "Arduino.h"
#include "./definitions/pinout_definitions.h"
#include "./motor_control/encoder.h"
#include "./utilities/rgb_led.h"
#include "./motor_control/dcmotor.h"
#include "./motor_control/motor_control.h"
#include "Arduino_APDS9960.h"
#include <Servo.h>
#include "Arduino_MAX17332.h"
#include "AT42QT2120.h"
#include "LSM6DSOSensor.h"
#include "motion_fx.h"
#include "./robotics/kinematics.h"



class Arduino_AlvikCarrier{
    private:
        RGBled * led1;
        RGBled * led2;


        APDS9960 * apds9960;
        int bottom_red, bottom_green, bottom_blue, bottom_clear, bottom_proximity;
        bool illuminator_state;


        Servo * servo_A;
        Servo * servo_B;


        MAX17332 * bms;
        float voltage, state_of_charge, charging; 


        AT42QT2120 * touch_sensor;
        AT42QT2120::Status touch_status;
        uint8_t touch_value;


        LSM6DSOSensor * imu;
        int32_t accelerometer[3];
        int32_t gyroscope[3];
        LSM6DSO_Event_Status_t imu_status;


        float imu_delta_time;
        MFX_knobs_t iKnobs;
        MFX_knobs_t *ipKnobs;
        uint8_t mfxstate[STATE_SIZE];
        MFX_input_t imu_data;
        MFX_output_t filter_data;
        uint16_t sample_to_discard;
        bool is_shaking;
        bool first_wakeup;
        unsigned long shake_time, shake_time_sig;
        uint16_t shake_counter;
        uint8_t tilt_status;
        uint8_t xl, xh, yl, yh, zl, zh;
        unsigned long tilt_time;
        uint8_t tilt_filter;
        uint8_t tmp_tilt_status;

        uint8_t version_high;
        uint8_t version_mid;
        uint8_t version_low;


        uint8_t kinematics_movement;
        bool kinematics_achieved;
        float previous_travel;
        float move_direction;
        float actual_direction;

        PidController * rotate_pid;
        PidController * move_pid;


        bool prev_illuminator_state;
        uint8_t behaviours;
        bool first_lift;
        unsigned long battery_alert_time;
        unsigned long battery_alert_wave;
        bool battery_alert;




    public:        
        Kinematics * kinematics;

        MotorControl * motor_control_right;
        MotorControl * motor_control_left;


        DCmotor * motor_left;
        DCmotor * motor_right;
        Encoder * encoder_left;
        Encoder * encoder_right;


        TwoWire * wire;
        TwoWire * ext_wire;
        HardwareSerial * serial;



        Arduino_AlvikCarrier();

        int begin();                                                                    // initialize the robot


        void getVersion(uint8_t &high_byte, uint8_t &mid_byte, uint8_t &low_byte);      // get firmware version


        // Color sensor, APDS9960
        int beginAPDS();                                                                // initialize all components required by color detection
        void updateAPDS();                                                              // refresh data
        void setIlluminator(uint8_t value);                                             // set white leds       
        void enableIlluminator();                                                       // white leds on
        void disableIlluminator();                                                      // white leds off
        int getRed();                                                                   // red value 0-4095
        int getGreen();                                                                 // green value 0-4095
        int getBlue();                                                                  // blue value 0-4095
        int getClear();                                                                 // clear value 0-4095
        int getProximity();                                                             // proximity value 0-127


        // Servo
        int beginServo();                                                               // initialize Servo interfaces
        void setServoA(int position);                                                   // 0°-180° servo position
        void setServoB(int position);                                                   // 0°-180° servo position


        // I2C select
        int beginI2Cselect();                                                           // initialize I2C bus selector
        void setExternalI2C(uint8_t state);                                             // set A4,A5 connection on I2C bus 2
        void connectExternalI2C();                                                      // allow A4,A5 on nano connector to be attached to I2C bus 2
        void disconnectExternalI2C();                                                   // disable the connection on A4,A5


        // BMS, MAX17332
        int beginBMS();                                                                 // initialize BMS                        
        void updateBMS();                                                               // update the BMS
        float getBatteryVoltage();                                                      // get Voltage
        float getBatteryChargePercentage();                                             // get battery percentage
        float isBatteryCharging();                                                      // return -1 if battery discharge or 1 if charging
        bool isBatteryAlert();                                                          // return true if battery is LOW and alert behaviour is setted, otherwise false



        // Motors
        int beginMotors();                                                              // initialize motor controls
        void updateMotors();                                                            // update motor control
        bool setRpmLeft(const float rpm);                                               // set RPM of left motor
        float getRpmLeft();                                                             // get RPM of left motor
        bool setRpmRight(const float rpm);                                              // set RPM of right motor
        float getRpmRight();                                                            // get RPM of right motor
        bool setRpm(const float left, const float right);                               // sets RPMs on left and right wheels
        void getRpm(float & left, float & right);                                       // get RPMs on left and right wheels
        void setKPidLeft(const float kp, const float ki, const float kd);               // set PID parameters for left wheel
        void setKPidRight(const float kp, const float ki, const float kd);              // set PID parameters for right wheel
        void setPositionLeft(const float degrees);                                      // set position in degrees on left wheel
        float getPositionLeft();                                                        // get left wheel position in degrees
        bool isPositionLeftReached();                                                   // return true if left wheel position is reached
        void setPositionRight(const float degrees);                                     // set position in degrees on right wheel
        float getPositionRight();                                                       // get right wheel position in degrees
        bool isPositionRightReached();                                                  // return true if right wheel position is reached
        void setPosition(const float left_deg, const float right_deg);                  // set positions on both wheels
        void getPosition(float & left_deg, float & right_deg);                          // get both wheels position
        bool isPositionReached();                                                       // return true if both wheels position are reached
        void resetPositionLeft(const float initial_position=0.0);                       // reset/set value of position for left wheel
        void resetPositionRight(const float initial_position=0.0);                      // reset/set value of position for right wheel
        void disablePositionControlLeft();                                              // disable the position control on left wheel
        void disablePositionControlRight();                                             // disable the position control on right wheel
        void disablePositionControl();                                                  // disable the position control on both wheels



        // Touch
        int beginTouch();                                                               // initialize touch
        void updateTouch();                                                             // update touch status
        bool getAnyTouchPressed();                                                      // get any touch pressed
        bool getTouchKey(const uint8_t key);                                            // return true if key touch is pressed
        uint8_t getTouchKeys(const bool single_touch = true);                           // return touched pads as byte
        bool getTouchUp();                                                              // get nav pad up
        bool getTouchRight();                                                           // get nav pad right
        bool getTouchDown();                                                            // get nav pad down
        bool getTouchLeft();                                                            // get nav pad left
        bool getTouchCenter();                                                          // get nav pad enter (center circle)
        bool getTouchOk();                                                              // get nav pad ok (right check)
        bool getTouchDelete();                                                          // get nav pad delete (right x)



        // Leds
        int beginLeds();                                                                // initialize leds
        void setLedBuiltin(const uint8_t value);                                        // set built-in led
        void setLedLeft(const uint32_t color);                                          // set left led by color
        void setLedLeft(const uint32_t red, const uint32_t green, const uint32_t blue); // set left led by RGB (only boolean values)
        void setLedLeftRed(const uint32_t red);                                         // set red left led
        void setLedLeftGreen(const uint32_t green);                                     // set green left led
        void setLedLeftBlue(const uint32_t blue);                                       // set blue left led
        void setLedRight(const uint32_t color);                                         // set right led by color
        void setLedRight(const uint32_t red, const uint32_t green, const uint32_t blue);// set right led by RGB (only boolean values)
        void setLedRightRed(const uint32_t red);                                        // set red right led
        void setLedRightGreen(const uint32_t green);                                    // set green right led
        void setLedRightBlue(const uint32_t blue);                                      // set blue right led
        void setLeds(const uint32_t color);                                             // set both leds by color
        void setLeds(const uint32_t red, const uint32_t green, const uint32_t blue);    // set both leds by RGB (only boolean values)
        void setAllLeds(const uint8_t value);                                           // set all leds by a byte


        // Imu
        int beginImu();                                                                 // initialize LSD6DSOX-
        void updateImu();                                                               // update accelerometer and gyroscope data. Update sensor fusion
        float getAccelerationX();                                                       // get acceleration on x axis
        float getAccelerationY();                                                       // get acceleration on y axis
        float getAccelerationZ();                                                       // get acceleration on z axis
        float getAngularVelocityX();                                                    // get angular velocity on x axis
        float getAngularVelocityY();                                                    // get angular velocity on y axis
        float getAngularVelocityZ();                                                    // get angular velocity on z axis
        float getRoll();                                                                // get robot roll
        float getPitch();                                                               // get robot pitch
        float getYaw();                                                                 // get robot yaw
        bool  isShaking();                                                              // get true if robot is shaking    
        uint8_t getMotion();                                                            // get tilt + shaking                                                    

        void errorLed(const int error_code);                                            // error routine, locks on code blinking led


        // Kinematics
        void updateKinematics();                                                        // update pose/velocity of the robot and controls
        void drive(const float linear, const float angular);                            // set mm/s and deg/s of the robot
        float getLinearVelocity();                                                      // get linear velocity of the robot in mm/s
        float getAngularVelocity();                                                     // get angular velocity of the robot in deg/s
        float getX();                                                                   // absolute position in mm
        float getY();                                                                   // absolute position in mm
        float getTheta();                                                               // angle in deg
        float getSinTheta();                                                            // stored sin of current theta
        float getCosTheta();                                                            // stored cos of current theta
        void resetPose(const float x0=0.0, const float y0=0.0, const float theta0=0.0); // reset pose in kinematics

        void move(const float distance);                                                // move of distance millimeters
        void rotate(const float angle);                                                 // rotate of angle degrees
        
        
        void lockingRotate(const float angle);                                          // rotate of angle degrees     
        void lockingMove(const float distance);                                         // move of distance millimeters

        void disableKinematicsMovement();                                               // disable movements that requires kinematics
        bool isTargetReached();                                                         // get true if a movement is accomplished
        uint8_t getKinematicsMovement();                                                // get which kind of motion is running in kinematic control


        // Behaviours
        void beginBehaviours();                                                         // internal behaviours of the robot
        void updateBehaviours();                                                        // updated internal behaviours
        void setBehaviour(const uint8_t behaviour, const bool enable);                  // set a behaviour
        bool isLifted();                                                                // is robot lifted?


        // Utilities
        void getSerialNumber(char * sn);                                                // get STM32 serial number

};

#endif