/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#ifndef __ROBOT_DEFINITIONS_H__
#define __ROBOT_DEFINITIONS_H__

// Robot parameters
#define WHEEL_TRACK_MM 89.0
#define WHEEL_DIAMETER_MM 34.0

// Motor Control and mechanical parameters
#define CONTROL_LIMIT 4095              // PWM resolution

#define MOTOR_LIMIT 70.0                // Mechanical RPM limit speed of used motors
#define MOTOR_CPR 6.0                   // Resolution of the encoder
#define MOTOR_GEAR_RATIO 150.0          // Gear ratio of the motor, maybe 150.58?

const float MOTOR_RATIO = MOTOR_CPR*MOTOR_GEAR_RATIO;

#define MOTOR_KP_DEFAULT 32.0
#define MOTOR_KI_DEFAULT 450.0
#define MOTOR_KD_DEFAULT 0.0
#define MOTOR_CONTROL_PERIOD 0.02
#define MOTOR_CONTROL_STEP 5.0

#define POSITION_KP_DEFAULT 1.0
#define POSITION_KI_DEFAULT 0.0
#define POSITION_KD_DEFAULT 0.0001
#define POSITION_CONTROL_PERIOD 0.02
#define POSITION_MAX_SPEED 30.0



// Kinematics control
#define ROTATE_KP_DEFAULT 4.0
#define ROTATE_KI_DEFAULT 0.01
#define ROTATE_KD_DEFAULT 0.001
#define ROTATE_CONTROL_PERIOD 0.02
#define ROTATE_MAX_SPEED 170.0
#define ROTATE_THRESHOLD 1

#define MOVE_KP_DEFAULT 4.0
#define MOVE_KI_DEFAULT 0.01
#define MOVE_KD_DEFAULT 0.001
#define MOVE_CONTROL_PERIOD 0.02
#define MOVE_MAX_SPEED 130.0
#define MOVE_THRESHOLD 3

#define MOVEMENT_DISABLED 0
#define MOVEMENT_ROTATE 1
#define MOVEMENT_MOVE 2


#define LIFT_THRESHOLD 80

#define ALL_BEHAVIOURS 255
#define LIFT_ILLUMINATOR 1






// Sensor fusioning parameters
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f
#define MOTION_FX_FREQ  100U
const float MOTION_FX_PERIOD = (1000U / MOTION_FX_FREQ);
#define MOTION_FX_ENGINE_DELTATIME  0.01f
#define STATE_SIZE                      (size_t)(2432)
#define MOTION_FX_SAMPLETODISCARD                 15
#define GBIAS_ACC_TH_SC                 (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC                (2.0f*0.002f)
#define MOTION_FX_DECIMATION                      1U


// Library version
#define VERSION_BYTE_HIGH 0
#define VERSION_BYTE_MID 3
#define VERSION_BYTE_LOW 4



#endif