/*
  This file is part of the Arduino Alvik library.
  Copyright (c) 2023 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __ROBOT_DEFINITIONS_H__
#define __ROBOT_DEFINITIONS_H__

// Motor Control and mechanical parameters
#define CONTROL_LIMIT 4096              // PWM resolution

#define MOTOR_LIMIT 80.0                // Mechanical RPM limit speed of used motors
#define MOTOR_CPR 6.0                   // Resolution of the encoder
#define MOTOR_GEAR_RATIO 150.0          // Gear ratio of the motor

const float MOTOR_RATIO = MOTOR_CPR*MOTOR_GEAR_RATIO;

#define MOTOR_KP_RIGHT 32.0
#define MOTOR_KI_RIGHT 450.0
#define MOTOR_KD_RIGHT 0.0

//30, 450, 0.0
//120,300,1,0
#define MOTOR_CONTROL_PERIOD 0.02


// Sensor fusioning parameters
#define FROM_MG_TO_G  0.001f
#define FROM_G_TO_MG  1000.0f
#define FROM_MDPS_TO_DPS  0.001f
#define FROM_DPS_TO_MDPS  1000.0f
#define MOTION_FX_FREQ  100U
const float MOTION_FX_PERIOD = (1000U / MOTION_FX_FREQ);
#define MOTION_FX_ENGINE_DELTATIME  0.01f
#define STATE_SIZE                      (size_t)(2432)
#define SAMPLETODISCARD                 15
#define GBIAS_ACC_TH_SC                 (2.0f*0.000765f)
#define GBIAS_GYRO_TH_SC                (2.0f*0.002f)
#define DECIMATION                      1U
#define STATE_SIZE                      (size_t)(2432)

#define VERSION_BYTE_HIGH 0
#define VERSION_BYTE_MID 0
#define VERSION_BYTE_LOW 5

#endif