#ifndef __ROBOT_DEFINITIONS_H__
#define __ROBOT_DEFINITIONS_H__

#define CONTROL_LIMIT 4096              // PWM resolution

#define MOTOR_LIMIT 100.0               // Mechanical RPM limit speed of used motors
#define MOTOR_CPR 6.0                   // Resolution of the encoder
#define MOTOR_GEAR_RATIO 150.0          // Gear ratio of the motor

const float MOTOR_RATIO = MOTOR_CPR*MOTOR_GEAR_RATIO;

#define MOTOR_KP_RIGHT 60.0
#define MOTOR_KI_RIGHT 0.01
#define MOTOR_KD_RIGHT 0.4
#define MOTOR_CONTROL_PERIOD 0.01

#endif