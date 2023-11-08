#ifndef __PINOUT_DEFINITIONS_H__
#define __PINOUT_DEFINITIONS_H__


// Motors
#define MOTORS_ENABLE PD2

// Right Motor
#define ENC_RIGHT_TIMER TIM5
#define ENC_RIGHT_A PA0
#define ENC_RIGHT_B PA1
#define MOTOR_RIGHT_A PA2
#define MOTOR_RIGHT_A_CH 3
#define MOTOR_RIGHT_B PA15
#define MOTOR_RIGHT_B_CH 1

// Left Motor
#define ENC_LEFT_TIMER TIM3
#define ENC_LEFT_A PC6
#define ENC_LEFT_B PC7
#define MOTOR_LEFT_A PA3
#define MOTOR_LEFT_A_CH 4
#define MOTOR_LEFT_B PB3
#define MOTOR_LEFT_B_CH 2


// Leds
#define LED_BUILTIN PC8
#define LED_1_RED PB12
#define LED_1_GREEN PB13
#define LED_1_BLUE PB14
#define LED_2_RED PC12
#define LED_2_GREEN PB4
#define LED_2_BLUE PB5


// Analog RC Servos
#define SERVO_A PC9
#define SERVO_B PA8


// APDS9960
#define APDS_LED PB6
#define APDS_INT PC10

// I2C ports
#define I2C_1_SDA PB7
#define I2C_1_SCL PB8
#define SELECT_I2C_BUS PB2

#define I2C_2_SDA PB9
#define I2C_2_SCL PB10
#define ARDUINO_ROBOT_ADDRESS 0xAB




#endif