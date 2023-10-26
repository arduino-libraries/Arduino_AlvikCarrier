#include "Arduino_Robot_Firmware.h"
#include "HAL_custom_init.h"



Arduino_Robot_Firmware::Arduino_Robot_Firmware(){
    // I2C internal bus
    wire = new TwoWire(I2C_1_SDA, I2C_1_SCL);

    // RGB leds
    led1 = new RGBled(LED_1_RED,LED_1_GREEN,LED_1_BLUE);
    led2 = new RGBled(LED_2_RED,LED_2_GREEN,LED_2_BLUE);

    // motors
    motor_left = new DCmotor(MOTOR_LEFT_A,MOTOR_LEFT_A_CH, MOTOR_LEFT_B, MOTOR_LEFT_B_CH,true);
    motor_right = new DCmotor(MOTOR_RIGHT_A,MOTOR_RIGHT_A_CH,MOTOR_RIGHT_B,MOTOR_RIGHT_B_CH);

    // encoders
    encoder_left = new Encoder(TIM3);
    encoder_right = new Encoder(TIM5);

    // color sensor
    apds9960 = new APDS9960(*wire,APDS_INT);

    // servo
    servo_A = new Servo();
    servo_B = new Servo();
}

int Arduino_Robot_Firmware::begin(){
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

    wire->begin();

    beginAPDS();
    beginServo();
    beginI2Cselect();
    connectExternalI2C();

    return 0;
}


/******************************************************************************************************/
/*                                      Color sensor, APDS9960                                        */
/******************************************************************************************************/

int Arduino_Robot_Firmware::beginAPDS(){
    pinMode(APDS_LED,OUTPUT);
    enableIlluminator();
    apds9960->begin();
    return 0;
}

void Arduino_Robot_Firmware::updateAPDS(){
    if (apds9960->proximityAvailable()){
        bottom_proximity=apds9960->readProximity();
    }
    //digitalWrite(APDS_LED,HIGH);
    if (apds9960->colorAvailable()){
        apds9960->readColor(bottom_red,bottom_green,bottom_blue,bottom_clear);
    }
    //digitalWrite(APDS_LED,LOW);
}

void Arduino_Robot_Firmware::setIlluminator(uint8_t value){
    digitalWrite(APDS_LED,value);
}

void Arduino_Robot_Firmware::enableIlluminator(){
    setIlluminator(HIGH);
}

void Arduino_Robot_Firmware::disableIlluminator(){
    setIlluminator(LOW);
}

int Arduino_Robot_Firmware::getRed(){
    return bottom_red;
}

int Arduino_Robot_Firmware::getGreen(){
    return bottom_green;
}

int Arduino_Robot_Firmware::getBlue(){
    return bottom_blue;
}

int Arduino_Robot_Firmware::getProximity(){
    return bottom_proximity;
}

/******************************************************************************************************/
/*                                        RC Servo A & B                                              */
/******************************************************************************************************/

int Arduino_Robot_Firmware::beginServo(){
    servo_A->attach(SERVO_A);
    servo_B->attach(SERVO_B);
    return 0;
}

void Arduino_Robot_Firmware::setServoA(int position){
    servo_A->write(position);
}

void Arduino_Robot_Firmware::setServoB(int position){
    servo_B->write(position);
}

/******************************************************************************************************/
/*                                        RC Servo A & B                                              */
/******************************************************************************************************/

int Arduino_Robot_Firmware::beginI2Cselect(){
    pinMode(SELECT_I2C_BUS,OUTPUT);
}

void Arduino_Robot_Firmware::setExternalI2C(uint8_t state){
    digitalWrite(SELECT_I2C_BUS,state);
}

void Arduino_Robot_Firmware::connectExternalI2C(){
    setExternalI2C(LOW);
}

void Arduino_Robot_Firmware::disconnectExternalI2C(){
    setExternalI2C(HIGH);
}