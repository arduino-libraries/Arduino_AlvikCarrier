#include "Arduino_Robot_Firmware.h"
#include "HAL_custom_init.h"



Arduino_Robot_Firmware::Arduino_Robot_Firmware(){
    // I2C internal bus
    wire = new TwoWire(I2C_1_SDA, I2C_1_SCL);

    // I2C external bus
    ext_wire = new TwoWire(I2C_2_SDA,I2C_2_SCL);

    // RGB leds
    led1 = new RGBled(LED_1_RED,LED_1_GREEN,LED_1_BLUE);
    led2 = new RGBled(LED_2_RED,LED_2_GREEN,LED_2_BLUE);

    // motors
    motor_left = new DCmotor(MOTOR_LEFT_A,MOTOR_LEFT_A_CH, MOTOR_LEFT_B, MOTOR_LEFT_B_CH,true);
    motor_right = new DCmotor(MOTOR_RIGHT_A,MOTOR_RIGHT_A_CH,MOTOR_RIGHT_B,MOTOR_RIGHT_B_CH);

    // encoders
    encoder_left = new Encoder(TIM3);
    encoder_right = new Encoder(TIM5);

    // motor control
    motor_control_right = new MotorControl(motor_right,encoder_right,MOTOR_KP_RIGHT,MOTOR_KI_RIGHT,MOTOR_KD_RIGHT,MOTOR_CONTROL_PERIOD);

    // color sensor
    apds9960 = new APDS9960(*wire,APDS_INT);

    // servo
    servo_A = new Servo();
    servo_B = new Servo();

    // bms
    bms = new MAX17332(*wire);

    // touch
    touch_sensor = new AT42QT2120(*wire);

    // imu
    imu = new LSM6DSOSensor(wire, LSM6DSO_I2C_ADD_L);

}

int Arduino_Robot_Firmware::begin(){
    beginLeds();

    // setup alternate functions
    AF_Tim2_pwm();
    AF_Tim5_pins_encoder();
    AF_Tim3_pins_encoder();


    motor_left->begin();
    motor_right->begin();
    motor_left->stop();
    motor_right->stop();

    encoder_left->begin();
    encoder_right->begin();


    wire->begin();
    wire->setClock(400000);
    
    connectExternalI2C();
    ext_wire->begin(ARDUINO_ROBOT_ADDRESS);

    //beginImu();
    
    if (beginAPDS()!=0){
        errorLed(ERROR_APDS);
    }
     
    beginServo();
    //beginI2Cselect();
    //connectExternalI2C();
    
    if (beginBMS()!=0){
        errorLed(ERROR_BMS);
    }
    
    if (beginTouch()!=0){
        //errorLed(ERROR_TOUCH);
    }
    

    return 0;
}


/******************************************************************************************************/
/*                                      Color sensor, APDS9960                                        */
/******************************************************************************************************/

int Arduino_Robot_Firmware::beginAPDS(){
    pinMode(APDS_LED,OUTPUT);
    enableIlluminator();
    if (!apds9960->begin()){
        return ERROR_APDS;
    }
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
/*                                        External I2C                                                */
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


/******************************************************************************************************/
/*                               Battery Management System, MAX17332                                  */
/******************************************************************************************************/

int Arduino_Robot_Firmware::beginBMS(){
    bms->begin();
    return 0;
}

void Arduino_Robot_Firmware::updateBMS(){
    voltage = bms->readVCell();
    state_of_charge = bms->readSoc();
}


float Arduino_Robot_Firmware::getBatteryVoltage(){
    return voltage;
}

float Arduino_Robot_Firmware::getBatteryChargePercentage(){
    return state_of_charge;
}


/******************************************************************************************************/
/*                                            Motor controls                                          */
/******************************************************************************************************/

int Arduino_Robot_Firmware::beginMotors(){
    motor_control_right->begin();
    
    return 0;
}

void Arduino_Robot_Firmware::updateMotors(){
    motor_control_right->update();
}

bool Arduino_Robot_Firmware::setRpmRight(const float rpm){
    return motor_control_right->setRPM(rpm);
}

float Arduino_Robot_Firmware::getRpmRight(){
    return motor_control_right->getRPM();
}

void Arduino_Robot_Firmware::setKPidRight(const float kp, const float ki, const float kd){
    motor_control_right->setKP(kp);
    motor_control_right->setKI(ki);
    motor_control_right->setKP(kp);
}


/******************************************************************************************************/
/*                                           Touch pads                                               */
/******************************************************************************************************/

int Arduino_Robot_Firmware::beginTouch(){
    touch_sensor->begin();
    if (!touch_sensor->communicating()){
        return ERROR_TOUCH;
    }

    /*  //NEED TO BE CHECKED
    AT42QT2120::KeyControl key_control;
    key_control.guard=1;
    touch_sensor->setKeyControl(1, key_control);
    */

    touch_sensor->reset();
    delay(2000);

    touch_sensor->triggerCalibration();
    delay(100);
    while (touch_sensor->calibrating()){
        delay(100);
    }
    return 0;
}

void Arduino_Robot_Firmware::updateTouch(){
    touch_status = touch_sensor->getStatus();
}

bool Arduino_Robot_Firmware::getTouchPressed(){
    if (touch_sensor->touched(touch_status,TOUCH_PAD_GUARD)){
        return true;
    }
    return false;
}

bool Arduino_Robot_Firmware::getTouchKey(const uint8_t key){
    if (touch_sensor->touched(touch_status,key)&&touch_sensor->touched(touch_status,TOUCH_PAD_GUARD)){
        return true;
    }
    return false;
}

bool Arduino_Robot_Firmware::getTouchUp(){
    return getTouchKey(TOUCH_PAD_UP);
}

bool Arduino_Robot_Firmware::getTouchRight(){
    return getTouchKey(TOUCH_PAD_RIGHT);
}

bool Arduino_Robot_Firmware::getTouchDown(){
    return getTouchKey(TOUCH_PAD_DOWN);
}

bool Arduino_Robot_Firmware::getTouchLeft(){
    return getTouchKey(TOUCH_PAD_LEFT);
}

bool Arduino_Robot_Firmware::getTouchEnter(){
    return getTouchKey(TOUCH_PAD_ENTER);
}

bool Arduino_Robot_Firmware::getTouchOk(){
    return getTouchKey(TOUCH_PAD_OK);
}

bool Arduino_Robot_Firmware::getTouchDelete(){
    return getTouchKey(TOUCH_PAD_DELETE);
}


/******************************************************************************************************/
/*                                               Leds                                                 */
/******************************************************************************************************/

int Arduino_Robot_Firmware::beginLeds(){
    pinMode(LED_BUILTIN,OUTPUT);
    // turn off leds
    led1->clear();
    led2->clear();
    return 0;
}

void Arduino_Robot_Firmware::setLedBuiltin(const uint8_t value){
    digitalWrite(LED_BUILTIN,value);
}

void Arduino_Robot_Firmware::setLedLeft(const uint32_t color){
    led1->set(color); 
}

void Arduino_Robot_Firmware::setLedLeft(const uint32_t red, const uint32_t green, const uint32_t blue){
    led1->set(red,green,blue); 
}

void Arduino_Robot_Firmware::setLedLeftRed(const uint32_t red){
    led1->setRed(red);
}

void Arduino_Robot_Firmware::setLedLeftGreen(const uint32_t green){
    led1->setGreen(green);
}

void Arduino_Robot_Firmware::setLedLeftBlue(const uint32_t blue){
    led1->setBlue(blue);
}

void Arduino_Robot_Firmware::setLedRight(const uint32_t color){
    led2->set(color);
}

void Arduino_Robot_Firmware::setLedRight(const uint32_t red, const uint32_t green, const uint32_t blue){
    led2->set(red,green,blue);
}

void Arduino_Robot_Firmware::setLedRightRed(const uint32_t red){
    led2->setRed(red);
}

void Arduino_Robot_Firmware::setLedRightGreen(const uint32_t green){
    led2->setGreen(green);
}

void Arduino_Robot_Firmware::setLedRightBlue(const uint32_t blue){
    led2->setBlue(blue);
}

void Arduino_Robot_Firmware::setLeds(const uint32_t color){
    setLedLeft(color);
    setLedRight(color);  
}

void Arduino_Robot_Firmware::setLeds(const uint32_t red, const uint32_t green, const uint32_t blue){
    setLedLeft(red,green,blue);
    setLedRight(red,green,blue);    
}

/******************************************************************************************************/
/*                                                IMU                                                 */
/******************************************************************************************************/

int Arduino_Robot_Firmware::beginImu(){
    imu->begin();
    imu->Set_X_ODR(100.0);
    imu->Set_X_FS(4);
    imu->Set_G_ODR(100.0);
    imu->Set_G_FS(2000);
    imu->Enable_X();
    imu->Enable_G();
}


/******************************************************************************************************/
/*                                               Error                                                */
/******************************************************************************************************/

void Arduino_Robot_Firmware::errorLed(const int error_code){
    while(true){
        for (int i = 0; i<error_code; i++){
            setLedBuiltin(HIGH);
            delay(100);
            setLedBuiltin(LOW);
            delay(500);
        }
        delay(5000);
    }
}