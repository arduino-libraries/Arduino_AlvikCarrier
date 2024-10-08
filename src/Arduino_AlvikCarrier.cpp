/*
    This file is part of the Arduino_AlvikCarrier library.

    Copyright (c) 2023 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#include "Arduino_AlvikCarrier.h"
#include "./utilities/HAL_custom_init.h"



Arduino_AlvikCarrier::Arduino_AlvikCarrier(){
    // I2C internal bus
    wire = new TwoWire(I2C_1_SDA, I2C_1_SCL);

    // I2C external bus
    ext_wire = new TwoWire(I2C_2_SDA, I2C_2_SCL);

    // uart to esp32
    serial = new HardwareSerial(UART_RX, UART_TX);

    // RGB leds
    led1 = new RGBled(LED_1_RED, LED_1_GREEN, LED_1_BLUE);
    led2 = new RGBled(LED_2_RED, LED_2_GREEN, LED_2_BLUE);

    // motors
    motor_left = new DCmotor(MOTOR_LEFT_A, MOTOR_LEFT_A_CH, MOTOR_LEFT_B, MOTOR_LEFT_B_CH, MOTOR_LEFT_FLIP);
    motor_right = new DCmotor(MOTOR_RIGHT_A, MOTOR_RIGHT_A_CH, MOTOR_RIGHT_B, MOTOR_RIGHT_B_CH, MOTOR_RIGHT_FLIP);

    // encoders
    encoder_left = new Encoder(TIM3, ENC_LEFT_FLIP);
    encoder_right = new Encoder(TIM5, ENC_RIGHT_FLIP);

    // motor control
    motor_control_left = new MotorControl(motor_left, encoder_left,
                                          MOTOR_KP_DEFAULT, MOTOR_KI_DEFAULT, MOTOR_KD_DEFAULT,
                                          MOTOR_CONTROL_PERIOD, CONTROL_MODE_LINEAR, MOTOR_CONTROL_STEP,
                                          POSITION_KP_DEFAULT, POSITION_KI_DEFAULT, POSITION_KD_DEFAULT,
                                          POSITION_CONTROL_PERIOD, POSITION_MAX_SPEED);

    motor_control_right = new MotorControl(motor_right, encoder_right, 
                                          MOTOR_KP_DEFAULT, MOTOR_KI_DEFAULT, MOTOR_KD_DEFAULT,
                                          MOTOR_CONTROL_PERIOD, CONTROL_MODE_LINEAR, MOTOR_CONTROL_STEP,
                                          POSITION_KP_DEFAULT, POSITION_KI_DEFAULT, POSITION_KD_DEFAULT,
                                          POSITION_CONTROL_PERIOD, POSITION_MAX_SPEED);

    // color sensor
    apds9960 = new APDS9960(*wire, APDS_INT);

    // servo
    servo_A = new Servo();
    servo_B = new Servo();

    // bms
    bms = new MAX17332(*wire);

    // touch
    touch_sensor = new AT42QT2120(*wire);

    // imu
    imu = new LSM6DSOSensor(wire, LSM6DSO_I2C_ADD_L);
    ipKnobs = &iKnobs;
    imu_delta_time = MOTION_FX_ENGINE_DELTATIME;
    sample_to_discard = 0;
    is_shaking = 0;
    first_wakeup = true;
    shake_time = 0;
    shake_counter = 0;
    tilt_status = 0x80;
    xl = 0;
    xh = 0;
    yl = 0;
    yh = 0;
    zl = 0;
    zh = 0;
    tilt_time = 0;
    tmp_tilt_status = 0;
    tilt_filter = 0;

    // version
    version_high = VERSION_BYTE_HIGH;
    version_mid = VERSION_BYTE_MID;
    version_low = VERSION_BYTE_LOW;

    // kinematics
    kinematics = new Kinematics(WHEEL_TRACK_MM, WHEEL_DIAMETER_MM);
    kinematics_movement = MOVEMENT_DISABLED;
    kinematics_achieved = false;
    previous_travel = 0.0;
    move_direction = 0.0;
    rotate_pid = new PidController(ROTATE_KP_DEFAULT, ROTATE_KI_DEFAULT, ROTATE_KD_DEFAULT, ROTATE_CONTROL_PERIOD, ROTATE_MAX_SPEED);
    move_pid = new PidController(MOVE_KP_DEFAULT, MOVE_KI_DEFAULT, MOVE_KD_DEFAULT, MOVE_CONTROL_PERIOD, MOVE_MAX_SPEED);
}

int Arduino_AlvikCarrier::begin(){
    beginLeds();
    pinMode(NANO_CHK,INPUT_PULLDOWN);

    serial->begin(UART_BAUD);
    serial->flush();

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
    encoder_left->reset();
    encoder_right->reset();

    motor_control_left->begin();
    motor_control_right->begin();


    wire->begin();
    wire->setClock(400000);
    
    connectExternalI2C();
    ext_wire->begin(ARDUINO_ROBOT_ADDRESS);

    
    if (beginAPDS()!=0){
        errorLed(ERROR_APDS);
    }
     
    beginServo();
    
    if (!beginBMS()){
        errorLed(ERROR_BMS);
    }
    
    if (beginTouch()!=0){
        //errorLed(ERROR_TOUCH);
    }
    
    if (beginImu()!=0){
        errorLed(ERROR_IMU);
    }

    beginBehaviours();
    

    return 0;
}

void Arduino_AlvikCarrier::getVersion(uint8_t &high_byte, uint8_t &mid_byte, uint8_t &low_byte){
    high_byte = version_high;
    mid_byte = version_mid;
    low_byte = version_low;
}


/******************************************************************************************************/
/*                                      Color sensor, APDS9960                                        */
/******************************************************************************************************/

int Arduino_AlvikCarrier::beginAPDS(){
    pinMode(APDS_LED, OUTPUT);
    //enableIlluminator();
    disableIlluminator();
    if (!apds9960->begin()){
        return ERROR_APDS;
    }
    return 0;
}

void Arduino_AlvikCarrier::updateAPDS(){
    if (apds9960->proximityAvailable()){
        bottom_proximity=apds9960->readProximity();
    }
    //digitalWrite(APDS_LED,HIGH);
    if (apds9960->colorAvailable()){
        apds9960->readColor(bottom_red, bottom_green, bottom_blue, bottom_clear);
    }
    //digitalWrite(APDS_LED,LOW);
}

void Arduino_AlvikCarrier::setIlluminator(uint8_t value){
    illuminator_state=value;
    digitalWrite(APDS_LED, value);
}

void Arduino_AlvikCarrier::enableIlluminator(){
    setIlluminator(HIGH);
    prev_illuminator_state = true;
}

void Arduino_AlvikCarrier::disableIlluminator(){
    setIlluminator(LOW);
    prev_illuminator_state = false;
}

int Arduino_AlvikCarrier::getRed(){
    return bottom_red;
}

int Arduino_AlvikCarrier::getGreen(){
    return bottom_green;
}

int Arduino_AlvikCarrier::getBlue(){
    return bottom_blue;
}

int Arduino_AlvikCarrier::getClear(){
    return bottom_clear;
}

int Arduino_AlvikCarrier::getProximity(){
    return bottom_proximity;
}

/******************************************************************************************************/
/*                                        RC Servo A & B                                              */
/******************************************************************************************************/

int Arduino_AlvikCarrier::beginServo(){
    servo_A->attach(SERVO_A);
    servo_B->attach(SERVO_B);
    return 0;
}

void Arduino_AlvikCarrier::setServoA(int position){
    servo_A->write(position);
}

void Arduino_AlvikCarrier::setServoB(int position){
    servo_B->write(position);
}



/******************************************************************************************************/
/*                                        External I2C                                                */
/******************************************************************************************************/

int Arduino_AlvikCarrier::beginI2Cselect(){
    pinMode(SELECT_I2C_BUS, OUTPUT);
}

void Arduino_AlvikCarrier::setExternalI2C(uint8_t state){
    digitalWrite(SELECT_I2C_BUS, state);
}

void Arduino_AlvikCarrier::connectExternalI2C(){
    setExternalI2C(LOW);
}

void Arduino_AlvikCarrier::disconnectExternalI2C(){
    setExternalI2C(HIGH);
}



/******************************************************************************************************/
/*                               Battery Management System, MAX17332                                  */
/******************************************************************************************************/

int Arduino_AlvikCarrier::beginBMS(){
    while(digitalRead(NANO_CHK)==HIGH){}
    return bms->begin();
}

void Arduino_AlvikCarrier::updateBMS(){
    voltage = bms->readVCell();
    state_of_charge = bms->readSoc();
}


float Arduino_AlvikCarrier::getBatteryVoltage(){
    return voltage;
}

float Arduino_AlvikCarrier::getBatteryChargePercentage(){
    return state_of_charge;
}



/******************************************************************************************************/
/*                                            Motor controls                                          */
/******************************************************************************************************/

int Arduino_AlvikCarrier::beginMotors(){
    motor_left->begin();
    motor_right->begin();
    motor_left->stop();
    motor_right->stop();
    
    encoder_left->begin();
    encoder_right->begin();
    encoder_left->reset();
    encoder_right->reset();

    motor_control_left->begin();
    motor_control_right->begin();
    return 0;
}

void Arduino_AlvikCarrier::updateMotors(){
    motor_control_left->update();
    motor_control_right->update();
}

bool Arduino_AlvikCarrier::setRpmLeft(const float rpm){
    return motor_control_left->setRPM(rpm);
}

float Arduino_AlvikCarrier::getRpmLeft(){
    return motor_control_left->getRPM();
}

bool Arduino_AlvikCarrier::setRpmRight(const float rpm){
    return motor_control_right->setRPM(rpm);
}

float Arduino_AlvikCarrier::getRpmRight(){
    return motor_control_right->getRPM();
}

bool Arduino_AlvikCarrier::setRpm(const float left, const float right){
    motor_control_left->setRPM(left);
    motor_control_right->setRPM(right);
    return true;
}

void Arduino_AlvikCarrier::getRpm(float & left, float & right){
    left=motor_control_left->getRPM();
    right=motor_control_right->getRPM();
}

void Arduino_AlvikCarrier::setKPidRight(const float kp, const float ki, const float kd){
    motor_control_right->setKP(kp);
    motor_control_right->setKI(ki);
    motor_control_right->setKD(kd);
}

void Arduino_AlvikCarrier::setKPidLeft(const float kp, const float ki, const float kd){
    motor_control_left->setKP(kp);
    motor_control_left->setKI(ki);
    motor_control_left->setKD(kd);
}

void Arduino_AlvikCarrier::setPositionLeft(const float degrees){
    motor_control_left->setPosition(degrees);
}

float Arduino_AlvikCarrier::getPositionLeft(){
    return motor_control_left->getPosition();
}

bool Arduino_AlvikCarrier::isPositionLeftReached(){
    return motor_control_left->isPositionReached();
}

void Arduino_AlvikCarrier::setPositionRight(const float degrees){
    motor_control_right->setPosition(degrees);
}  

float Arduino_AlvikCarrier::getPositionRight(){
    return motor_control_right->getPosition();
}

bool Arduino_AlvikCarrier::isPositionRightReached(){
    return motor_control_right->isPositionReached();
}

void Arduino_AlvikCarrier::setPosition(const float left_deg, const float right_deg){
    setPositionLeft(left_deg);
    setPositionRight(right_deg);
}

void Arduino_AlvikCarrier::getPosition(float & left_deg, float & right_deg){
    left_deg = getPositionLeft();
    right_deg = getPositionRight();
}

bool Arduino_AlvikCarrier::isPositionReached(){
    return isPositionLeftReached() && isPositionRightReached();
}

void Arduino_AlvikCarrier::resetPositionLeft(const float initial_position){
    motor_control_left->resetPosition(initial_position);
}

void Arduino_AlvikCarrier::resetPositionRight(const float initial_position){
    motor_control_right->resetPosition(initial_position);
}

void Arduino_AlvikCarrier::disablePositionControlLeft(){
    motor_control_left->disablePositionControl();
}

void Arduino_AlvikCarrier::disablePositionControlRight(){
    motor_control_right->disablePositionControl();
}

void Arduino_AlvikCarrier::disablePositionControl(){
    disablePositionControlLeft();
    disablePositionControlRight();
} 



/******************************************************************************************************/
/*                                           Touch pads                                               */
/******************************************************************************************************/

int Arduino_AlvikCarrier::beginTouch(){
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

void Arduino_AlvikCarrier::updateTouch(){
    touch_status = touch_sensor->getStatus();
}

bool Arduino_AlvikCarrier::getAnyTouchPressed(){
    if (touch_sensor->touched(touch_status, TOUCH_PAD_GUARD)){
        return true;
    }
    return false;
}

bool Arduino_AlvikCarrier::getTouchKey(const uint8_t key){
    if (touch_sensor->touched(touch_status, key)&&touch_sensor->touched(touch_status, TOUCH_PAD_GUARD)){
        return true;
    }
    return false;
}

uint8_t Arduino_AlvikCarrier::getTouchKeys(const bool single_touch){
    touch_value=0;
    if (getAnyTouchPressed()){
        touch_value|=1;
        if (!single_touch){
            touch_value|=getTouchOk()<<1;
            touch_value|=getTouchDelete()<<2;
            touch_value|=getTouchCenter()<<3;
            touch_value|=getTouchUp()<<4;
            touch_value|=getTouchLeft()<<5;
            touch_value|=getTouchDown()<<6;
            touch_value|=getTouchRight()<<7;
        }
        else{
            if (getTouchOk()){
                touch_value|=1<<1;
            }else{
                if (getTouchDelete()){
                    touch_value|=1<<2;
                }
                else{
                    if (getTouchCenter()){
                        touch_value|=1<<3;
                    }
                    else{
                        if (getTouchLeft()){
                            touch_value|=1<<5;
                        }
                        else{
                            if (getTouchDown()){
                                touch_value|=1<<6;
                            }
                            else{
                                if (getTouchRight()){
                                    touch_value|=1<<7;
                                }
                                else{
                                    if (getTouchUp()){
                                        touch_value|=1<<4;
                                    }
                                }
                            }

                        }
                    }
                }
            }
        }

    }
    return touch_value;
}

bool Arduino_AlvikCarrier::getTouchUp(){
    return getTouchKey(TOUCH_PAD_UP);
}

bool Arduino_AlvikCarrier::getTouchRight(){
    return getTouchKey(TOUCH_PAD_RIGHT);
}

bool Arduino_AlvikCarrier::getTouchDown(){
    return getTouchKey(TOUCH_PAD_DOWN);
}

bool Arduino_AlvikCarrier::getTouchLeft(){
    return getTouchKey(TOUCH_PAD_LEFT);
}

bool Arduino_AlvikCarrier::getTouchCenter(){
    return getTouchKey(TOUCH_PAD_CENTER);
}

bool Arduino_AlvikCarrier::getTouchOk(){
    return getTouchKey(TOUCH_PAD_OK);
}

bool Arduino_AlvikCarrier::getTouchDelete(){
    return getTouchKey(TOUCH_PAD_DELETE);
}



/******************************************************************************************************/
/*                                               Leds                                                 */
/******************************************************************************************************/

int Arduino_AlvikCarrier::beginLeds(){
    pinMode(LED_BUILTIN, OUTPUT);
    // turn off leds
    led1->clear();
    led2->clear();
    return 0;
}

void Arduino_AlvikCarrier::setLedBuiltin(const uint8_t value){
    digitalWrite(LED_BUILTIN, value);
}

void Arduino_AlvikCarrier::setLedLeft(const uint32_t color){
    led1->set(color); 
}

void Arduino_AlvikCarrier::setLedLeft(const uint32_t red, const uint32_t green, const uint32_t blue){
    led1->set(red, green, blue); 
}

void Arduino_AlvikCarrier::setLedLeftRed(const uint32_t red){
    led1->setRed(red);
}

void Arduino_AlvikCarrier::setLedLeftGreen(const uint32_t green){
    led1->setGreen(green);
}

void Arduino_AlvikCarrier::setLedLeftBlue(const uint32_t blue){
    led1->setBlue(blue);
}

void Arduino_AlvikCarrier::setLedRight(const uint32_t color){
    led2->set(color);
}

void Arduino_AlvikCarrier::setLedRight(const uint32_t red, const uint32_t green, const uint32_t blue){
    led2->set(red, green, blue);
}

void Arduino_AlvikCarrier::setLedRightRed(const uint32_t red){
    led2->setRed(red);
}

void Arduino_AlvikCarrier::setLedRightGreen(const uint32_t green){
    led2->setGreen(green);
}

void Arduino_AlvikCarrier::setLedRightBlue(const uint32_t blue){
    led2->setBlue(blue);
}

void Arduino_AlvikCarrier::setLeds(const uint32_t color){
    setLedLeft(color);
    setLedRight(color);  
}

void Arduino_AlvikCarrier::setLeds(const uint32_t red, const uint32_t green, const uint32_t blue){
    setLedLeft(red, green, blue);
    setLedRight(red, green, blue);    
}

void Arduino_AlvikCarrier::setAllLeds(const uint8_t value){
    setLedBuiltin(value&1);
    //setIlluminator((value>>1)&1);
    if ((value>>1)&1){
        if ((behaviours|=1 == 1)&&isLifted()){
            prev_illuminator_state = true;
        }
        else{
            enableIlluminator();
        }
    }
    else{
        disableIlluminator();
    }
    setLedLeftRed(((value>>2)&1));
    setLedLeftGreen(((value>>3)&1));
    setLedLeftBlue(((value>>4)&1));
    setLedRightRed(((value>>5)&1));
    setLedRightGreen(((value>>6)&1));
    setLedRightBlue(((value>>7)&1));
}



/******************************************************************************************************/
/*                                                IMU                                                 */
/******************************************************************************************************/

int Arduino_AlvikCarrier::beginImu(){
    imu->begin();
    imu->Set_X_ODR(100.0);
    imu->Set_X_FS(4);
    imu->Set_G_ODR(100.0);
    imu->Set_G_FS(2000);
    imu->Enable_X();
    imu->Enable_G();
    imu->Enable_Wake_Up_Detection(LSM6DSO_INT1_PIN);
    imu->Set_Wake_Up_Threshold(1);
    imu->Set_Wake_Up_Duration(3);
    imu->Enable_6D_Orientation(LSM6DSO_INT1_PIN);


    delay(10);

    MotionFX_initialize((MFXState_t *)mfxstate);

    MotionFX_getKnobs(mfxstate, ipKnobs);

    ipKnobs->acc_orientation[0] = 'n';
    ipKnobs->acc_orientation[1] = 'e';
    ipKnobs->acc_orientation[2] = 'd';
    ipKnobs->gyro_orientation[0] = 'n';
    ipKnobs->gyro_orientation[1] = 'e';
    ipKnobs->gyro_orientation[2] = 'd';

    ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC;
    ipKnobs->gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;

    ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
    ipKnobs->LMode = 1;
    ipKnobs->modx = MOTION_FX_DECIMATION;

    MotionFX_setKnobs(mfxstate, ipKnobs);
    MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);
    MotionFX_enable_9X(mfxstate, MFX_ENGINE_DISABLE);

    return 0;
}

void Arduino_AlvikCarrier::updateImu(){
    imu->Get_X_Axes(accelerometer);
    imu->Get_G_Axes(gyroscope);
    imu_data.gyro[0] = (float)gyroscope[0] * FROM_MDPS_TO_DPS;
    imu_data.gyro[1] = (float)gyroscope[1] * FROM_MDPS_TO_DPS;
    imu_data.gyro[2] = (float)gyroscope[2] * FROM_MDPS_TO_DPS;
    imu_data.acc[0] = (float)accelerometer[0] * FROM_MG_TO_G;
    imu_data.acc[1] = (float)accelerometer[1] * FROM_MG_TO_G;
    imu_data.acc[2] = (float)accelerometer[2] * FROM_MG_TO_G;

    if (sample_to_discard>MOTION_FX_SAMPLETODISCARD){
        MotionFX_propagate(mfxstate, &filter_data, &imu_data, &imu_delta_time);
        MotionFX_update(mfxstate, &filter_data, &imu_data, &imu_delta_time, NULL);
    }else{
        sample_to_discard++;
    }

    imu->Get_X_Event_Status(&imu_status);
 
    if (imu_status.WakeUpStatus && 
            (motor_control_left->getRPM()<1 && motor_control_left->getRPM()>-1) &&
            (motor_control_right->getRPM()<1 && motor_control_right->getRPM()>-1)){
        if (first_wakeup){
            shake_time = millis();
            first_wakeup = false;
            shake_counter = 0;
        }
        shake_counter++;
    }

    if (millis()-shake_time>500){
        if (shake_counter>10){
            is_shaking = true;
            shake_counter = 0;
            shake_time_sig = millis();
        }
    }
    if (is_shaking && (millis()-shake_time_sig>1000)){
        is_shaking = false;
        tilt_time = millis();
    }

    if ((!is_shaking) && (millis()-tilt_time>1000)){
        imu->Get_6D_Orientation_XL(&xl);
        imu->Get_6D_Orientation_XH(&xh);
        imu->Get_6D_Orientation_YL(&yl);
        imu->Get_6D_Orientation_YH(&yh);
        imu->Get_6D_Orientation_ZL(&zl);
        imu->Get_6D_Orientation_ZH(&zh);
        
        tmp_tilt_status = 0;
        tmp_tilt_status |= xl<<4;
        tmp_tilt_status |= xh<<5;
        tmp_tilt_status |= zl<<7;
        tmp_tilt_status |= zh<<6;
        tmp_tilt_status |= yh<<3;
        tmp_tilt_status |= yl<<2;

        if (tilt_status !=  tmp_tilt_status){
            tilt_filter++;
        }else{
            tilt_filter = 0;
        }

        if (tilt_filter>20){
            tilt_status = tmp_tilt_status;
            tilt_filter = 0;
        }

    }

}

float Arduino_AlvikCarrier::getAccelerationX(){
    return -imu_data.acc[1];
}

float Arduino_AlvikCarrier::getAccelerationY(){
    return -imu_data.acc[0];
}

float Arduino_AlvikCarrier::getAccelerationZ(){
    return imu_data.acc[2];
}

float Arduino_AlvikCarrier::getAngularVelocityX(){
    return imu_data.gyro[1];
}

float Arduino_AlvikCarrier::getAngularVelocityY(){
    return imu_data.gyro[0];
}

float Arduino_AlvikCarrier::getAngularVelocityZ(){
    return -imu_data.gyro[2];
}

float Arduino_AlvikCarrier::getRoll(){
    return -filter_data.rotation[1];
}

float Arduino_AlvikCarrier::getPitch(){
    return -filter_data.rotation[2];
}

float Arduino_AlvikCarrier::getYaw(){
    return 360.0-filter_data.rotation[0];
}

bool Arduino_AlvikCarrier::isShaking(){
    return is_shaking;
}

uint8_t Arduino_AlvikCarrier::getMotion(){
    return tilt_status | isShaking();
}



/******************************************************************************************************/
/*                                               Error                                                */
/******************************************************************************************************/

void Arduino_AlvikCarrier::errorLed(const int error_code){
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



/******************************************************************************************************/
/*                                             Kinematics                                             */
/******************************************************************************************************/

void Arduino_AlvikCarrier::updateKinematics(){
    kinematics->inverse(motor_control_left->getRPM(), motor_control_right->getRPM());
    kinematics->updatePose();
    if (kinematics_movement!=MOVEMENT_DISABLED){
        if (kinematics_movement==MOVEMENT_ROTATE){
            rotate_pid->update(kinematics->getTheta());
            drive(0, round(rotate_pid->getControlOutput()/10.0)*10);
            if (abs(rotate_pid->getError())<ROTATE_THRESHOLD){
                kinematics_achieved=true;
                disableKinematicsMovement();
                drive(0,0);
            }
        }
        if (kinematics_movement==MOVEMENT_MOVE){
            move_pid->update((kinematics->getTravel()-previous_travel)*move_direction);
            drive(round(move_pid->getControlOutput()/10.0)*10, 0);
            if (abs(move_pid->getError())<MOVE_THRESHOLD){
                kinematics_achieved=true;
                disableKinematicsMovement();
                drive(0,0);
            }
            
        }
    }
}

void Arduino_AlvikCarrier::drive(const float linear, const float angular){
    kinematics->forward(linear, angular);
    setRpm(kinematics->getLeftVelocity(), kinematics->getRightVelocity());
}

float Arduino_AlvikCarrier::getLinearVelocity(){
    return kinematics->getLinearVelocity();
}

float Arduino_AlvikCarrier::getAngularVelocity(){
    return kinematics->getAngularVelocity();
}

float Arduino_AlvikCarrier::getX(){
    return kinematics->getX();
}

float Arduino_AlvikCarrier::getY(){
    return kinematics->getY();
}

float Arduino_AlvikCarrier::getTheta(){
    return kinematics->getTheta();
}

void Arduino_AlvikCarrier::resetPose(const float x0, const float y0, const float theta0){
    kinematics->resetPose(x0,y0,theta0);
}

void Arduino_AlvikCarrier::lockingRotate(const float angle){
    float initial_angle = kinematics->getTheta();
    float final_angle = angle+initial_angle;
    float error = angle;
    unsigned long t = millis();
    while(abs(error)>2){
        if (millis()-t>20){
            t = millis();
            updateMotors();
            kinematics->inverse(motor_control_left->getRPM(),motor_control_right->getRPM());
            kinematics->updatePose();
            error = final_angle-kinematics->getTheta();
        }
        if (angle>0){
            drive(0, 40);
        }else{
            drive(0, -40);
        }
    }
    drive(0,0);
    updateMotors();
    motor_control_left->brake();
    motor_control_right->brake();
}

void Arduino_AlvikCarrier::rotate(const float angle){
    disableKinematicsMovement();
    kinematics_achieved=false;
    rotate_pid->reset();
    rotate_pid->setReference(kinematics->getTheta()+angle);
    kinematics_movement=MOVEMENT_ROTATE;
}

void Arduino_AlvikCarrier::lockingMove(const float distance){
    float initial_travel = kinematics->getTravel();
    float final_travel = abs(distance)+initial_travel;
    float error = abs(distance);
    unsigned long t = millis();

    while(error>2){
        if (millis()-t>20){
            t = millis();
            updateMotors();
            kinematics->inverse(motor_control_left->getRPM(), motor_control_right->getRPM());
            kinematics->updatePose();
            error = final_travel-kinematics->getTravel();
        }
        if (distance>0){
            drive(40, 0);
        }else{
            drive(-40, 0);
        }
    }
    drive(0, 0);
    updateMotors();
    motor_control_left->brake();
    motor_control_right->brake();
}

void Arduino_AlvikCarrier::move(const float distance){
    disableKinematicsMovement();
    kinematics_achieved=false;

    move_pid->reset();
    previous_travel=kinematics->getTravel();
    if (distance<0){
        move_direction=-1.0;
    }
    else{
        move_direction=1.0;
    }
    move_pid->setReference(distance);
    kinematics_movement=MOVEMENT_MOVE;
}

void Arduino_AlvikCarrier::disableKinematicsMovement(){
    kinematics_movement=MOVEMENT_DISABLED;
}

bool Arduino_AlvikCarrier::isTargetReached(){
    return kinematics_achieved;
}

uint8_t Arduino_AlvikCarrier::getKinematicsMovement(){
    return kinematics_movement;
}



/******************************************************************************************************/
/*                                             Behaviours                                             */
/******************************************************************************************************/
void Arduino_AlvikCarrier::beginBehaviours(){
    prev_illuminator_state = illuminator_state;
    behaviours = 0;
    first_lift = true;
}


void Arduino_AlvikCarrier::updateBehaviours(){
    if (behaviours|=1 == 1){
        /*
        if (isLifted()&&first_lift){
            first_lift = false;
            prev_illuminator_state = illuminator_state;
            disableIlluminator();
        }
        if (isLifted()&&!first_lift) {
            if (prev_illuminator_state!=0){
                disableIlluminator();
            }
        }
        if (!isLifted()&&!first_lift){
            if (prev_illuminator_state!=0){
                //first_lift = true;
                enableIlluminator();
            }
        }
        */
       if (isLifted()&&first_lift){
        //disableIlluminator();
        setIlluminator(LOW);
        first_lift=false;
       }
       else{
        if (!isLifted()){
            setIlluminator(prev_illuminator_state);
        }
        if (!isLifted()&&!first_lift){
            first_lift = true;
        }
       }
    }
}

void Arduino_AlvikCarrier::setBehaviour(const uint8_t behaviour, const bool enable){
    if (enable){
        behaviours |= behaviour;
    }
    else{
        behaviours &= ~behaviour;
    }
}

bool Arduino_AlvikCarrier::isLifted(){
    if (getProximity()>LIFT_THRESHOLD){
        return true;
    }
    else{
        return false;
    }
}



/******************************************************************************************************/
/*                                             Utilities                                              */
/******************************************************************************************************/

void Arduino_AlvikCarrier::getSerialNumber(char * sn){
  uint32_t id[3];
  id[0] = HAL_GetUIDw0();
  id[1] = HAL_GetUIDw1();
  id[2] = HAL_GetUIDw2();
  uint32_t top = id[0]+id[2];
  uint16_t bottom = (id[1]&0xFFFF0000)>>16;
  sprintf(sn,"%08lx", top);
  sprintf(sn+8,"%x",bottom);
}