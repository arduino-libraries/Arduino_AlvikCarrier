#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>
#include "sensor_tof_matrix.h"
#include "platform_config_custom.h"

TwoWire wire(PB7, PB8);
#define DEV_I2C wire

//#define SerialPort Serial

#define LPN_PIN PB1
#define I2C_RST_PIN PB0

// VL53L7CX sensor_vl53l7cx(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

SensorTofMatrix tof(&wire, LPN_PIN, I2C_RST_PIN);

void setup() {
    tof.begin();
}

void loop() {
    tof.get_range_mm_top();
    delay(100);
}