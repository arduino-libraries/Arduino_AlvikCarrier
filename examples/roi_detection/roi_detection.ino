#include <Arduino.h>
#include <Wire.h>
#include "platform_config_custom.h"
#include <vl53l7cx_class.h>
#include "sensor_tof_matrix.h"

TwoWire wire(PB7, PB8);
#define DEV_I2C wire

#define LPN_PIN PB1
#define I2C_RST_PIN PB0

SensorTofMatrix tof(&wire, LPN_PIN, I2C_RST_PIN);

void setup() {
    tof.begin();
}

void loop() {
    Serial.print("TOP MIN: ");
    Serial.println(tof.get_min_range_top_mm());
    delay(10);
}