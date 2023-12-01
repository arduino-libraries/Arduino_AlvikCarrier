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

unsigned long T1 = 0, T2 = 0;

SensorTofMatrix tof(&wire, LPN_PIN, I2C_RST_PIN);

void setup() {
    tof.begin();
}

void loop() {
    bool updated = false;
    T1 = micros();
    updated = tof.update();
    T2 = micros();

    Serial.print(T2-T1);
    Serial.print(", ");

    if (updated) {
      Serial.println(" ");
      Serial.print("TOP=");
      Serial.println(tof.get_min_range_top_mm());
    }

    delay(100);
}