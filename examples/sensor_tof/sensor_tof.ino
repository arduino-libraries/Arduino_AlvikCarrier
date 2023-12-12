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
    updated = tof.update();
    
    if (updated) {
      Serial.println(" ");
      Serial.print("TOP=");
      Serial.println(tof.get_min_range_top_mm());
      Serial.print("BOT=");
      Serial.println(tof.get_max_range_bottom_mm());
      Serial.print("LEFT=");
      Serial.println(tof.get_min_range_left_mm());
      Serial.print("RIGHT=");
      Serial.println(tof.get_min_range_right_mm());
      Serial.print("CLEFT=");
      Serial.println(tof.get_min_range_center_left_mm());
      Serial.print("CRIGHT=");
      Serial.println(tof.get_min_range_center_right_mm());
      Serial.print("CENTER=");
      Serial.println(tof.get_min_range_center_mm());
    }

    //delay(1000);
}