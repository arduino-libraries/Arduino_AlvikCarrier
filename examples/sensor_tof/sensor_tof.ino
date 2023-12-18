#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>
#include "sensor_tof_matrix.h"
#include "platform_config_custom.h"

TwoWire wire(PB7, PB8);

#define LPN_PIN PB1
#define I2C_RST_PIN PB0

SensorTofMatrix tof(&wire, LPN_PIN, I2C_RST_PIN);

void setup() {
    Serial.begin(9600);
    tof.begin();
}

void loop() {
    bool updated = false;
    updated = tof.update_rois();
    
    if (updated) {
      Serial.println(" ");
      Serial.print("TOP=");
      Serial.println(tof.top);
      Serial.print("BOT=");
      Serial.println(tof.bottom);
      Serial.print("LEFT=");
      Serial.println(tof.left);
      Serial.print("RIGHT=");
      Serial.println(tof.right);
      Serial.print("CLEFT=");
      Serial.println(tof.center_left);
      Serial.print("CRIGHT=");
      Serial.println(tof.center_right);
      Serial.print("CENTER=");
      Serial.println(tof.center);
      Serial.print("LEFT_MIN=");
      Serial.println(tof.get_min_range_left_mm());
      Serial.print("RIGHT_MIN=");
      Serial.println(tof.get_min_range_right_mm());
      Serial.print("CLEFT_MIN=");
      Serial.println(tof.get_min_range_center_left_mm());
      Serial.print("CRIGHT_MIN=");
      Serial.println(tof.get_min_range_center_right_mm());
      Serial.print("CENTER_MIN=");
      Serial.println(tof.get_min_range_center_mm());
    }

    delay(1000);
}