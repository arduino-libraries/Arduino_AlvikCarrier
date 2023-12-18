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
      /*
      Serial.println(" ");
      Serial.print("TOP=");
      Serial.println(tof.getTop());
      Serial.print("BOT=");
      Serial.println(tof.getBottom());
      Serial.print("LEFT=");
      Serial.println(tof.getLeft());
      Serial.print("RIGHT=");
      Serial.println(tof.getRight());
      Serial.print("CLEFT=");
      Serial.println(tof.getCenterLeft());
      Serial.print("CRIGHT=");
      Serial.println(tof.getCenterRight());
      Serial.print("CENTER=");
      Serial.println(tof.getCenter());
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
      */
      Serial.print("\t");
      Serial.print(tof.getLeft());
      Serial.print("\t");
      Serial.print(tof.getLeftCenter());
      Serial.print("\t");
      Serial.print(tof.getCenter());
      Serial.print("\t");
      Serial.print(tof.getRightCenter());
      Serial.print("\t");
      Serial.print(tof.getRight());
      Serial.print("\t");
      Serial.print(tof.getTop());
      Serial.print("\t");
      Serial.print(tof.getBottom());
      Serial.print("\n");
    }
    

    delay(1000);
}