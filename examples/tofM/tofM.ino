#include <Arduino.h>
#include <Wire.h>
#include <vl53l7cx_class.h>


TwoWire wire(PB7, PB8);
#define DEV_I2C wire

#define SerialPort Serial

#define LPN_PIN PB1
#define I2C_RST_PIN PB0

VL53L7CX sensor_vl53l7cx(&DEV_I2C, LPN_PIN, I2C_RST_PIN);


uint8_t res = VL53L7CX_RESOLUTION_8X8; //VL53L7CX_RESOLUTION_4X4


void setup(){
  SerialPort.begin(115200);
  DEV_I2C.begin();
  sensor_vl53l7cx.begin();
  sensor_vl53l7cx.init_sensor();
  sensor_vl53l7cx.vl53l7cx_set_resolution(res);
  sensor_vl53l7cx.vl53l7cx_start_ranging();
}

void loop(){
  VL53L7CX_ResultsData Results;
  uint8_t NewDataReady = 0;
  uint8_t status;
  do {
    status = sensor_vl53l7cx.vl53l7cx_check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    status = sensor_vl53l7cx.vl53l7cx_get_ranging_data(&Results);
    for (int y=0; y<8; y++){
      for (int x=0; x<8; x++){
        Serial.print((int)Results.distance_mm[x+y*8]);
        Serial.print(" ");
      }
      Serial.println();
    }
    Serial.println();
  }
  delay(100);

}