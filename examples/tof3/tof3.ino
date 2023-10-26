#include <Wire.h>
#include <VL53L1X.h>
#include <Wire.h>

TwoWire wire(PB7, PB8);
const uint8_t sensorCount = 3;

const uint8_t xshutPins[sensorCount] = { PC5, PB0, PB1 }; //Right, Center, Left

VL53L1X sensors[sensorCount];

void setup(){
  while (!Serial);
  Serial.begin(115200);
  wire.begin();
  wire.setClock(400000);

  for (uint8_t i = 0; i < sensorCount; i++){
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  for (uint8_t i = 0; i < sensorCount; i++){
    pinMode(xshutPins[i], INPUT);
    delay(10);
    sensors[i].setBus(&wire);
    sensors[i].setTimeout(500);
    if (!sensors[i].init()){
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    sensors[i].setAddress(0x2A + i);
    sensors[i].startContinuous(50);
  }
}

void loop(){
  for (uint8_t i = 0; i < sensorCount; i++){
    Serial.print(sensors[i].read());
    if (sensors[i].timeoutOccurred()){
        Serial.print(" TIMEOUT");
    }
    Serial.print('\t');
  }
  Serial.print("\n");
}
