#include <Wire.h>

TwoWire wire(PB7, PB8);

 
void setup() {
  wire.begin();
  Serial.begin(115200);
  while(!Serial);
  Serial.println("\nI2C Scanner");
  /*
  pinMode(PB1,OUTPUT);
  pinMode(PC5,OUTPUT);
  pinMode(PB0,OUTPUT);
  digitalWrite(PB1,HIGH);
  digitalWrite(PC5, HIGH);
  digitalWrite(PB0, HIGH);
  */

}
 
void loop() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    wire.beginTransmission(address);
    error = wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          
}
