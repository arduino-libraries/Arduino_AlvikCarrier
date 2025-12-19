

#include "Arduino_AlvikCarrier.h"

Arduino_AlvikCarrier alvik;

uint8_t version[3];
unsigned long led_update = 0;
bool led_state = false;
volatile uint8_t lastCommand = 0xfe;
uint32_t lastcolor = COLOR_BLACK;

// Called when master sends data
void receiveEvent(int bytes) {
    while (alvik.ext_wire->available()) {
        lastCommand = alvik.ext_wire->read();
    }
    alvik.setLedLeft(++lastcolor);
    lastcolor = lastcolor%(COLOR_WHITE+1);
}

// Called when master requests data
void requestEvent() {
    byte response = lastCommand + 1;   // Simple response logic
    alvik.ext_wire->write(response);
}

void setup() {
    alvik.begin();

    alvik.ext_wire->onReceive(receiveEvent);
    alvik.ext_wire->onRequest(requestEvent);

    alvik.disableIlluminator();
    alvik.setLeds(COLOR_ORANGE);
    alvik.setLedBuiltin(HIGH);

    alvik.getVersion(version[0], version[1], version[2]);

    alvik.updateBMS();

    alvik.setLedBuiltin(LOW);
    alvik.setLeds(COLOR_BLACK);

}

void loop() {

    if (millis()-led_update>=200) {
        led_update=millis();
        if (led_state) {
            alvik.setLedRight(COLOR_RED);
        } else {
            alvik.setLedRight(COLOR_BLACK);
        }
        led_state=!led_state;
    }

}