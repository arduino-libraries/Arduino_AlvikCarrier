#include "rgb_led.h"

RGBled::RGBled(const uint8_t _Red, const uint8_t _Green, const uint8_t _Blue){
    R=_Red;
    G=_Green;
    B=_Blue;
    pinMode(R,OUTPUT);
    pinMode(G,OUTPUT);
    pinMode(B,OUTPUT);
    digitalWrite(R,LOW);
    digitalWrite(G,LOW);
    digitalWrite(B,LOW);
}

void RGBled::setRed(const uint32_t red){
    digitalWrite(R, red);
}

void RGBled::setGreen(const uint32_t green){
    digitalWrite(G, green);
}

void RGBled::setBlue(const uint32_t blue){
    digitalWrite(B, blue);
}

void RGBled::set(const uint32_t red, const uint32_t green, const uint32_t blue){
    setRed(red);
    setGreen(green);
    setBlue(blue);
}

void RGBled::set(const uint32_t color){
    switch(color){
        case COLOR_RED:
            set(HIGH,LOW,LOW);
            break;
        case COLOR_GREEN:
            set(LOW,HIGH,LOW);
            break;
        case COLOR_BLUE:
            set(LOW,LOW,HIGH);
            break;
        case COLOR_ORANGE:
            set(HIGH,HIGH,LOW);
            break;
        case COLOR_VIOLET:
            set(HIGH,LOW,HIGH);
            break;
        case COLOR_TEAL:
            set(LOW,HIGH,HIGH);
            break;
        case COLOR_WHITE:
            set(HIGH,HIGH,HIGH);
            break;
        case COLOR_BLACK:
        default:
            set(LOW,LOW,LOW);
    }
}

void RGBled::clear(){
    setRed(0);
    setGreen(0);
    setBlue(0);
}