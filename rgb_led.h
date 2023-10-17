#ifndef __RGB_LED_H__
#define __RGB_LED_H__

#include "Arduino.h"

class RGBled{
    private:
        uint8_t R;
        uint8_t G;
        uint8_t B;
    public:
        RGBled(const uint8_t _Red, const uint8_t _Green, const uint8_t _Blue){
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

        void setRed(const uint32_t red){
            digitalWrite(R, red);
        }

        void setGreen(const uint32_t green){
            digitalWrite(G, green);
        }

        void setBlue(const uint32_t blue){
            digitalWrite(B, blue);
        }

        void set(const uint32_t red, const uint32_t green, const uint32_t blue){
            setRed(red);
            setGreen(green);
            setBlue(blue);
        }

        void clear(){
            setRed(0);
            setGreen(0);
            setBlue(0);
        }
};



#endif