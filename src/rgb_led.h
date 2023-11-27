#ifndef __RGB_LED_H__
#define __RGB_LED_H__

#include "Arduino.h"

#define COLOR_BLACK 0
#define COLOR_RED 1
#define COLOR_GREEN 2
#define COLOR_ORANGE 3
#define COLOR_BLUE 4
#define COLOR_VIOLET 5
#define COLOR_TEAL 6
#define COLOR_WHITE 7

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

        void set(const uint32_t color){
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

        void clear(){
            setRed(0);
            setGreen(0);
            setBlue(0);
        }
};



#endif