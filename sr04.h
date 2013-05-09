#pragma once

#include "mbed.h"

class SR04
{
public:
    SR04(PinName ECHO, PinName TRIG);
    
    void Trigger();
    float Distance() const;
    bool Valid() const;
    
protected:
    void EchoRaise();
    void EchoFall();
    
    InterruptIn echo;
    DigitalOut trig;
    Timer timer;
    volatile float distance;
};
