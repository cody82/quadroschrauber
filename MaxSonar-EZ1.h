#pragma once

#include "mbed.h"

class MaxSonarPWM
{
private:
    InterruptIn pwm;
    Timer timer;
    volatile int distance;
    
    void EchoRaise();
    void EchoFall();
    
public:
    //MaxSonarPWM(InterruptIn &_pwm);
    MaxSonarPWM(PinName _pwm);
    MaxSonarPWM(PinName ECHO, PinName TRIG);
    
    int DistanceCM() const;
    float Distance() const;
    bool Valid() const;
    void Trigger();
    
};
