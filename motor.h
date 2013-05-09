#pragma once

#include "mbed.h"

class Motor
{
public:
    Motor(PwmOut &_pwm);
    
    void SetThrottle(float throttle);
    void SetThrottleMillis(int millis);
    
    void SetThrottleLimit(float _limit);
protected:
    PwmOut &pwm;
    float limit;
};

