#include "motor.h"


#define CLAMP(X,MIN,MAX) if(X<(MIN))X=(MIN);else if(X>(MAX))X = (MAX);

Motor::Motor(PwmOut &_pwm)
    : pwm(_pwm), limit(1.0f)
{
}

void Motor::SetThrottleLimit(float _limit)
{
    limit = _limit;
}
    
void Motor::SetThrottle(float throttle)
{
    CLAMP(throttle, 0.0f, limit);
    
    uint16_t pulse_us = 1000 + (int)(throttle * 1000.0f);
    pwm.pulsewidth_us(pulse_us);
}


void Motor::SetThrottleMillis(int millis)
{
    CLAMP(millis, 0, (int)(limit * 1000.0f));
    
    uint16_t pulse_us = 1000 + millis;
    pwm.pulsewidth_us(pulse_us);
}

