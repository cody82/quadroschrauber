#include "MaxSonar-EZ1.h"


MaxSonarPWM::MaxSonarPWM(PinName _pwm)
    : pwm(_pwm)
{
    pwm.rise(this, &MaxSonarPWM::EchoRaise);
    pwm.fall(this, &MaxSonarPWM::EchoFall);
    pwm.mode(PullNone);
    distance = -1;
}

MaxSonarPWM::MaxSonarPWM(PinName ECHO, PinName TRIG)
    : pwm(ECHO)
{
    pwm.rise(this, &MaxSonarPWM::EchoRaise);
    pwm.fall(this, &MaxSonarPWM::EchoFall);
    pwm.mode(PullNone);
    distance = -1;
}
    
void MaxSonarPWM::EchoRaise()
{
    timer.stop();
    timer.reset();
    timer.start();
}
 
void MaxSonarPWM::EchoFall()
{
    int time = timer.read_us();
    distance = time / 58;
}


int MaxSonarPWM::DistanceCM() const
{
    return distance;
}

float MaxSonarPWM::Distance() const
{
    return Valid() ? ((float)distance * 0.01f) : -1.0f;
}

bool MaxSonarPWM::Valid() const
{
    return distance > 0 && distance < 400;
}

void MaxSonarPWM::Trigger()
{
}
