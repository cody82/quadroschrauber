#include "sr04.h"


SR04::SR04(PinName ECHO,PinName TRIG)
        : echo(ECHO), trig(TRIG)
{
    echo.rise(this, &SR04::EchoRaise);
    echo.fall(this, &SR04::EchoFall);
    echo.mode(PullNone);
    trig = 0;
    distance = -1;
}
    

void SR04::Trigger()
{
    trig = 1;
    wait_us(15);
    trig = 0;
}
    
void SR04::EchoRaise()
{
    timer.stop();
    timer.reset();
    timer.start();
}
 
void SR04::EchoFall()
{
    int time = timer.read_us();
    distance = (float)time / 58.0f / 100.0f;
}

float SR04::Distance() const
{
    return Valid() ? distance : -1.0f;
}

bool SR04::Valid() const
{
    return distance > 0 && distance < 4;
}
