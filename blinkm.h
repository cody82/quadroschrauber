#include "mbed.h"

class BlinkM
{
public:
    BlinkM(I2C &_i2c, char _adress);
    bool StopScript();
    bool PlayScript(char number, char repeats);
    bool Fade(char r, char g, char b);
    bool Set(char r, char g, char b);
protected:
    I2C &i2c;
    char adress;
};
