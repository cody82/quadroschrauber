#include "blinkm.h"

BlinkM::BlinkM(I2C &_i2c, char _adress) : i2c(_i2c), adress(_adress)
{
}

bool BlinkM::StopScript()
{
    return i2c.write(adress, "o", 1) == 0;
}

bool BlinkM::PlayScript(char number, char repeats)
{
    char data[4] = {'p', number, repeats, 0};
    return i2c.write(adress, data, 4) == 0;
}

bool BlinkM::Fade(char r, char g, char b)
{
    char data[4] = {'c', r, g, b};
    return i2c.write(adress, data, 4) == 0;
}

bool BlinkM::Set(char r, char g, char b)
{
    char data[4] = {'n', r, g, b};
    return i2c.write(adress, data, 4) == 0;
}
