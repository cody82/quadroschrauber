#include "TinyGPS.h"

TinyGPS::TinyGPS(I2C &_i2c)
    : i2c(_i2c), ok(true)
{
    ok = Update();
}

float TinyGPS::GetLatitude()
{
    return nav_data.gps.lat.deg;
}

float TinyGPS::GetLongitude()
{
    return nav_data.gps.lon.deg;
}

float TinyGPS::GetAltitude()
{
    return nav_data.gps.alt.m;
}

bool TinyGPS::Update()
{
    char cmd;
    
    ok = true;
    
    for(cmd = 0; cmd < 20; ++cmd)
    {
        if(i2c.write((TINY_GPS_ADRESS<<1), &cmd, 1) != 0)
            ok = false;
        if(i2c.read((TINY_GPS_ADRESS<<1), &((char*)&nav_data)[cmd], 1) != 0)
            ok = false;
    }
    
    return ok;
}

