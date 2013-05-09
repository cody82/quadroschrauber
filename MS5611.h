#pragma once

#include "mbed.h"

class MS5611
{
public:
    MS5611(I2C &port);
    bool Init();
    
    uint16_t GetCoefficient(int index);
    float GetPressure();
    float GetTemperature();
    float GetAltitude();
    bool Update(float dtime);
    
    uint32_t pressure_raw;
    uint32_t temperature_raw;
    
protected:
    I2C &i2c;
    // 128bit PROM
    char calibration_data[16];
    void Compensate(uint32_t up, uint32_t ut);
    bool Command(char cmd);
    
    float pressure;
    float temperature;
    float altitude;
    
    float command_timer;
    char current_command;
};
