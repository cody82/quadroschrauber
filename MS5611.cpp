#include "MS5611.h"

//#define MS5611_ADRESS (0b1110111) => 0xEE
#define MS5611_ADRESS (0x77)

#define I2CADR_W(ADR)           (ADR<<1&0xFE)
#define I2CADR_R(ADR)           (ADR<<1|0x01)

#define MS5611_RESET (0x1E)
#define MS6511_CONVERT_D1_256 (0x40)

#define MS5611_PROM_READ_COMMAND(INDEX) (0xA0 + 2 * i)


#define MS5611_PRESSURE    0x40
#define MS5611_TEMPERATURE 0x50


#define MS5611_OSR_256  0x00
#define MS5611_OSR_512  0x02
#define MS5611_OSR_1024 0x04
#define MS5611_OSR_2048 0x06
#define MS5611_OSR_4096 0x08

MS5611::MS5611(I2C &port)
    :i2c(port), command_timer(0), current_command(0), temperature_raw(0), pressure_raw(0), pressure(0), temperature(0), altitude(0)
{
    memset(calibration_data, 0, 16);
}


uint16_t MS5611::GetCoefficient(int index)
{
    return (uint16_t)calibration_data[index * 2] * 256 + (uint16_t)calibration_data[index * 2 + 1];
}

float MS5611::GetPressure()
{
    return pressure;
}

float MS5611::GetTemperature()
{
    return temperature;
}

float MS5611::GetAltitude()
{
    return altitude;
}

void MS5611::Compensate(uint32_t up, uint32_t ut)
{
  int32_t temperature,off2=0,sens2=0,delt;

  int32_t dT   = ut - ((uint32_t)GetCoefficient(5) << 8);
  int64_t off  = ((uint32_t)GetCoefficient(2) <<16) + (((int64_t)dT * GetCoefficient(4)) >> 7);
  int64_t sens = ((uint32_t)GetCoefficient(1) <<15) + (((int64_t)dT * GetCoefficient(3)) >> 8);
  temperature  = 2000 + (((int64_t)dT * GetCoefficient(6))>>23);

  if (temperature < 2000) { // temperature lower than 20st.C 
    delt = temperature-2000;
    delt  = delt*delt;
    off2  = (5 * delt)>>1; 
    sens2 = (5 * delt)>>2; 
    if (temperature < -1500) { // temperature lower than -15st.C
      delt  = temperature+1500;
      delt  = delt*delt;
      off2  += 7 * delt; 
      sens2 += (11 * delt)>>1; 
    }
  } 
  off  -= off2; 
  sens -= sens2;
  pressure     = (( (up * sens ) >> 21) - off) >> 15;
  pressure *= 0.01f;
  this->temperature = temperature * 0.01f;
  
  altitude = (1.0f - pow(pressure/1013.250f, 0.190295f)) * 44330.0f; //meter
}

bool MS5611::Command(char cmd)
{
    command_timer = 0.010f;
    current_command = cmd;
    cmd += MS5611_OSR_4096;
    if(i2c.write(I2CADR_W(MS5611_ADRESS), &cmd, 1) != 0)
        return false;
    return true;
}

bool MS5611::Update(float dtime)
{
    command_timer -= dtime;
    
    if(command_timer <= 0)
    {
        if(current_command != 0)
        {
            char cmd = 0;
            char convert[3];
            
            if(i2c.write(I2CADR_W(MS5611_ADRESS), &cmd, 1) != 0)
                return false;
            if(i2c.read(I2CADR_R(MS5611_ADRESS), convert, 3) != 0)
                return false;
                
            if(current_command == MS5611_PRESSURE)
            {
                pressure_raw = (((uint32_t)convert[0]) << 16) + (((uint32_t)convert[1]) << 8) + ((uint32_t)convert[2]);
                Command(MS5611_TEMPERATURE);
            }
            else if(current_command == MS5611_TEMPERATURE)
            {
                temperature_raw = (((uint32_t)convert[0]) << 16) + (((uint32_t)convert[1]) << 8) + ((uint32_t)convert[2]);
                Command(MS5611_PRESSURE);
            }
            
            if(temperature_raw != 0 && pressure_raw != 0)
                Compensate(pressure_raw, temperature_raw);
        }
        else
            Command(MS5611_PRESSURE);
    }
    
    return true;
}

bool MS5611::Init()
{
    char cmd = MS5611_RESET;
    if(i2c.write(I2CADR_W(MS5611_ADRESS), &cmd, 1) != 0)
        return false;

    wait_ms(5);
    
    for(int i = 1; i < 8; ++i)
    {
        cmd = MS5611_PROM_READ_COMMAND(i);
        if(i2c.write(I2CADR_W(MS5611_ADRESS), &cmd, 1) != 0)
            return false;
    
        if(i2c.read(I2CADR_R(MS5611_ADRESS), &calibration_data[i*2], 2) != 0)
            return false;
    }
    
    Update(0.01);
    wait_ms(10);
    Update(0.01);
    wait_ms(10);
    Update(0.01);
    
    return true;
}

