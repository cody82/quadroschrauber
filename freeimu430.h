#pragma once

#include "mbed.h"
#include "mpu-60x0.h"
#include "GTMath.h"
#include "HMC5883.h"
#include "MS5611.h"

enum MPU_DLPF
{
    MPU_DLPF_98 = 2,
    MPU_DLPF_42 = 3,
    MPU_DLPF_20 = 4
};

class FreeIMU430
{
public:
    FreeIMU430(I2C &port, char _adress, Timer &_timer);
    
    bool Init();
    bool Update(float dtime);
    void Calibrate();
    
    Vector3 accel;
    Vector3 gyro;
    Vector3 magneto;
    
    Vector3 gyro_calibration;
    Vector3 accel_calibration;
    
    void SetDivider(uint8_t div);
    void SetLowPass(MPU_DLPF lpf);
    
    MS5611 ms5611;
protected:
    char adress;
    struct mpu_60x0_data data;
    I2C &i2c;
    Timer &timer;
    
    HMC5883 hmc5883;
};

