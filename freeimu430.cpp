#include "freeimu430.h"



FreeIMU430::FreeIMU430(I2C &port, char _adress, Timer &_timer)
: i2c(port), adress(_adress), timer(_timer), hmc5883(port, _timer), ms5611(port)
{
}


#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1

#define MPU6050_USERCTRL_I2C_MST_EN_BIT         5


void FreeIMU430::SetDivider(uint8_t div)
{
    mpu_write(i2c, adress, R_SMPLRT_DIV, (char*)&div, 1);
}
    
void FreeIMU430::SetLowPass(MPU_DLPF lpf)
{
    mpu_write(i2c, adress, R_CONFIG, (char*)&lpf, 1);
}

bool FreeIMU430::Init()
{
    if (!mpu_probe(i2c, adress))
    {
        return false;
    }
    
    if (!mpu_init(i2c, adress))
    {
        return false;
    }
    
    char tmp;
    
    mpu_read(i2c, adress, R_USER_CTRL, &tmp, 1);
    tmp &=~(1<<MPU6050_USERCTRL_I2C_MST_EN_BIT);
    mpu_write(i2c, adress, R_USER_CTRL, &tmp, 1);
    
    mpu_read(i2c, adress, R_INT_PIN_CFG, &tmp, 1);
    tmp |=1<<MPU6050_INTCFG_I2C_BYPASS_EN_BIT;
    mpu_write(i2c, adress, R_INT_PIN_CFG, &tmp, 1);
    
    //MPU_DLPF_42
    //SetLowPass(MPU_DLPF_98);
    //SetLowPass(MPU_DLPF_42);
    SetLowPass(MPU_DLPF_20);
    SetDivider(9);
    //SetDivider(0);
    
    wait(0.1);
    
    if(!hmc5883.Init())
        return false;
        
    return ms5611.Init();
}

//#define TO_G(x) (((float)(x))/16384.0f)
#define TO_G(x) (((float)(x))/8192.0f) //4g
//#define TO_DEG_PER_SEC(x) (((float)(x))/131.0f)
//#define TO_DEG_PER_SEC(x) (((float)(x))/65.5f) //500deg/s
#define TO_DEG_PER_SEC(x) (((float)(x))/32.8f) //1000deg/s
#define TO_RAD_PER_SEC(x) (TO_DEG_PER_SEC(x) / 180.0f * 3.141f)

bool FreeIMU430::Update(float dtime)
{
    hmc5883.Update();

    if (!mpu_int_data(i2c, adress, &data, 1))
    {
        return false;
    }
    
    ms5611.Update(dtime);
    
    
    accel.x = TO_G(data.accel[1]) + accel_calibration.x;
    accel.y = TO_G(data.accel[0]) + accel_calibration.y;
    accel.z = TO_G(data.accel[2]) + accel_calibration.z;
    gyro.x = -TO_RAD_PER_SEC(data.gyro[1]) + gyro_calibration.x;
    gyro.y = -TO_RAD_PER_SEC(data.gyro[0]) + gyro_calibration.y;
    gyro.z = -TO_RAD_PER_SEC(data.gyro[2]) + gyro_calibration.z;
    magneto.x = ((float)hmc5883.RawMag[1]) / 2048.0f;
    magneto.y = ((float)hmc5883.RawMag[0]) / 2048.0f;
    magneto.z = ((float)hmc5883.RawMag[2]) / 2048.0f;
     
    return true;
}

void FreeIMU430::Calibrate()
{
    accel_calibration = Vector3(-accel.x + accel_calibration.x, -accel.y + accel_calibration.y, -accel.z + accel_calibration.y + 1.0f);
    gyro_calibration = Vector3(-gyro.x + gyro_calibration.x, -gyro.y + gyro_calibration.y, -gyro.z + gyro_calibration.z);
}
