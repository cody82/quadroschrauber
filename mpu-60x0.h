/* ============================================
This MPU-60X0 test program code and driver is placed under the MIT license
Copyright (c) 2012 Anthony Loeppert anthony@loeppert.net

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#pragma once

enum AXES
{
    AXIS_X, AXIS_Y, AXIS_Z, AXIS_NUM
};

struct __attribute__((__packed__)) mpu_60x0_data
{
    short   accel[AXIS_NUM];
    short   temp;
    short   gyro[AXIS_NUM];
};

int mpu_write(I2C &port, char i2c_addr, char reg_addr, char* data, char size);
int mpu_read(I2C &port, char i2c_addr, char reg_addr, char* dest, char size);
int mpu_probe(I2C &port, char i2c_addr);
int mpu_init(I2C &port, char i2c_addr);
int mpu_int_data(I2C &port, char i2c_addr, struct mpu_60x0_data* data, char big_to_little);


enum MPU_60X0_REG_ADDR{
    R_AUX_VDDIO         = 0x01,
    R_SMPLRT_DIV        = 0x19,
    R_CONFIG            = 0x1A,
    R_GYRO_CONFIG       = 0x1B,
    R_ACCEL_CONFIG      = 0x1C,
    R_FF_THR            = 0x1D,
    R_FF_DUR            = 0x1E,
    R_MOT_THR           = 0x1F,
    R_MOT_DUR           = 0x20,
    R_ZRMOT_THR         = 0x21,
    R_ZRMOT_DUR         = 0x22,
    R_FIFO_EN           = 0x23,
    R_I2C_MST_CTRL      = 0x24,
    R_I2C_SLV0_ADDR     = 0x25,
    R_I2C_SLV0_REG      = 0x26,
    R_I2C_SLV0_CTRL     = 0x27,
    R_I2C_SLV1_ADDR     = 0x28,
    R_I2C_SLV1_REG      = 0x29,
    R_I2C_SLV1_CTRL     = 0x2A,
    R_I2C_SLV2_ADDR     = 0x2B,
    R_I2C_SLV2_REG      = 0x2C,
    R_I2C_SLV2_CTRL     = 0x2D,
    R_I2C_SLV3_ADDR     = 0x2E,
    R_I2C_SLV3_REG      = 0x2F,
    R_I2C_SLV3_CTRL     = 0x30,
    R_I2C_SLV4_ADDR     = 0x31,
    R_I2C_SLV4_REG      = 0x32,
    R_I2C_SLV4_DO       = 0x33,
    R_I2C_SLV4_CTRL     = 0x34,
    R_I2C_SLV4_DI       = 0x35,
    R_I2C_MST_STATUS    = 0x36,
    R_INT_PIN_CFG       = 0x37,
    R_INT_ENABLE        = 0x38,
    R_INT_STATUS        = 0x3A,
    R_ACCEL_XOUT_H      = 0x3B,
    R_ACCEL_XOUT_L      = 0x3C,
    R_ACCEL_YOUT_H      = 0x3D,
    R_ACCEL_YOUT_L      = 0x3E,
    R_ACCEL_ZOUT_H      = 0x3F,
    R_ACCEL_ZOUT_L      = 0x40,
    R_TEMP_OUT_H        = 0x41,
    R_TEMP_OUT_L        = 0x42,
    R_GYRO_XOUT_H       = 0x43,
    R_GYRO_XOUT_L       = 0x44,
    R_GYRO_YOUT_H       = 0x45,
    R_GYRO_YOUT_L       = 0x46,
    R_GYRO_ZOUT_H       = 0x47,
    R_GYRO_ZOUT_L       = 0x48,
    R_EXT_SENS_DATA_00  = 0x49,
    R_EXT_SENS_DATA_01  = 0x4A,
    R_EXT_SENS_DATA_02  = 0x4B,
    R_EXT_SENS_DATA_03  = 0x4C,
    R_EXT_SENS_DATA_04  = 0x4D,
    R_EXT_SENS_DATA_05  = 0x4E,
    R_EXT_SENS_DATA_06  = 0x4F,
    R_EXT_SENS_DATA_07  = 0x50,
    R_EXT_SENS_DATA_08  = 0x51,
    R_EXT_SENS_DATA_09  = 0x52,
    R_EXT_SENS_DATA_10  = 0x53,
    R_EXT_SENS_DATA_11  = 0x54,
    R_EXT_SENS_DATA_12  = 0x55,
    R_EXT_SENS_DATA_13  = 0x56,
    R_EXT_SENS_DATA_14  = 0x57,
    R_EXT_SENS_DATA_15  = 0x58,
    R_EXT_SENS_DATA_16  = 0x59,
    R_EXT_SENS_DATA_17  = 0x5A,
    R_EXT_SENS_DATA_18  = 0x5B,
    R_EXT_SENS_DATA_19  = 0x5C,
    R_EXT_SENS_DATA_20  = 0x5D,
    R_EXT_SENS_DATA_21  = 0x5E,
    R_EXT_SENS_DATA_22  = 0x5F,
    R_EXT_SENS_DATA_23  = 0x60,
    R_MOT_DETECT_STATUS = 0x61,
    R_I2C_SLV0_DO       = 0x63,
    R_I2C_SLV1_DO       = 0x64,
    R_I2C_SLV2_DO       = 0x65,
    R_I2C_SLV3_DO       = 0x66,
    R_I2C_MST_DELAY_CTRL= 0x67,
    R_SIGNAL_PATH_RESET = 0x68,
    R_MOT_DETECT_CTRL   = 0x69,
    R_USER_CTRL         = 0x6A,
    R_PWR_MGMT_1        = 0x6B,
    R_PWR_MGMT_2        = 0x6C,
    R_FIFO_COUNTH       = 0x72,
    R_FIFO_COUNTL       = 0x73,
    R_FIFO_R_W          = 0x74,
    R_WHO_AM_I          = 0x75,
    // WHO_AM_I is the last register
    R_INVALID           = 0x76,
    R_REG_SPACE_SIZE    = R_INVALID
};
