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

#include "mbed.h"
#include "mpu-60x0.h"

int mpu_read(I2C &port, char i2c_addr, char reg_addr, char* dest, char size)
{
    int     status          = 0;
    char    eight_bit_addr  = (i2c_addr & 0x7F) << 1;
    char *  current         = dest;
    char    num_bytes       = size;

    if (num_bytes != 0) {
        port.start();
        port.write(eight_bit_addr);
        port.write(reg_addr);
        port.start();
        port.write(eight_bit_addr | 0x1);
        
        while(num_bytes-- != 1) {
            *(current++)    = port.read(1);
        }
        *current    = port.read(0);
        port.stop();
        status = 1;
    }
    return status;
}

int mpu_write(I2C &port, char i2c_addr, char reg_addr, char* data, char size)
{
    int     status          = 0;
    char    eight_bit_addr  = (i2c_addr & 0x7F) << 1;
    char *  current         = data;
    char    num_bytes       = size;
    
    if (num_bytes != 0) {
        port.start();
        port.write(eight_bit_addr);
        port.write(reg_addr);
        
        while(num_bytes-- != 0) {
            port.write(*current++);
        }
        port.stop();
        status = 1;
    }
    return status;
}

int mpu_write_table(I2C &port, char i2c_addr, char table[][2])
{
    int ret = 1;
    int i=0;
    while(table[i][0] < R_INVALID) {
        if (!mpu_write(port, i2c_addr, table[i][0], &table[i][1], 1)) {
            ret = 0;
            break;
        }
        i++;
    }
    return ret;
}

int mpu_probe(I2C &port, char i2c_addr)
{
    char    data;
    char    reg = R_WHO_AM_I;
    char    size = sizeof(data);
    int     ret  = 0;
    if (mpu_read(port, i2c_addr, reg, &data, size) &&
        (data == i2c_addr))
    {
        ret = 1;
    }
    return ret;
}

int mpu_init(I2C &port, char i2c_addr)
{
    char init[][2] =
    {
        {R_PWR_MGMT_1,      0x1},
        //{R_GYRO_CONFIG,     0x0},
        //{R_GYRO_CONFIG,     0x8},// 500 deg/s
        {R_GYRO_CONFIG,     0x10},// 1000 deg/s
        //{R_ACCEL_CONFIG,    0x0},
        {R_ACCEL_CONFIG,    0x8}, //4g
        {R_INVALID,         0x0}
    };
    return mpu_write_table(port, i2c_addr, init);
}

int mpu_int_data(I2C &port, char i2c_addr, struct mpu_60x0_data *data, char big_to_little)
{
    int ret     = 0;
    char i      = 0;
    char size   = (R_GYRO_ZOUT_L + 1) - R_ACCEL_XOUT_H;
    char temp;
    char* byte_data = (char *)data;
    
    if (mpu_read(port, i2c_addr, R_ACCEL_XOUT_H, (char*)byte_data, size)) {
        ret = 1;
    }
    if (big_to_little) {
        for (i = 0; i < size - 1; i+=2) {
            temp             = byte_data[i];
            byte_data[i]     = byte_data[i+1];
            byte_data[i+1]   = temp;
        }
    }
    return ret;
}
