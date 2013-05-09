#pragma once

#include "mbed.h"

#include "freeimu430.h"
#include "blinkm.h"
#include "control.h"
#include "motor.h"
#include "config.h"
#include "TinyGPS.h"
#include "sr04.h"
#include "MaxSonar-EZ1.h"

#include "MODSERIAL.h"
#include "Watchdog.h"

#include "mavlink/common/mavlink.h"


#define BATTERY_MIN_VOLTAGE (9.6f)
//#define BATTERY_MIN_VOLTAGE (-1000.0f)

class QuadroSchrauber
{
public:
    static Vector3 forward;
    static Vector3 left;
    static Vector3 up;
    
    Watchdog wd;
    I2C i2c;
    BlinkM blinkm;
    FreeIMU430 imu;
    MODSERIAL pc;
    //MODSERIAL bluetooth;
    MODSERIAL remote_serial;
    TinyGPS gps;
    PwmOut pwm1;
    PwmOut pwm2;
    PwmOut pwm3;
    PwmOut pwm4;
    DigitalOut led1;
    DigitalOut led2;
    DigitalOut led3;
    DigitalOut led4;
    Timer timer;
    AnalogIn battery_voltage;
    
    Motor motor_left;
    Motor motor_front;
    Motor motor_back;
    Motor motor_right;
    
    //SR04 ultrasound;
    MaxSonarPWM ultrasound;
    
    Controller controller;
    
    bool ok;
    int counter;
    
    QuadroSchrauber();
    
    void Shutdown();
    void Update(float dtime);
    void Reset();
    
    bool GetSensorData(float dtime, SensorInput &data);
    bool GetRemoteData(RemoteInput &data);
    bool SetMotors(MotorOutput &data);

    SensorInput sensors;
    RemoteInput remote;
    MotorOutput motors;
    uint16_t ch1[6];
    
    LocalFileSystem local;
    
    #define QUADROSCHRAUBER_CONFIG { \
        {"ctrl_gain_x", &controller.gain.x}, \
        {"ctrl_gain_y", &controller.gain.y}, \
        {"ctrl_gain_z", &controller.gain.z}, \
        {"ctrl_inner_p_x", &controller.inner.P.x}, \
        {"ctrl_inner_p_y", &controller.inner.P.y}, \
        {"ctrl_inner_p_z", &controller.inner.P.z}, \
        {"ctrl_inner_i_x", &controller.inner.I.x}, \
        {"ctrl_inner_i_y", &controller.inner.I.y}, \
        {"ctrl_inner_i_z", &controller.inner.I.z}, \
        {"ctrl_inner_d_x", &controller.inner.D.x}, \
        {"ctrl_inner_d_y", &controller.inner.D.y}, \
        {"ctrl_inner_d_z", &controller.inner.D.z}, \
        {"ctrl_inner_im_x", &controller.inner.I_max.x}, \
        {"ctrl_inner_im_y", &controller.inner.I_max.y}, \
        {"ctrl_inner_im_z", &controller.inner.I_max.z}, \
        {"ctrl_outer_p_x", &controller.outer.P.x}, \
        {"ctrl_outer_p_y", &controller.outer.P.y}, \
        {"ctrl_outer_p_z", &controller.outer.P.z}, \
        {"ctrl_outer_i_x", &controller.outer.I.x}, \
        {"ctrl_outer_i_y", &controller.outer.I.y}, \
        {"ctrl_outer_i_z", &controller.outer.I.z}, \
        {"ctrl_outer_d_x", &controller.outer.D.x}, \
        {"ctrl_outer_d_y", &controller.outer.D.y}, \
        {"ctrl_outer_d_z", &controller.outer.D.z}, \
        {"ctrl_outer_im_x", &controller.outer.I_max.x}, \
        {"ctrl_outer_im_y", &controller.outer.I_max.y}, \
        {"ctrl_outer_im_z", &controller.outer.I_max.z}, \
        {"remote_gain_x", &controller.remote_gain.x}, \
        {"remote_gain_y", &controller.remote_gain.y}, \
        {"remote_gain_z", &controller.remote_gain.z}, \
        {"altitude_p", &controller.altitude_P} \
        }
    #define QUADROSCHRAUBER_CONFIG_COUNT 31
    
    void LoadConfig();
    void SaveConfig();
    float GetBatteryVoltage();
    float voltage;
    bool voltage_ok;
    
    bool save_flag;
    
    float loop_time_max;
protected:
    int remote_time;
    char remote_data[16];
    void ReceiveRemote(char c);
    //void ReceiveCallback(MODSERIAL_IRQ_INFO *info);
    void ReceiveCallback();
    void ReceiveChar(uint8_t c, uint8_t com);
    void Send(uint8_t c);
    void Send(uint8_t *buf, int len);

    void mavlink_send_heartbeat();
    void mavlink_send_data();
    void mavlink_send_text(const char *text);
    mavlink_system_t mavlink_system;
    uint8_t mavlink_system_mode;
                 uint8_t buf[MAVLINK_MAX_PACKET_LEN];
   
    void LoadConfigValue(const char *name, float value);
};


