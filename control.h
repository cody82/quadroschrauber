#pragma once

#include "GTMath.h"
#include "pid.h"
#include "filter.h"

class RemoteInput
{
public:
    RemoteInput();
    
    float roll;
    float pitch;
    float yaw;
    float throttle;
    
    bool active;
    
    bool switch1;
    bool switch2;
};

class SensorInput
{
public:
    SensorInput();
    Vector3 accel;
    Vector3 gyro;
    Vector3 magneto;
    float ultrasound_down;
};

class MotorOutput
{
public:
    MotorOutput();
    
    float motor_left;
    float motor_right;
    float motor_front;
    float motor_back;
};

class Controller
{
public:
    Controller();
    void Update(float dtime, RemoteInput &remote, SensorInput &sensors, MotorOutput &output);
    
    //Vector3 gyro_filter;
    //Vector3 accel_filter;
    //Vector3 magneto_filter;
    VectorLowPass gyro_filter;
    VectorLowPass accel_filter;
    VectorLowPass magneto_filter;
    
    Vector3 inner_pid_output;
    Vector3 outer_pid_output;
    
    Vector3 ahrs;
    Vector3 ahrs_dev;
    Vector3 ahrs_int;
    
    Vector3 remote_gain;
    Vector3 gain;
    
    
    VectorPID inner;
    VectorPID outer;
    
    float altitude_P;
    float altitude_hold;
    
protected:
    //Vector3 gyro_before;
    //Vector3 gyro_dev_before;
    Vector3 ahrs_before;
    Vector3 ahrs_dev_before;
    bool first;
};

class Controller2
{
public:
    Controller2();
    void Update(float dtime, RemoteInput &remote, SensorInput &sensors, MotorOutput &output);
    
    
    Vector3 ahrs;
};
