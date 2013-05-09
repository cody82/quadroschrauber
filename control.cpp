#include "control.h"

#include "mbed.h"

#include "MadgwickAHRS.h"

#include "GTMath.h"


SensorInput::SensorInput()
    : ultrasound_down(-1)
{
}

RemoteInput::RemoteInput()
    : roll(0), pitch(0), yaw(0), throttle(0), active(false), switch1(false), switch2(false)
{
}

MotorOutput::MotorOutput()
    : motor_left(0), motor_right(0), motor_front(0), motor_back(0)
{
}

Controller::Controller()
    : first(true), remote_gain(Vector3(0.50f,0.50f,0.50f)), gain(Vector3(0.05f,0.05f,0.05f)), altitude_P(0.2f), altitude_hold(-1)
{
}


#define CLAMP(X,MIN,MAX) if(X<(MIN))X=(MIN);else if(X>(MAX))X = (MAX);

#define ALPHA(HZ,DT) ((DT)/((DT)+(1.0f/(2.0f*3.141f*(float)(HZ)))))

#define RAD(DEG) (((float)DEG)/180.0f*3.141f)

void Controller::Update(float dtime, RemoteInput &remote, SensorInput &sensors, MotorOutput &output)
{
    //gyro_filter.Filter(sensors.gyro, dtime, 1000);
    gyro_filter.Filter(sensors.gyro, dtime, 1000, 1000, 4);
    //accel_filter.Filter(sensors.accel, dtime, 2);
    accel_filter.Filter(sensors.accel, dtime, 10);
    magneto_filter.Filter(sensors.magneto, dtime, 2);
    
    if(false)
    {
        MadgwickAHRSupdate(gyro_filter.x, gyro_filter.y, gyro_filter.z, accel_filter.x, accel_filter.y, accel_filter.z, magneto_filter.x, magneto_filter.y, magneto_filter.z);
        Quaternion q(q0,q1,q2,q3);
        Vector3 v = q.CalcEulerAngles();
        ahrs = Vector3(-v.x,-v.y,-v.z + 3.141f * 5.0f / 6.0f);
        
        if(first)
        {
            ahrs_dev_before = ahrs;
            first = false;
        }
    
        ahrs_dev = gyro_filter;
    }
    else
    {
        if(accel_filter.y == 0 && accel_filter.z == 0)
            ahrs.x = 0;
        else
            ahrs.x = (ahrs.x + gyro_filter.x * dtime) * 0.98f + atan2(accel_filter.y, accel_filter.z) * 0.02f;
            
        if(accel_filter.x == 0 && accel_filter.z == 0)
            ahrs.y = 0;
        else
            ahrs.y = (ahrs.y + gyro_filter.y * dtime) * 0.98f - atan2(accel_filter.x, accel_filter.z) * 0.02f;
            
        ahrs.z = ahrs.z + gyro_filter.z * dtime;
        
        ahrs_dev = gyro_filter;
    }
    
    
    Vector3 setpoint = Vector3(remote.roll * remote_gain.x, remote.pitch * remote_gain.y, remote.yaw * remote_gain.z);
    
    outer_pid_output = outer.Update(setpoint - ahrs, dtime);
    outer_pid_output.z = setpoint.z;
    
    inner_pid_output = inner.Update(outer_pid_output - ahrs_dev, dtime);
    
    inner_pid_output *= gain;
    const float max_control = 0.2f;
    CLAMP(inner_pid_output.x, -max_control, max_control);
    CLAMP(inner_pid_output.y, -max_control, max_control);
    CLAMP(inner_pid_output.z, -max_control, max_control);
    
    float altitude_compensation = 0.0f;
    /*
    if(altitude_hold < 0 && remote.switch1 && remote.active && sensors.ultrasound_down > 0)
    {
        //altitude hold turned on
        altitude_hold = sensors.ultrasound_down;
    }
    else if(altitude_hold >= 0 && !remote.switch1 && remote.active)
    {
        //altitude hold turned off
        altitude_hold = -1.0f;
    }
    
    if(altitude_hold >= 0 && sensors.ultrasound_down > 0)
    {
        altitude_compensation = (sensors.ultrasound_down - altitude_hold) * 0.2f;
        CLAMP(altitude_compensation, -0.2f, 0.2f);
    }
    else if(fabs(ahrs.x) < RAD(40) && fabs(ahrs.y) < RAD(40))
    {
        altitude_compensation = (1.0f - accel_filter.z) * altitude_P;
    }
    */
    
    if(remote.active && remote.throttle >= 0.05f)
    {
        output.motor_left = remote.throttle + inner_pid_output.x - inner_pid_output.z + altitude_compensation;
        output.motor_right = remote.throttle - inner_pid_output.x - inner_pid_output.z + altitude_compensation;
        output.motor_front = remote.throttle + inner_pid_output.y + inner_pid_output.z + altitude_compensation;
        output.motor_back = remote.throttle - inner_pid_output.y + inner_pid_output.z + altitude_compensation;
    }
    else
    {
        output.motor_left = 0;
        output.motor_right = 0;
        output.motor_front = 0;
        output.motor_back = 0;
    }
    
}

Controller2::Controller2()
{


}

void Controller2::Update(float dtime, RemoteInput &remote, SensorInput &sensors, MotorOutput &output)
{
}