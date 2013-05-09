#include "pid.h"


#define CLAMP(X,MIN,MAX) if(X<(MIN))X=(MIN);else if(X>(MAX))X = (MAX);

VectorPID::VectorPID()
    : I_max(1,1,1)
{
}


Vector3 VectorPID::Update(const Vector3 &error, float dtime)
{
    error_integral += error * dtime;
    error_derivate = (error - previous_error) / dtime;
    previous_error = error;
    
    CLAMP(error_integral.x, -I_max.x, I_max.x);
    CLAMP(error_integral.y, -I_max.y, I_max.y);
    CLAMP(error_integral.z, -I_max.z, I_max.z);
    
    return error * P + error_integral * I + error_derivate * D;
}
