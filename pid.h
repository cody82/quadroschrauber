#pragma once

#include "GTMath.h"

class VectorPID
{
public:
    VectorPID();
    
    Vector3 Update(const Vector3 &error, float dtime);
    
    Vector3 error_derivate;
    Vector3 error_integral;
    
    Vector3 P;
    Vector3 I;
    Vector3 D;
    
    Vector3 I_max;
protected:
    Vector3 previous_error;
};
