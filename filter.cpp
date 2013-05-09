#include "filter.h"

#define ALPHA(HZ,DT) ((DT)/((DT)+(1.0f/(2.0f*3.141f*(float)(HZ)))))

VectorLowPass::VectorLowPass(float hz)
    : Vector3(0,0,0)
{
    this->hz = hz;
}

VectorLowPass::VectorLowPass()
    : Vector3(0,0,0)
{
    this->hz = 100000000;
}


Vector3 VectorLowPass::Filter(const Vector3 &v, float dt)
{
    return Filter(v, dt, hz);
}

Vector3 VectorLowPass::Filter(const Vector3 &v, float dt, float hz)
{
    float alpha = ALPHA(hz,dt);
    Vector3 last = v * alpha + (*this) * (1.0f - alpha);
    x = last.x;
    y = last.y;
    z = last.z;
    return last;
}


Vector3 VectorLowPass::Filter(const Vector3 &v, float dt, float hz_x, float hz_y, float hz_z)
{
    Vector3 alpha = Vector3(ALPHA(hz_x,dt), ALPHA(hz_y,dt), ALPHA(hz_z,dt));
    Vector3 last = v * alpha + (*this) * (Vector3(1.0f) - alpha);
    x = last.x;
    y = last.y;
    z = last.z;
    return last;
}
    
const Vector3& VectorLowPass::Current() const
{
    return Vector3(x,y,z);
}

VectorLowPass::operator Vector3 ()
{
    return Current();
}

Vector3 VectorIntegrator::Filter(const Vector3 &v, float dt)
{
    integral += v * dt;
    return integral;
}

VectorDerivator::VectorDerivator()
    : first(true)
{
}
    
Vector3 VectorDerivator::Filter(const Vector3 &v, float dt)
{
    if(first)
    {
        first = false;
        last = v;
    }
    
    Vector3 ret = (v - last) / dt;
    
    last = v;
    
    return ret;
}


VectorHighPass::VectorHighPass(float hz)
    : first(true)
{
    this->hz = hz;
}

VectorHighPass::VectorHighPass()
    : hz(0.001f), first(true)
{
}

Vector3 VectorHighPass::Filter(const Vector3 &v, float dt)
{
    return Filter(v, dt, hz);
}

Vector3 VectorHighPass::Filter(const Vector3 &v, float dt, float hz)
{
    return Filter(v, dt, hz, hz, hz);
}

Vector3 VectorHighPass::Filter(const Vector3 &v, float dt, float hz_x, float hz_y, float hz_z)
{
    if(first)
    {
        first = false;
        last = v;
    }
    
    Vector3 d = (v - last) / dt;
    last = v;
    
    Vector3 alpha = Vector3(ALPHA(hz_x,dt), ALPHA(hz_y,dt), ALPHA(hz_z,dt));
    *((Vector3*)this) = alpha * ((*this) + d);
    
    return *this;
}
