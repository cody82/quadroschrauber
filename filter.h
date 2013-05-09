#pragma once
#include "GTMath.h"

class VectorLowPass : public Vector3
{
public:
    VectorLowPass(float hz);
    VectorLowPass();
    Vector3 Filter(const Vector3 &v, float dt);
    Vector3 Filter(const Vector3 &v, float dt, float hz);
    Vector3 Filter(const Vector3 &v, float dt, float hz_x, float hz_y, float hz_z);
    const Vector3& Current() const;
    operator Vector3 ();
protected:
    //Vector3 last;
    float hz;
};

class VectorHighPass : public Vector3
{
public:
    VectorHighPass(float hz);
    VectorHighPass();
    Vector3 Filter(const Vector3 &v, float dt);
    Vector3 Filter(const Vector3 &v, float dt, float hz);
    Vector3 Filter(const Vector3 &v, float dt, float hz_x, float hz_y, float hz_z);
protected:
    Vector3 last;
    bool first;
    float hz;
};

class VectorBandPass
{
};

class VectorIntegrator
{
public:
    Vector3 Filter(const Vector3 &v, float dt);
protected:
    Vector3 integral;
};

class VectorDerivator
{
public:
    VectorDerivator();
    Vector3 Filter(const Vector3 &v, float dt);
protected:
    Vector3 last;
    bool first;
};
