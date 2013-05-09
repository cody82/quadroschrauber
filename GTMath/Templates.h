#pragma once

//The number Pi
static const float Pi= 3.1415926535897932384626433832795f;

//Minimum
template<typename T>
const T & TMin(const T & A, const T & B)
{
    return(A < B ? A : B);
}

//Maximum
template<typename T>
const T & TMax(const T & A, const T & B)
{
    return(A > B ? A : B);
}

//Lineare Interpolation
template<typename T, typename S>
const T TLerp(const T & A, const T & B, const S f)
{
    return(A*f + B*(1.0-f));
}
