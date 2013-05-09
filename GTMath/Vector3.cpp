#include "mbed.h"
#include "GTMath.h"

//Konstruktor
Vector3::Vector3()
{}
Vector3::Vector3(float all) :
    x(all),
    y(all),
    z(all)
{}
Vector3::Vector3(float _x, float _y, float _z) :
    x(_x),
    y(_y),
    z(_z)
{}
Vector3::Vector3(const Vector3 & v) :
    x(v.x),
    y(v.y),
    z(v.z)
{}

//Casting
Vector3::operator float* ()
{
    return(af);
}

//Arithmetik komponentenweise
const Vector3 Vector3::operator +(const Vector3 & v) const
{
    return(Vector3(x+v.x, y+v.y, z+v.z));
}
const Vector3 Vector3::operator -(const Vector3 & v) const
{
    return(Vector3(x-v.x, y-v.y, z-v.z));
}
const Vector3 Vector3::operator *(const Vector3 & v) const
{
    return(Vector3(x*v.x, y*v.y, z*v.z));
}
const Vector3 Vector3::operator /(const Vector3 & v) const
{
    return(Vector3(x/v.x, y/v.y, z/v.z));
}
const Vector3 Vector3::operator *(float f) const
{
    return(Vector3(x*f, y*f, z*f));
}
const Vector3 Vector3::operator /(float f) const
{
    const float fi= 1.0 / f;
    return(Vector3(x*fi, y*fi, z*fi));
}

//Zuweisungsoperatoren komponentenweise
Vector3 & Vector3::operator =(const Vector3 & v)
{
    x= v.x;
    y= v.y;
    z= v.z;
    return(*this);
}
Vector3 & Vector3::operator +=(const Vector3 & v)
{
    x+= v.x;
    y+= v.y;
    z+= v.z;
    return(*this);
}
Vector3 & Vector3::operator -=(const Vector3 & v)
{
    x-= v.x;
    y-= v.y;
    z-= v.z;
    return(*this);
}
Vector3 & Vector3::operator *=(const Vector3 & v)
{
    x*= v.x;
    y*= v.y;
    z*= v.z;
    return(*this);
}
Vector3 & Vector3::operator /=(const Vector3 & v)
{
    x/= v.x;
    y/= v.y;
    z/= v.z;
    return(*this);
}
Vector3 & Vector3::operator *=(float f)
{
    x*= f;
    y*= f;
    z*= f;
    return(*this);
}
Vector3 & Vector3::operator /=(float f)
{
    const float fi= 1.0 / f;
    x*= fi;
    y*= fi;
    z*= fi;
    return(*this);
}

//Vergleich
bool Vector3::operator == (const Vector3 & v) const
{
    return(x==v.x && y==v.y && z==v.z);
}
bool Vector3::operator != (const Vector3 & v) const
{
    return(x!=v.x || y!=v.y || z!=v.z);
}

//Min, Max
const Vector3 Vector3::Min(const Vector3 & v) const
{
    return(Vector3(TMin(x, v.x), TMin(y, v.y), TMin(z, v.z)));
}
const Vector3 Vector3::Max(const Vector3 & v) const
{
    return(Vector3(TMax(x, v.x), TMax(y, v.y), TMax(z, v.z)));
}

//Skalar- und Kreuzprodukt, Winkel
float Vector3::DotP(const Vector3 & v) const
{
    return(x*v.x + y*v.y + z*v.z);
}
const Vector3 Vector3::CrossP(const Vector3 & v) const
{
    return(Vector3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x));
}
float Vector3::Angle(const Vector3 &v) const                    
{    
    float c= (x*v.x + y*v.y + z*v.z) / sqrtf((x*x + y*y + z*z) * (v.x * v.x + v.y * v.y + v.z * v.z));
    return(c < -1.0f ? Pi : c > 1.0f ? 0.0f : acosf(c));
}

//Laenge, Quadratlaenge, Nomralisieren
float Vector3::Length() const                    
{    
    return(sqrtf(x*x + y*y + z*z));    
}
float Vector3::LengthSq() const                    
{    
    return(x*x + y*y + z*z);
}
const Vector3 Vector3::Normalize() const        
{
    return(*this / sqrtf(x*x + y*y + z*z));
}


//Operatoren, die nicht innerhalt der Klasse definiert werden koennen
const Vector3 operator - (const Vector3 & v)
{
    return(Vector3(-v.x, -v.y, -v.z));
}
const Vector3 operator * (float f, const Vector3 & v)
{
    return(Vector3(f*v.x, f*v.y, f*v.z));
}
const Vector3 operator / (float f, const Vector3 & v)
{
    const float fi= 1.0 / f;
    return(Vector3(fi*v.x, fi*v.y, fi*v.z));
}