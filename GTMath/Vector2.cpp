#include "mbed.h"
#include "GTMath.h"

//Konstruktor
Vector2::Vector2()
{}
Vector2::Vector2(float all) :
    x(all),
    y(all)
{}
Vector2::Vector2(float _x, float _y) :
    x(_x),
    y(_y)
{}
Vector2::Vector2(const Vector2 & v) :
    x(v.x),
    y(v.y)
{}

//Casting
Vector2::operator float* ()
{
    return(af);
}

//Arithmetik komponentenweise
const Vector2 Vector2::operator +(const Vector2 & v) const
{
    return(Vector2(x+v.x, y+v.y));
}
const Vector2 Vector2::operator -(const Vector2 & v) const
{
    return(Vector2(x-v.x, y-v.y));
}
const Vector2 Vector2::operator *(const Vector2 & v) const
{
    return(Vector2(x*v.x, y*v.y));
}
const Vector2 Vector2::operator /(const Vector2 & v) const
{
    return(Vector2(x/v.x, y/v.y));
}
const Vector2 Vector2::operator *(float f) const
{
    return(Vector2(x*f, y*f));
}
const Vector2 Vector2::operator /(float f) const
{
    const float fi= 1.0 / f;
    return(Vector2(x*fi, y*fi));
}

//Zuweisungsoperatoren komponentenweise
Vector2 & Vector2::operator =(const Vector2 & v)
{
    x= v.x;
    y= v.y;
    return(*this);
}
Vector2 & Vector2::operator +=(const Vector2 & v)
{
    x+= v.x;
    y+= v.y;
    return(*this);
}
Vector2 & Vector2::operator -=(const Vector2 & v)
{
    x-= v.x;
    y-= v.y;
    return(*this);
}
Vector2 & Vector2::operator *=(const Vector2 & v)
{
    x*= v.x;
    y*= v.y;
    return(*this);
}
Vector2 & Vector2::operator /=(const Vector2 & v)
{
    x/= v.x;
    y/= v.y;
    return(*this);
}
Vector2 & Vector2::operator *=(float f)
{
    x*= f;
    y*= f;
    return(*this);
}
Vector2 & Vector2::operator /=(float f)
{
    const float fi= 1.0 / f;
    x*= fi;
    y*= fi;
    return(*this);
}

//Vergleich
bool Vector2::operator == (const Vector2 & v) const
{
    return(x==v.x && y==v.y);
}
bool Vector2::operator != (const Vector2 & v) const
{
    return(x!=v.x || y!=v.y);
}

//Min, Max
const Vector2 Vector2::Min(const Vector2 & v) const
{
    return(Vector2(TMin(x, v.x), TMin(y, v.y)));
}
const Vector2 Vector2::Max(const Vector2 & v) const
{
    return(Vector2(TMax(x, v.x), TMax(y, v.y)));
}

//Skalarprodukt, Winkel
float Vector2::DotP(const Vector2 & v) const
{
    return(x*v.x + y*v.y);
}
float Vector2::Angle(const Vector2 &v) const                    
{
    float c= (x*v.x + y*v.y) / sqrtf((x*x + y*y ) * (v.x * v.x + v.y * v.y));
    return(c < -1.0f ? Pi : c > 1.0f ? 0.0f : acosf(c));
}

//Laenge, Quadratlaenge, Nomralisieren
float Vector2::Length() const                    
{    
    return(sqrtf(x*x + y*y));    
}
float Vector2::LengthSq() const                    
{    
    return(x*x + y*y);
}
const Vector2 Vector2::Normalize() const        
{
    return(*this / sqrtf(x*x + y*y));
}


//Operatoren, die nicht innerhalt der Klasse definiert werden koennen
const Vector2 operator - (const Vector2 & v)
{
    return(Vector2(-v.x, -v.y));
}
const Vector2 operator * (float f, const Vector2 & v)
{
    return(Vector2(f*v.x, f*v.y));
}
const Vector2 operator / (float f, const Vector2 & v)
{
    const float fi= 1.0 / f;
    return(Vector2(fi*v.x, fi*v.y));
}