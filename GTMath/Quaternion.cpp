#include "GTMath.h"

///////////////////////////////////////////////////////////////////
//Konstruktoren
Quaternion::Quaternion()
{}
Quaternion::Quaternion(const Quaternion& q) :
    w(q.w),
    v(q.v.x, q.v.y, q.v.z)
{}
Quaternion::Quaternion(float _w, const Vector3& _v) :
    w(_w),
    v(_v)
{}
Quaternion::Quaternion(float _w, float _x= 0.0F, float _y= 0.0F, float _z= 0.0F) :
    w(_w),
    v(_x, _y, _z)
{}


//////////////////////////////////////////////////////////////////
//Casting
//Direkter Zugriff auf die Daten
Quaternion::operator float* ()
{
    return(&w);
}


///////////////////////////////////////////////////////////////////
//Zuweisungen
Quaternion& Quaternion::operator =(const Quaternion& q)
{    
    w= q.w;
    v= q.v;
    return(*this);
}
Quaternion& Quaternion::operator+=(const Quaternion& q)
{    
    w+= q.w;
    v+= q.v;
    return(*this);
}
Quaternion& Quaternion::operator-=(const Quaternion& q)    
{    
    w-= q.w;
    v-= q.v;
    return(*this);    
}
Quaternion& Quaternion::operator*=(const Quaternion& q)    
{    
    const float a= (v.z - v.y) * (q.v.y - q.v.z),
                b= (  w + v.x) * (q.  w + q.v.x),
                c= (  w - v.x) * (q.v.y + q.v.z),
                d= (v.y + v.z) * (q.  w - q.v.x),
                e= (v.z - v.x) * (q.v.x - q.v.y),
                f= (v.z + v.x) * (q.v.x + q.v.y),
                g= (  w + v.y) * (q.  w - q.v.z),
                h= (  w - v.y) * (q.  w + q.v.z),
                i= f + g + h,
                j= 0.5F * (e + i);

    return(*this= Quaternion(a + j - f,
                             b + j - i,
                             c + j - h,
                             d + j - g));
}
Quaternion& Quaternion::operator/=(const Quaternion& q)
{        
    return(*this*= q.Invert());
}
Quaternion& Quaternion::operator*=(float f)
{    
    w*= f;
    v*= f;
    return(*this);
}
Quaternion& Quaternion::operator/=(float f)    
{    
    f= 1.0F/f;
    w*= f; 
    v*= f; 
    return(*this);
}

////////////////////////////////////////////////////////////
//Arithmetik
const Quaternion Quaternion::operator+(const Quaternion& q) const
{    
    return(Quaternion(w+q.w, v.x+q.v.x, v.y+q.v.y, v.z+q.v.z));    
}
const Quaternion Quaternion::operator-(const Quaternion& q) const    
{    
    return(Quaternion(w-q.w, v.x-q.v.x, v.y-q.v.y, v.z-q.v.z));
}
const Quaternion Quaternion::operator*(const Quaternion& q) const
{
    const float a= (v.z - v.y) * (q.v.y - q.v.z),
                b= (  w + v.x) * (q.  w + q.v.x),
                c= (  w - v.x) * (q.v.y + q.v.z),
                d= (v.y + v.z) * (q.  w - q.v.x),
                e= (v.z - v.x) * (q.v.x - q.v.y),
                f= (v.z + v.x) * (q.v.x + q.v.y),
                g= (  w + v.y) * (q.  w - q.v.z),
                h= (  w - v.y) * (q.  w + q.v.z),
                i= f + g + h,
                j= 0.5F * (e + i);

    return(Quaternion(a + j - f,
                      b + j - i,
                      c + j - h,
                      d + j - g));
}
const Quaternion Quaternion::operator/(const Quaternion& q) const
{    
    return(*this * q.Invert());
}
const Quaternion Quaternion::operator*(float f) const
{
    return(Quaternion(w*f, v*f));
}
const Quaternion Quaternion::operator/(float f) const
{
    f= 1.0F/f; return(Quaternion(w*f, v*f));
}        
const Quaternion Quaternion::operator-() const
{
    return(Quaternion(-w, -v.x, -v.y, -v.z));
}

////////////////////////////////////////////////////////////
//Identit&#65533;quaternion
const Quaternion Quaternion::MulIdentity()    
{    
    return(Quaternion(1.0F, 0.0F, 0.0F, 0.0F));
}
const Quaternion Quaternion::AddIdentity()
{    
    return(Quaternion(0.0F, 0.0F, 0.0F, 0.0F));    
}

////////////////////////////////////////////////////////////
//Betrag (=L&#65533;e), Normalisieren
float Quaternion::Length() const    
{    
    return(sqrtf(w*w + v.x*v.x + v.y*v.y + v.z*v.z));
}
float Quaternion::LengthSq() const        
{    
    return(w*w + v.x*v.x + v.y*v.y + v.z*v.z); 
}
const Quaternion Quaternion::Normalize() const    
{    
    return(*this / Length());
}


////////////////////////////////////////////////////////////
//Konjugieren , Invertieren
const Quaternion Quaternion::Conjugate() const    
{    
    return(Quaternion(w, -v.x, -v.y, -v.z));    
}
const Quaternion Quaternion::Invert() const        
{    
    const float fLengthSqInv= 1.0F / (w*w + v.x*v.x + v.y*v.y + v.z*v.z);
    return(Quaternion(  w * fLengthSqInv,
                     -v.x * fLengthSqInv,
                     -v.y * fLengthSqInv,
                     -v.z * fLengthSqInv));
}

////////////////////////////////////////////////////////////
//Punktprodukt berechnen
float Quaternion::DotP(const Quaternion& q) const
{
    return(w*q.w + v.x*q.v.x + v.y*q.v.y + v.z*q.v.z);
}

////////////////////////////////////////////////////////////
//Rotation von Vektoren
const Vector3 Quaternion::RotateVector(const Vector3 &v) const
{
    //nach der Formel: x'= q * x * qk
    return((*this * Quaternion(0, v) * Conjugate()).v);
}


////////////////////////////////////////////////////////////
//Konstruieren
//aus: Normale und Winkel(Radiant!)
const Quaternion Quaternion::CreateFromAxisAndAngle(const Vector3& vAxis, float fAngle)    
{    
    fAngle*= 0.5F; //Halbieren, da immer nur die H&#65533;te des Winkels ben&#65533;t wird
    return(Quaternion(cosf(fAngle), 
        vAxis * sinf(fAngle)));
}
//aus: Euler-Rotation (=3 Winkel als Vektor3)
const Quaternion Quaternion::CreateFromEuler(Vector3 vEulerAngles)
{
    vEulerAngles*= 0.5F; //Halbieren, da immer nur die H&#65533;te der Winkel ben&#65533;t wird

    return( Quaternion(cosf(vEulerAngles.x), sinf(vEulerAngles.x), 0.0F, 0.0F) *
            Quaternion(cosf(vEulerAngles.y), 0.0F, sinf(vEulerAngles.y), 0.0F) *
            Quaternion(cosf(vEulerAngles.z), 0.0F, 0.0F, sinf(vEulerAngles.z)) );
}
//aus: Matrix3x3
const Quaternion Quaternion::CreateFromMatrix(const Matrix3x3 & m)
{
    float qw, qx, qy, qz;
    float tr= m.m11 + m.m22 + m.m33;

    if(tr > 0)
    { 
        float S = sqrt(tr+1.0) * 2; // S=4*qw 
        qw = 0.25 * S;
        qx = (m.m32 - m.m23) / S;
        qy = (m.m13 - m.m31) / S; 
        qz = (m.m21 - m.m12) / S; 
    }
    else if((m.m11 > m.m22)&(m.m11 > m.m33))
    { 
        float S = sqrt(1.0 + m.m11 - m.m22 - m.m33) * 2; // S=4*qx 
        qw = (m.m32 - m.m23) / S;
        qx = 0.25 * S;
        qy = (m.m12 + m.m21) / S; 
        qz = (m.m13 + m.m31) / S; 
    }
    else if(m.m22 > m.m33)
    { 
        float S = sqrt(1.0 + m.m22 - m.m11 - m.m33) * 2; // S=4*qy
        qw = (m.m13 - m.m31) / S;
        qx = (m.m12 + m.m21) / S; 
        qy = 0.25 * S;
        qz = (m.m23 + m.m32) / S; 
    }
    else
    { 
        float S = sqrt(1.0 + m.m33 - m.m11 - m.m22) * 2; // S=4*qz
        qw = (m.m21 - m.m12) / S;
        qx = (m.m13 + m.m31) / S;
        qy = (m.m23 + m.m32) / S;
        qz = 0.25 * S;
    }
    return(Quaternion(qw, qx, qy, qz));
}

//Quaternion in Euler-Winkel umwandeln
const Vector3 Quaternion::CalcEulerAngles() const
{
    return(Vector3( asin(-2.0 * (w*v.x + v.y*v.z)),
                   atan2(-2.0 * (w*v.y - v.x*v.z), 1.0 - 2.0 * (v.y*v.y + v.x*v.x)),
                   atan2(-2.0 * (w*v.z - v.x*v.y), 1.0 - 2.0 * (v.x*v.x + v.z*v.z)) ));
}

////////////////////////////////////////////////////////////
//Lineare Interpolation
const Quaternion Quaternion::Lerp(Quaternion q, float f) const    
{   
   return((*this*f + q*(1.0F-f)).Normalize());
}
//Sphaerische lineare Interpolation
const Quaternion Quaternion::Slerp(Quaternion q, float f) const    
{    
     //den kuerzeren weg gehen
    if(DotP(q) < 0)
        q= -q;
    const float fAngle= v.Angle(q.v);
    if(fAngle < 0.1F)
        return((*this*f + q*(1.0F-f)).Normalize());
    
    const float fInvSinAngle= 1.0F / sinf(fAngle);    
    return(*this * sinf(      f  * fAngle) * fInvSinAngle
             + q * sinf((1.0F-f) * fAngle) * fInvSinAngle);
}
