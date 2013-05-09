#include "GTMath.h"

//Konstruktoren
Matrix3x3::Matrix3x3()
{}

Matrix3x3::Matrix3x3(float m11, float m12, float m13,
                     float m21, float m22, float m23,
                     float m31, float m32, float m33) :
    m11(m11), m12(m12), m13(m13),
    m21(m21), m22(m22), m23(m23),
    m31(m31), m32(m32), m33(m33)
{}

Matrix3x3::Matrix3x3(const Matrix3x3 &m) :
    m11(m.m11), m12(m.m12), m13(m.m13),
    m21(m.m21), m22(m.m22), m23(m.m23),
    m31(m.m31), m32(m.m32), m33(m.m33)
{}


//Casting
//Direkter Zugriff auf die Daten
Matrix3x3::operator float* ()
{
    return(af);
}

//operator() f&#65533; besseren Zugriff
float Matrix3x3::operator()(int iRow, int iColumn) const    {    return(aaf[iRow-1][iColumn-1]);    }
float & Matrix3x3::operator()(int iRow, int iColumn)        {    return(aaf[iRow-1][iColumn-1]);    }

//Zuweisungen
Matrix3x3 & Matrix3x3::operator = (const Matrix3x3 &m)
{
    m11= m.m11; m12= m.m12; m13= m.m13;
    m21= m.m21; m22= m.m22; m23= m.m23;
    m31= m.m31; m32= m.m32; m33= m.m33;
    return(*this);
}
Matrix3x3 & Matrix3x3::operator += (const Matrix3x3 &m)
{
    m11+= m.m11; m12+= m.m12; m13+= m.m13;
    m21+= m.m21; m22+= m.m22; m23+= m.m23;
    m31+= m.m31; m32+= m.m32; m33+= m.m33;
    return(*this);
}
Matrix3x3 & Matrix3x3::operator-=(const Matrix3x3 &m)
{
    m11-= m.m11; m12-= m.m12; m13-= m.m13;
    m21-= m.m21; m22-= m.m22; m23-= m.m23;
    m31-= m.m31; m32-= m.m32; m33-= m.m33;
    return(*this);
}
Matrix3x3 & Matrix3x3::operator*=(const Matrix3x3 &m)
{
    return(*this= Matrix3x3(m.m11 * m11 + m.m21 * m12 + m.m31 * m13,
                            m.m12 * m11 + m.m22 * m12 + m.m32 * m13,
                            m.m13 * m11 + m.m23 * m12 + m.m33 * m13,
                            m.m11 * m21 + m.m21 * m22 + m.m31 * m23,
                            m.m12 * m21 + m.m22 * m22 + m.m32 * m23,
                            m.m13 * m21 + m.m23 * m22 + m.m33 * m23,
                            m.m11 * m31 + m.m21 * m32 + m.m31 * m33,
                            m.m12 * m31 + m.m22 * m32 + m.m32 * m33,
                            m.m13 * m31 + m.m23 * m32 + m.m33 * m33));
}
Matrix3x3 & Matrix3x3::operator/=(const Matrix3x3 &m)
{
    return(*this *= m.Invert());
}

Matrix3x3 & Matrix3x3::operator*=(float f)
{
    m11*=f; m12*=f; m13*=f;
    m21*=f; m22*=f; m23*=f; 
    m31*=f; m32*=f; m33*=f;
    return *this;
}
Matrix3x3 & Matrix3x3::operator/=(float f)
{
    const float fInv= 1.0 / f;
    m11*=fInv; m12*=fInv; m13*=fInv;
    m21*=fInv; m22*=fInv; m23*=fInv;
    m31*=fInv; m32*=fInv; m33*=fInv;
    return *this;
}

////////////////////////////////////////////////////////////
//Arithmetik
const Matrix3x3 Matrix3x3::operator+(const Matrix3x3 &m) const
{
    return(Matrix3x3(m11+m.m11, m12+m.m12, m13+m.m13,
                     m21+m.m21, m22+m.m22, m23+m.m23,
                     m31+m.m31, m32+m.m32, m33+m.m33));
}
const Matrix3x3 Matrix3x3::operator-(const Matrix3x3 &m) const
{
    return(Matrix3x3(m11-m.m11, m12-m.m12, m13-m.m13,
                     m21-m.m21, m22-m.m22, m23-m.m23,
                     m31-m.m31, m32-m.m32, m33-m.m33));
}
const Matrix3x3 Matrix3x3::operator*(const Matrix3x3 &m) const
{
    return(Matrix3x3(m.m11 * m11 + m.m21 * m12 + m.m31 * m13,
                     m.m12 * m11 + m.m22 * m12 + m.m32 * m13,
                     m.m13 * m11 + m.m23 * m12 + m.m33 * m13,
                     m.m11 * m21 + m.m21 * m22 + m.m31 * m23,
                     m.m12 * m21 + m.m22 * m22 + m.m32 * m23,
                     m.m13 * m21 + m.m23 * m22 + m.m33 * m23,
                     m.m11 * m31 + m.m21 * m32 + m.m31 * m33,
                     m.m12 * m31 + m.m22 * m32 + m.m32 * m33,
                     m.m13 * m31 + m.m23 * m32 + m.m33 * m33));
}
const Matrix3x3 Matrix3x3::operator/(const Matrix3x3 &m) const
{
    return(*this * m.Invert());
}
const Matrix3x3 Matrix3x3::operator*(float f) const
{
    return(Matrix3x3(m11*f, m12*f, m13*f,
                     m21*f, m22*f, m23*f,
                     m31*f, m32*f, m33*f));
}

const Matrix3x3 Matrix3x3::operator/(float f) const
{
    const float fInv= 1.0 / f;
    return(Matrix3x3(m11*fInv, m12*fInv, m13*fInv,
                     m21*fInv, m22*fInv, m23*fInv,
                     m31*fInv, m32*fInv, m33*fInv));
}

////////////////////////////////////////////////////////////
//Identit&#65533;Matrix3x3, Transponieren
const Matrix3x3 & Matrix3x3::Identity()
{
    static const Matrix3x3 mIdentity(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0);
    return(mIdentity);
}
const Matrix3x3 Matrix3x3::Transpose() const
{
    return(Matrix3x3(m11, m21, m31,
                     m12, m22, m32,
                     m13, m23, m33));
}

////////////////////////////////////////////////////////////
//Determinante, Invertieren
//Determinante
float Matrix3x3::Determinate() const
{
    return(m11 * (m22*m33 - m23*m32) -
           m12 * (m21*m33 - m23*m31) +
           m13 * (m21*m32 - m22*m31));
}
//Invertieren
const Matrix3x3 Matrix3x3::Invert() const
{
    float fInvDet= Determinate();
    if(fInvDet == 0.0)
        return(Identity());
    fInvDet= 1.0 / fInvDet;

    Matrix3x3 mResult;
    mResult.m11 =  fInvDet * (m22*m33 - m23*m32);
    mResult.m12 = -fInvDet * (m12*m33 - m13*m32);
    mResult.m13 =  fInvDet * (m12*m23 - m13*m22);
    mResult.m21 = -fInvDet * (m21*m33 - m23*m31);
    mResult.m22 =  fInvDet * (m11*m33 - m13*m31);
    mResult.m23 = -fInvDet * (m11*m23 - m13*m21);
    mResult.m31 =  fInvDet * (m21*m32 - m22*m31);
    mResult.m32 = -fInvDet * (m11*m32 - m12*m31);
    mResult.m33 =  fInvDet * (m11*m22 - m12*m21);

    return(mResult);
}

////////////////////////////////////////////////////////////
//Skalierung, Achsen-Matrix
//Skalierung
const Matrix3x3 Matrix3x3::Scale(const Vector3 &v)
{
    return(Matrix3x3(v.x, 0.0, 0.0,
                     0.0, v.y, 0.0,
                     0.0, 0.0, v.z));
}
//AchsenMatrix3x3
const Matrix3x3 Matrix3x3::Axes(const Vector3 &vX, const Vector3 &vY, const Vector3 &vZ)
{
    return(Matrix3x3(vX.x, vX.y, vX.z,
                     vY.x, vY.y, vY.z,
                     vZ.x, vZ.y, vZ.z));
}
////////////////////////////////////////////////////////////
//Rotationsmatrizen
//Rotation um X-Achse
const Matrix3x3 Matrix3x3::RotateX(float fAngle)
{
    Matrix3x3 mResult;

    mResult.m11 = 1.0; mResult.m12 = 0.0; mResult.m13 = 0.0;
    mResult.m21 = 0.0;
    mResult.m31 = 0.0;

    mResult.m22 = mResult.m33 = cos(fAngle);
    mResult.m23 = sin(fAngle);
    mResult.m32 = -mResult.m23;

    return(mResult);
}
//Rotation um Y-Achse
const Matrix3x3 Matrix3x3::RotateY(float fAngle)
{
    Matrix3x3 mResult;

                        mResult.m12 = 0.0F;
    mResult.m21 = 0.0F; mResult.m22 = 1.0F; mResult.m23 = 0.0F;
                        mResult.m32 = 0.0F;

    mResult.m11 = mResult.m33 = cos(fAngle);
    mResult.m31 = sin(fAngle);
    mResult.m13 = -mResult.m31;

    return(mResult);
}
//Rotation um Z-Achse
const Matrix3x3 Matrix3x3::RotateZ(float fAngle)
{
    Matrix3x3 mResult;
    
                                            mResult.m13 = 0.0F;
                                            mResult.m23 = 0.0F;
    mResult.m31 = 0.0F; mResult.m32 = 0.0F; mResult.m33 = 1.0F;

    mResult.m11 = mResult.m22 = cos(fAngle);
    mResult.m12 = sin(fAngle);
    mResult.m21 = -mResult.m12;

    return(mResult);
}
//Rotation um alle Achsen zugleich
const Matrix3x3 Matrix3x3::RotateXYZ(const Vector3 &v)
{
    return(RotateX(v.x) * RotateY(v.y) * RotateZ(v.z));
}
const Matrix3x3 Matrix3x3::RotateXZY(const Vector3 &v)
{
    return(RotateX(v.x) * RotateZ(v.z) * RotateY(v.y));
}
const Matrix3x3 Matrix3x3::RotateYXZ(const Vector3 &v)
{
    return(RotateY(v.y) * RotateX(v.x) * RotateZ(v.z));
}
const Matrix3x3 Matrix3x3::RotateYZX(const Vector3 &v)
{
    return(RotateY(v.y) * RotateZ(v.z) * RotateX(v.x));
}
const Matrix3x3 Matrix3x3::RotateZXY(const Vector3 &v)
{
    return(RotateZ(v.z) * RotateX(v.x) * RotateY(v.y));
}
const Matrix3x3 Matrix3x3::RotateZYX(const Vector3 &v)
{
    return(RotateZ(v.z) * RotateY(v.y) * RotateX(v.x));
}

//Rotation um beliebige Achse
const Matrix3x3 Matrix3x3::RotateAxis(const Vector3 &v, float fAngle)
{
    const float fSin        = sin(-fAngle);
    const float fCos        = cos(-fAngle);
    const float fOneMinusCos= 1.0-fCos;

    const Vector3 vAxis(v.Normalize());

    return(Matrix3x3((vAxis.x*vAxis.x) * fOneMinusCos + fCos,
                     (vAxis.x*vAxis.y) * fOneMinusCos - (vAxis.z*fSin),
                     (vAxis.x*vAxis.z) * fOneMinusCos + (vAxis.y*fSin),
                     (vAxis.y*vAxis.x) * fOneMinusCos + (vAxis.z*fSin),
                     (vAxis.y*vAxis.y) * fOneMinusCos + fCos,
                     (vAxis.y*vAxis.z) * fOneMinusCos - (vAxis.x*fSin),
                     (vAxis.z*vAxis.x) * fOneMinusCos - (vAxis.y*fSin),
                     (vAxis.z*vAxis.y) * fOneMinusCos + (vAxis.x*fSin),
                     (vAxis.z*vAxis.z) * fOneMinusCos + fCos));
}
//Rotation aus Quaternion
const Matrix3x3 Matrix3x3::RotateQuaternion(const Quaternion &q)
{
    return(Matrix3x3(1.0F - 2.0F*q.v.y*q.v.y - 2.0F*q.v.z*q.v.z,        2.0F*q.v.x*q.v.y + 2.0F*q.  w*q.v.z,        2.0F*q.v.x*q.v.z - 2.0F*q.  w*q.v.y,
                            2.0F*q.v.x*q.v.y - 2.0F*q.  w*q.v.z, 1.0F - 2.0F*q.v.x*q.v.x - 2.0F*q.v.z*q.v.z,        2.0F*q.v.y*q.v.z + 2.0F*q.  w*q.v.x,
                            2.0F*q.v.x*q.v.z + 2.0F*q.  w*q.v.y,        2.0F*q.v.y*q.v.z - 2.0F*q.  w*q.v.x, 1.0F - 2.0F*q.v.x*q.v.x - 2.0F*q.v.y*q.v.y ));
}


//Rotationsmatix in Euler-Winkel umwandeln
const Vector3 Matrix3x3::CalcEulerAngles() const
{
    return(Vector3(-atan2(m32, m22),
                    atan2(m31, m33),
                    atan2(m12, m22)));
}

//Vector3 transformieren
const Vector3 Matrix3x3::Transform(const Vector3 &v) const
{
    Vector3 vResult(v.x*m11 + v.y*m21 + v.z*m31,
                    v.x*m12 + v.y*m22 + v.z*m32,
                    v.x*m13 + v.y*m23 + v.z*m33);
    
    return(vResult);
}

////////////////////////////////////////////////////////////
//Arithmetik, die nicht definiert werden kann
const Matrix3x3 operator - (const Matrix3x3 &m)
{    
    return( Matrix3x3(-m.m11, -m.m12, -m.m13,
                      -m.m21, -m.m22, -m.m23,
                      -m.m31, -m.m32, -m.m33) );
}
const Matrix3x3 operator * (float f, const Matrix3x3 &m)
{    
    return( Matrix3x3(f*m.m11, f*m.m12, f*m.m13,
                      f*m.m21, f*m.m22, f*m.m23,
                      f*m.m31, f*m.m32, f*m.m33) );
}
const Matrix3x3 operator / (float f, const Matrix3x3 &m)
{    
    const float fInv= 1.0 / f;
    return( Matrix3x3(fInv*m.m11, fInv*m.m12, fInv*m.m13,
                      fInv*m.m21, fInv*m.m22, fInv*m.m23,
                      fInv*m.m31, fInv*m.m32, fInv*m.m33) );
}

