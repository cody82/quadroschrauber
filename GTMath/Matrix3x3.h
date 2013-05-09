#pragma once

class Matrix3x3
{
public:
    //Daten
    union
    {
        struct
        {
            float m11, m12, m13,
                  m21, m22, m23,
                  m31, m32, m33;
        };    
        float aaf[3][3];
        float af[9];
    };

    //Konstruktoren
    Matrix3x3();
    Matrix3x3(float m11, float m12, float m13,
              float m21, float m22, float m23,
              float m31, float m32, float m33);
    Matrix3x3(const Matrix3x3 & m);


    //Casting
    operator float* ();

    //operator() f&#65533; komfortableren Zugriff
    float operator()(int iRow, int iColumn) const;
    float & operator()(int iRow, int iColumn);


    //Zuweisungen
    Matrix3x3 & operator  = (const Matrix3x3 &m);
    Matrix3x3 & operator += (const Matrix3x3 &m);
    Matrix3x3 & operator -= (const Matrix3x3 &m);
    Matrix3x3 & operator *= (const Matrix3x3 &m);
    Matrix3x3 & operator /= (const Matrix3x3 &m);
    Matrix3x3 & operator *= (float f);
    Matrix3x3 & operator /= (float f);
    

    //Arithmetik
    const Matrix3x3 operator +(const Matrix3x3 &m) const;
    const Matrix3x3 operator - (const Matrix3x3 &m) const;
    const Matrix3x3 operator * (const Matrix3x3 &m) const;
    const Matrix3x3 operator / (const Matrix3x3 &m) const;
    const Matrix3x3 operator * (float f) const;
    const Matrix3x3 operator / (float f) const;


    //Identitaets-Matrix, Transponieren
    static const Matrix3x3 & Identity();
    const Matrix3x3 Transpose() const;
    
    //Determinante, Invertieren
    float Determinate() const;
    const Matrix3x3 Invert() const;

    //Skalierung, Achsen-Matrix
    static const Matrix3x3 Scale(const Vector3 &v);
    static const Matrix3x3 Axes(const Vector3 &vX, const Vector3 &vY, const Vector3 &vZ);

    //Rotation
    //Eulerwinkel
    static const Matrix3x3 RotateX(float fAngle);
    static const Matrix3x3 RotateY(float fAngle);
    static const Matrix3x3 RotateZ(float fAngle);
    
    static const Matrix3x3 RotateXYZ(const Vector3 &v);
    static const Matrix3x3 RotateXZY(const Vector3 &v);
    static const Matrix3x3 RotateYXZ(const Vector3 &v);
    static const Matrix3x3 RotateYZX(const Vector3 &v);
    static const Matrix3x3 RotateZXY(const Vector3 &v);
    static const Matrix3x3 RotateZYX(const Vector3 &v);
    
    //Um Achse
    static const Matrix3x3 RotateAxis(const Vector3 &v, float fAngle);
    //Rotation aus Quaternion
    const Matrix3x3 RotateQuaternion(const Quaternion &q);
    
    //Rotationsmatix in Euler-Winkel umwandeln
    const Vector3 CalcEulerAngles() const;
    
    //Transformation von Vector3: Da 3x3 nur Rotation/Skalierung moeglich
    const Vector3 Transform(const Vector3 &v) const;
};

////////////////////////////////////////////////////////////
//Arithmetik, die nicht inline definiert werden kann
const Matrix3x3 operator - (const Matrix3x3 &m);
const Matrix3x3 operator * (float f, const Matrix3x3 &m);
const Matrix3x3 operator / (float f, const Matrix3x3 &m);
