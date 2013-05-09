#pragma once

//Forward Declaration
class Matrix3x3;

class Quaternion
{
public:
    ///////////////////////////////////////////////////////////////////
    //Variablen
    float w;
    Vector3 v;
                
    
    ///////////////////////////////////////////////////////////////////
    //Konstruktoren
    Quaternion();
    Quaternion(const Quaternion & q);
    Quaternion(float _w, const Vector3 & _v);
    Quaternion(float _w, float _x, float _y, float _z);

    //////////////////////////////////////////////////////////////////
    //Casting
    operator float* ();

    ///////////////////////////////////////////////////////////////////
    //Zuweisungen
    Quaternion& operator =(const Quaternion& q);
    Quaternion& operator+=(const Quaternion& q);
    Quaternion& operator-=(const Quaternion& q);
    Quaternion& operator*=(const Quaternion& q);
    Quaternion& operator/=(const Quaternion& q);
    Quaternion& operator*=(float f);
    Quaternion& operator/=(float f);

    ////////////////////////////////////////////////////////////
    //Arithmetik
    const Quaternion operator+(const Quaternion& q) const;
    const Quaternion operator-(const Quaternion& q) const;
    const Quaternion operator*(const Quaternion& q) const;
    const Quaternion operator/(const Quaternion& q) const;
    const Quaternion operator*(float f) const;
    const Quaternion operator/(float f) const;
    const Quaternion operator-() const;
    
    ////////////////////////////////////////////////////////////
    //Identit&#65533;quaternion
    static const Quaternion MulIdentity();
    static const Quaternion AddIdentity();

    ////////////////////////////////////////////////////////////
    //Betrag (=Laenge), Normalisieren
    float Length() const;
    float LengthSq() const;
    const Quaternion Normalize() const;
    
    ////////////////////////////////////////////////////////////
    //Konjugieren, Invertieren
    const Quaternion Conjugate() const;
    const Quaternion Invert() const;
    
    ////////////////////////////////////////////////////////////
    //Punktprodukt berechnen
    float DotP(const Quaternion& q) const;

    ////////////////////////////////////////////////////////////
    //Rotation von Vektoren
    const Vector3 RotateVector(const Vector3 &v) const;

    ////////////////////////////////////////////////////////////
    //Konstruieren
    //aus: Richtungsvektor und Winkel (Radiant!)
    static const Quaternion CreateFromAxisAndAngle(const Vector3& vAxis, float fAngle);
    //aus: Euler-Rotation (=3 Winkel als Vektor3)
    static const Quaternion CreateFromEuler(Vector3 vEulerAngles);
    //aus: Matrix3x3
    static const Quaternion CreateFromMatrix(const Matrix3x3 & m);
    
    
    //Quaternion in Euler-Winkel umwandeln
    const Vector3 CalcEulerAngles() const;

    ////////////////////////////////////////////////////////////
    //Lineare Interpolation
    const Quaternion Lerp(Quaternion q, float f) const;
    //Sphaerische lineare Interpolation
    const Quaternion Slerp(Quaternion q, float f) const;
};
