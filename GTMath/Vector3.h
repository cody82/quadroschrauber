#pragma once

class Vector3
{
public:
    //Daten
    union
    {
        struct
        {
            float x;
            float y;
            float z;
        };
        float af[3];
    };
    
    //Konstruktor
    Vector3();
    Vector3(float all);
    Vector3(float _x, float _y, float _z);
    Vector3(const Vector3 & v);
    
    //Casting
    operator float* ();
    
    //Arithmetik komponentenweise
    const Vector3 operator +(const Vector3 & v) const;
    const Vector3 operator -(const Vector3 & v) const;
    const Vector3 operator *(const Vector3 & v) const;
    const Vector3 operator /(const Vector3 & v) const;
    const Vector3 operator *(float f) const;
    const Vector3 operator /(float f) const;
    
    //Zuweisungsoperatoren komponentenweise
    Vector3 & operator =(const Vector3 & v);
    Vector3 & operator +=(const Vector3 & v);
    Vector3 & operator -=(const Vector3 & v);
    Vector3 & operator *=(const Vector3 & v);
    Vector3 & operator /=(const Vector3 & v);
    Vector3 & operator *=(float f);
    Vector3 & operator /=(float f);
    
    //Vergleich
    bool operator == (const Vector3 & v) const;
    bool operator != (const Vector3 & v) const;
        
    //Min, Max
    const Vector3 Min(const Vector3 & v) const;
    const Vector3 Max(const Vector3 & v) const;
    
    //Skalar- und Kreuzprodukt, Winkel
    float DotP(const Vector3 & v) const;
    const Vector3 CrossP(const Vector3 & v) const;
    float Angle(const Vector3 &v) const;
    
    //Laenge, Quadratlaenge, Nomralisieren
    float Length() const;
    float LengthSq() const;
    const Vector3 Normalize() const;
};

//Operatoren, die nicht innerhalt der Klasse definiert werden koennen
const Vector3 operator - (const Vector3 & v);
const Vector3 operator * (float f, const Vector3 & v);
const Vector3 operator / (float f, const Vector3 & v);
