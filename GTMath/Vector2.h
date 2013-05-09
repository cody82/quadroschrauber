#pragma once

class Vector2
{
public:
    //Daten
    union
    {
        struct
        {
            float x;
            float y;
        };
        float af[2];
    };
    
    //Konstruktor
    Vector2();
    Vector2(float all);
    Vector2(float _x, float _y);
    Vector2(const Vector2 & v);
    
    //Casting
    operator float* ();
    
    //Arithmetik komponentenweise
    const Vector2 operator +(const Vector2 & v) const;
    const Vector2 operator -(const Vector2 & v) const;
    const Vector2 operator *(const Vector2 & v) const;
    const Vector2 operator /(const Vector2 & v) const;
    const Vector2 operator *(float f) const;
    const Vector2 operator /(float f) const;
    
    //Zuweisungsoperatoren komponentenweise
    Vector2 & operator =(const Vector2 & v);
    Vector2 & operator +=(const Vector2 & v);
    Vector2 & operator -=(const Vector2 & v);
    Vector2 & operator *=(const Vector2 & v);
    Vector2 & operator /=(const Vector2 & v);
    Vector2 & operator *=(float f);
    Vector2 & operator /=(float f);
    
    //Vergleich
    bool operator == (const Vector2 & v) const;
    bool operator != (const Vector2 & v) const;
        
    //Min, Max
    const Vector2 Min(const Vector2 & v) const;
    const Vector2 Max(const Vector2 & v) const;
    
    //Skalarprodukt, Winkel
    float DotP(const Vector2 & v) const;
    float Angle(const Vector2 &v) const;
    
    //Laenge, Quadratlaenge, Nomralisieren
    float Length() const;
    float LengthSq() const;
    const Vector2 Normalize() const;
};

//Operatoren, die nicht innerhalt der Klasse definiert werden koennen
const Vector2 operator - (const Vector2 & v);
const Vector2 operator * (float f, const Vector2 & v);
const Vector2 operator / (float f, const Vector2 & v);
