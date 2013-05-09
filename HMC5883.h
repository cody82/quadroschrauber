#pragma once

//I2C Adresse
#define HMC5883_ADRESS 0x1E


class HMC5883
{
private:
    I2C & I2CBus;
    Timer & GlobalTime;
        
public:
    //Calibration
    char AutoCalibration;       //automatische Kalibrierung der Sensordaten
    short RawMin[3], RawMax[3]; //gespeicherte Daten fuer die Auto-Kalibrierung
    float Scale[3];             //jede Achse einzeln skalieren
    float Offset[3];            //zum Schluss noch das Offset dazu
    
    //Feldstaerke auf allen drei Achsen
    short RawMag[3];            //Rohdaten
    float Mag[3];               //kalibrierte Rohdaten (nicht normalisiert)
    
    //Bei zu hoher Feldstaerke wird -4096 gelesen
    //Wenn dies eine Achse betreffen sollte, ist diese Variable 1
    short MeasurementError;
    
    
    //Initialisieren
    HMC5883(I2C & I2CBus_, Timer & GlobalTime_);
    bool Init();

private:    
    //Rohdaten lesen
    bool ReadRawData();
    
public:
    //Update-Methode
    bool Update();
    
    //Kalibrieren
    //Fertige Daten benutzen. Scale[0] muss 1.0 sein!
    void Calibrate(const short * pRawMin, const short * pRawMax);
    
    //Selbst auswerten, dauert s Sekunden lang
    void Calibrate(int s);
};
