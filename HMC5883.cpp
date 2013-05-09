#include "mbed.h"
#include "HMC5883.h"

#define I2CADR_W(ADR)           (ADR<<1&0xFE)
#define I2CADR_R(ADR)           (ADR<<1|0x01)

//Initialisieren
HMC5883::HMC5883(I2C & I2CBus_, Timer & GlobalTime_) :
    I2CBus(I2CBus_),
    GlobalTime(GlobalTime_)
 
{}

bool HMC5883::Init()
{
    //Nullsetzen
    MeasurementError= 0;
    AutoCalibration= 0;
    #pragma unroll
    for(int i= 0; i < 3; i++)
    {
        RawMin[i]= -400;
        RawMax[i]= 400;
        Offset[i]= 0;
        Scale[i]= 1.0;
        RawMag[i]= 0;
        Mag[i]= 0;
    }
    
       
    //HMC5883 initialisieren
    char tx[4];
    
    //1 Sample pro Messung
    //75Hz Output Rate
    //Range: +- 1.3 Gauss
    //Continuous-Measurement Mode
    tx[0]= 0x00;
    tx[1]= 0x78;    //Configuration Register A
    tx[2]= 0x20;    //Configuration Register B
    tx[3]= 0x00;    //Mode Register
    if(I2CBus.write(I2CADR_W(HMC5883_ADRESS), tx, 4) != 0)
        return false;
    
    return Update();
}

//Rohdaten lesen
bool HMC5883::ReadRawData()
{
    //Drehrate aller Axen abholen    
    char tx[1];
    char rx[6];
     
    tx[0]= 0x03;
    if(I2CBus.write(I2CADR_W(HMC5883_ADRESS), tx, 1) != 0)
        return false;
    
    if(I2CBus.read (I2CADR_R(HMC5883_ADRESS), rx, 6) != 0)
        return false;
    
    //Aus den einzelnen Bytes den 16-Bit-Wert zusammenbauen
    short r[3];
    r[0]= rx[0]<<8|rx[1];
    r[1]= rx[4]<<8|rx[5];
    r[2]= rx[2]<<8|rx[3];
    
    //Grober Messfehler?
    if(r[0] == -4096
    || r[1] == -4096
    || r[2] == -4096)
        MeasurementError= 1;
    else
    {
        MeasurementError= 0;
        RawMag[0]= r[0];
        RawMag[1]= r[1];
        RawMag[2]= r[2];
    }
    
    return true;
}


//Update-Methode
bool HMC5883::Update()
{
    //Rohdaten lesen
    if(!ReadRawData())
        return false;
    
    if(AutoCalibration)
    {
        #pragma unroll
        for(int i= 0; i < 3; i++)
        {
            //Neuer Min Max Wert?
            RawMin[i]= RawMin[i] < RawMag[i] ? RawMin[i] : RawMag[i];
            RawMax[i]= RawMax[i] > RawMag[i] ? RawMax[i] : RawMag[i];
            
            //Scale und Offset aus gesammelten Min Max Werten berechnen
            //Die neue Untere und obere Grenze bilden -1 und +1
            Scale[i]= 2.0 / (float)(RawMax[i]-RawMin[i]);
            Offset[i]= 1.0 - (float)(RawMax[i]) * Scale[i];
        }
    }
    
    //Feldstaerke berechnen
    Mag[0]= Scale[0] * (float)(RawMag[0]) + Offset[0];
    Mag[1]= Scale[1] * (float)(RawMag[1]) + Offset[1];
    Mag[2]= Scale[2] * (float)(RawMag[2]) + Offset[2];
    
    return true;
}


//Kalibrieren
void HMC5883::Calibrate(const short * pRawMin, const short * pRawMax)
{
    #pragma unroll
    for(int i= 0; i < 3; i++)
    {        
        RawMin[i]= pRawMin[i];
        RawMax[i]= pRawMax[i];
    }
    char AutoCalibrationBak= AutoCalibration;
    AutoCalibration= 1;
    Update();
    AutoCalibration= AutoCalibrationBak;
}

void HMC5883::Calibrate(int s)
{
    char AutoCalibrationBak= AutoCalibration;
    AutoCalibration= 1;
    
    //Ende der Kalibrierung in ms Millisekunden berechnen
    int CalibEnd= GlobalTime.read_ms() + s*1000;
    
    while(GlobalTime.read_ms() < CalibEnd)
    {
        //Update erledigt alles
        Update();
    }
    
    AutoCalibration= AutoCalibrationBak;
}