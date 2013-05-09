#pragma once

#include "mbed.h"

#define NMEA_MINUTE_FRACTS 4
#define NMEA_ALTITUDE_FRACTS 2

#define NMEA_RMC_FLAGS_STATUS_OK 0
#define NMEA_RMC_FLAGS_LAT_NORTH 1
#define NMEA_RMC_FLAGS_LON_EAST 2

struct coord {
    /* degrees, 0-180 or 0-90 */
    uint8_t deg;
    /* minutes, 0-60 */
    uint8_t min;
    /* fractions of minutes saved as BCD */
    uint8_t frac[(NMEA_MINUTE_FRACTS+1)/2];
};

struct altitude_t {
    int16_t m;
    uint8_t frac[(NMEA_ALTITUDE_FRACTS+1)/2];
};

struct gps_clock_t {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

struct date_t {
    uint8_t day;
    uint8_t month;
    uint8_t year;
};

struct nmea_data_t {
    uint8_t flags;
    struct date_t date;
    struct gps_clock_t clock;
    struct coord lat;
    struct coord lon;
    struct altitude_t alt;
    uint8_t quality;
    uint8_t sats;
};

struct optical_data_t {
    int8_t dx;
    int8_t dy;
};

struct sonar_data_t {
    int16_t distance;
};


struct nav_data_t
{
    struct nmea_data_t gps;
    struct sonar_data_t sonar;
    struct optical_data_t optical;
};

#define TINY_GPS_ADRESS (0x11)

class TinyGPS
{
public:
    TinyGPS(I2C &_i2c);
    
    float GetLatitude();
    float GetLongitude();
    float GetAltitude();
    
    bool Update();
    bool ok;
protected:
    I2C &i2c;
    nav_data_t nav_data;
};
