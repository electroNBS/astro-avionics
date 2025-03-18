#include <Arduino.h>

void setupGPS();

struct GPSData
{
    double lat,lng;
    float alt,vel,dir;
};

GPSData getData();

String packGPSDATA(GPSData data);