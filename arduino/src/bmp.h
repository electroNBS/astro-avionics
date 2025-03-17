#ifndef BMP_H
#define BMP_H

#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include "bmp3.h"

class BMPSensor
{
public:
    BMPSensor(int sdaPin, int sclPin);
    bool begin();
    float getAltitude();

private:
    Adafruit_BMP3XX bmp;
    int sdaPin, sclPin;
};

#endif