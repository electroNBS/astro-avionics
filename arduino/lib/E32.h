#ifndef E32_H
#define E32_H

#include <Arduino.h>
#include <SoftwareSerial.h>

#define e32RX A3
#define e32TX A2
#define e32M0 D11
#define e32M1 D12

class E32Module
{
public:
    E32Module(); // Constructor
    void begin(long baudRate = 9600);
    void sendMessage(byte message[], int length);

private:
    SoftwareSerial e32Serial;
};

#endif