#include "E32.h"

E32Module::E32Module() : e32Serial(e32TX, e32RX) {}

void E32Module::begin(long baudRate) {
    Serial.begin(115200);
    e32Serial.begin(baudRate);

    // Setup M0 and M1 for Normal Transmission Mode
    pinMode(e32M0, OUTPUT);
    pinMode(e32M1, OUTPUT);
    digitalWrite(e32M0, LOW);
    digitalWrite(e32M1, LOW);
}

void E32Module::sendMessage(byte message[], int length) {
    e32Serial.write(message, length);
}
