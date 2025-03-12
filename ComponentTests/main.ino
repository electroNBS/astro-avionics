#include <SoftwareSerial.h>

bool rpiStatus;
SoftwareSerial serial(6,7);

bool piStatusCheck(SoftwareSerial &serial){
    unsigned long start = millis();

    while(millis() - start < 30000){  // Keep checking for 30 seconds
        if(serial.available()){

            String msg = serial.readStringUntil('\n');  // Read serial
            msg.trim();

            if (msg == "PING") {
                serial.println("PONG");  // Send response
                return true;
            }
        }
    }

    return false;
}

void setup(){
    serial.begin(9600);
    rpiStatus = piStatusCheck(serial);
}

void loop(){
    // add normal and failure code
}