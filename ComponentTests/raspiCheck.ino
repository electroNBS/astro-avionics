#include <SoftwareSerial.h>

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