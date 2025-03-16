#include <Arduino.h>
#include "E32.h"
#include "testEjectionCharges.h"
#include "./bmp.h"

#define BMP_SDA A4
#define BMP_SCL A5
BMPSensor bmpSensor(BMP_SDA, BMP_SCL);

// define pins
#define BuzzerPin A1

// define Constants
#define BOOT_TIME 15000    // time to wait for both systems to boot up
#define CONNECT_TIME 15000 // time to wait for the connection to be established

// Define serial ports
HardwareSerial SerialE32(1);   // Use UART1 (A3 RX, A2 TX)
HardwareSerial SerialRaspi(2); // Use UART2 (A6 RX, A7 TX)
E32Module e32(SerialE32);


// define the status register bits
#define RPI_h    (1 << 0) // is the raspberry pi alive
#define GPS_h    (1 << 1) // is the gps alive
#define LORA_h   (1 << 2) // is the lora alive
#define BMP_h    (1 << 3) // is the bmp alive
#define IMU_h    (1 << 4) //is the imu alive
#define ECd_h    (1 << 5) //ejecion charge drogue
#define ECm_h    (1 << 6) //ejecion charge main
#define ECb_h    (1 << 7) //ejecion charge backup
// binary status register
// 0b00000000
int status = 0b00000000;


enum State {
  BOOT,
  CONN,
  CALIB,
  IDLE,
  FLIGHT,
  DROUGE,
  PARACHUTE,
  RECOVERY,
  EMMERGENCY
};

State state = BOOT;



void setup() {
  Serial.begin(115200);

  // initialize the buzzer
  pinMode(BuzzerPin, OUTPUT);
  digitalWrite(BuzzerPin, LOW);
}

void loop() {
  while(state == BOOT) {
    // beep buzzer for 200ms every second for next 15 seconds
    for(int i = 0; i < 15; i++) {
      digitalWrite(BuzzerPin, HIGH);
      delay(200);
      digitalWrite(BuzzerPin, LOW);
      delay(800);
    }
    state = CONN;
  }
  while (state == CONN){
    // talk to the raspberry pi
    // if the raspberry pi is alive
    // set the RPI_h bit in the status register
    // else
    // clear the RPI_h bit in the status register
  }
}