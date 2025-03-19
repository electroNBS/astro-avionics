#include <Arduino.h>
#include "E32.h"
#include "triggerEjectionCharges.h"
#include "checkEjectionCharges.h"
#include "bmp.h"
#include "gps.h"
#include "imu.h"
#include "buzzer.h"

#define BMP_SDA A4
#define BMP_SCL A5
BMPSensor bmpSensor(BMP_SDA, BMP_SCL);

// define pins
#define BuzzerPin A1
#define RaspiTX A7
#define RaspiRX A6
#define E32RX A3
#define E32TX A2
#define GPSRX D1
#define GPSTX D0

// define Constants
#define BOOT_TIME 15000    // time to wait for both systems to boot up
#define CONNECT_TIME 15000 // time to wait for the connection to be established
#define SAFE_PARACHUTE_VEL 2 // safe velocity below which we can deoply parachute

// Define serial ports
//HardwareSerial SerialE32(1);   // Use UART1 (A3 RX, A2 TX)
#define SerialE32 Serial1
#define SerialRaspi Serial2
//HardwareSerial SerialRaspi(2); // Use UART2 (A6 RX, A7 TX)
//HardwareSerial SerialGPS(3); //Use UART3 (D0 TX, D1 RX)
#define SerialGPS Serial0
E32Module e32(SerialE32);


// define the status register bits
#define RPI_h    (1 << 0) // is the raspberry pi alive
#define GPS_h    (1 << 1) // is the gps on
#define LORA_h   (1 << 2) // is the lora alive
#define BMP_h    (1 << 3) // is the bmp alive
#define IMU_h    (1 << 4) //is the imu alive
// ejection charge continuity , 0 means no continuity , 1 means continuity
#define ECd_h    (1 << 5) //ejecion charge continuity drogue 
#define ECm_h    (1 << 6) //ejecion charge continuity main
#define ECb_h    (1 << 7) //ejecion charge continuity backup
// ejection charge status , 0 means not fired , 1 means fired
#define ECrd_h    (1 << 8) //ejecion charge status drogue
#define ECrm_h    (1 << 9) //ejecion charge status main
#define ECrb_h    (1 << 10) //ejecion charge status backup
// next 3 (11,12,13) bits show the state of the rocket, 000 means boot, 001 means connection, 010 means calibration, 011 means idle, 100 means flight, 101 means drogue, 110 means parachute, 111 means recovery
#define S10_h    (1 << 11) // state bit 0
#define S11h    (1 << 12) // state bit 1
#define S12_h    (1 << 13) // state bit 2
// is tip over detected is 14th bit
#define TIP_OVER (1 << 14)
// binary status register
// 0b0000000000000000 
// bit 0: raspberry pi status
// bit 1: gps status
// bit 2: lora status
// bit 3: bmp status
// bit 4: imu status
// bit 5 ,6 ,7: ejection charge continuity (drogue, main, backup) 
// bit 8,9,10: ejection charge status (drogue, main, backup)
// bit 11,12,13: state of the rocket
// bit 14: TIP_OVER
uint16_t status = 0b0000000000000000;
// define ejection charge status register


enum State {
  BOOT,
  CONN,
  CALIB,
  IDLE,
  FLIGHT,
  DROUGE,
  PARACHUTE,
  RECOVERY,
};

void setState(State s){
  // update the status register , set the state bits
// next 3 (11,12,13) bits show the state of the rocket, 000 means boot, 001 means connection, 010 means calibration, 011 means idle, 100 means flight, 101 means drogue, 110 means parachute, 111 means recovery
  switch (s)
  {
  case BOOT:
    status &= ~(S10_h | S11h | S12_h); //000
    break;
  case CONN:
    status &= ~(S10_h | S11h); //001
    status |= S12_h;
    break;
  case CALIB:
    status &= ~(S10_h | S12_h); //010
    status |= S11h;
    break;
  case IDLE:
    status &= ~(S10_h); // 011
    status |= S11h | S12_h;
    break;
  case FLIGHT:
    status &= ~(S11h | S12_h); // 100
    status |= S10_h;
    break;
  case DROUGE:
    status &= ~(S10_h | S11h); // 101
    status |= S12_h;
    break;
  case PARACHUTE:
    status &= ~(S10_h); // 110
    status |= S11h | S12_h;
    break;
  case RECOVERY:
    status |= S10_h | S11h | S12_h; // 111
    break;
  default:
    break;
  }


}

State state = BOOT;

// state variables
float groundAltitude = 0;
unsigned long timeof_tip_over = 0;

// flight data struct
struct Data {
  unsigned long time;
  float bmpAltitude;
  float imuAltitude;
  float pressure;
  float accel_x;
  float accel_y;
  float accel_z;
  float vel_bmp;
  float vel_imu;
  uint16_t statusReg;
};





String packDATA(Data data){
  String binaryStatus = "";
  for (int i = 15; i >= 0; i--) {
    binaryStatus += String((data.statusReg >> i) & 1);
  }
  return "DATA:" + String(data.bmpAltitude) + "," + String(data.imuAltitude) + "," + String(data.pressure) + ","  + String(data.accel_x) + "," + String(data.accel_y) + "," + String(data.accel_z) + "," + String(data.vel_bmp) + "," + String(data.vel_imu) + "," + binaryStatus;
}

// checks all ejection charge continuity and updates status
// checks only once 200 time it is called or force is true , resets counter when force is called
// takes 10*3 ms to execute
void checkAllEjectionChargeContinuity(bool force = false){
  static int count = 0;
  if (count<200&& !force){
    count ++ ;
    return;
  }else{
    count = 0;
  }
  if (checkMainEjectionCharges()){
    status |= ECm_h;
  }else {
    status &= !ECm_h;
  }
  if (checkBackupEjectionCharges()){
    status |= ECb_h;
  }
  else {
    status &= !ECb_h;

  }
  if (checkDrogueEjectionCharges()){
    status |= ECd_h;
  }
  else {
    status &= !ECd_h;

  }
}

/*
we keep the averaging the ax , ay , az with the readings we recive, and if there is more than 40% change, we return 1;
we ignore change in first 10 readings
*/
// int checkTipOver(float ax,float ay , float az){
//   // static float aax = 0, aay=0, aaz =0;
//   // static unsigned long count = 0;
//   // if (count>10){
//   //   // check if anything has changed more then 30% from average
//   //   float dx = abs((aax-ax)/aax) , dy  = abs((aay - ay)/aay) , dz = abs((aaz-az)/aaz);
//   //   if (dx > 0.3 || dy >0.3||dx >0.3)
//   //   {
//   //     return 1;
//   //   }
//   // }
//   // // updating averages
//   //  aax = (count == 0) ? ax : (aax * count + ax) / (count + 1);
//   //  aay = (count == 0) ? ay : (aay * count + ay) / (count + 1);
//   //  aaz = (count == 0) ? az : (aaz * count + az) / (count + 1);

//   //  count++; // increment count


// }



Data updateDataWithoutGPS(){
  Data data;
  data.time = millis();
  data.bmpAltitude = bmpSensor.getAltitude();
  data.pressure = bmpSensor.getPressure();
  data.imuAltitude = getHeightIMU();
  IMUReading reading_i = readIMU();
  data.accel_x = reading_i.accel_x;
  data.accel_y = reading_i.accel_y;
  data.accel_z = reading_i.accel_z;
  data.vel_imu = getVelocityIMU();
  data.vel_bmp = bmpSensor.getVelocity();
  checkAllEjectionChargeContinuity();
  data.statusReg = status;
  return data;
}

void setup() {
  Serial.begin(115200);

  // initialize the buzzer
  pinMode(BuzzerPin, OUTPUT);
  digitalWrite(BuzzerPin, LOW);
}

void loop() {
  while(state == BOOT) {
    // beep buzzer for 200ms every second till the end of the boot time
    for(int i = 0; i < BOOT_TIME; i+=1000) {
      digitalWrite(BuzzerPin, HIGH);
      delay(200);
      digitalWrite(BuzzerPin, LOW);
      delay(800);
    }
    setState (CONN);
  }
  while (state == CONN){
    // begin communication with the raspberry pi
    SerialRaspi.begin(115200, SERIAL_8N1, RaspiRX, RaspiTX); 

    // begin communication with the e32
    SerialE32.begin(115200, SERIAL_8N1, E32RX, E32TX);

    // connect to raspberry pi
    unsigned long start = millis();
    // this loop runs while the time is less than connect time , or  the raspberry pi and lora module are not connected
    while (millis() - start < CONNECT_TIME ||  (!(status & RPI_h) || !(status & LORA_h))){
      // send ping , and wait for pong
      SerialRaspi.println("PING");
      if(SerialRaspi.available()){
        String response = SerialRaspi.readStringUntil('\n');
        if(response == "PONG"){
          status |= RPI_h;
          break;
        }
      }

      // send ping to lora module
      e32.sendMessage("PING");
    }
    // beep buzzer long for 2 seconds
    digitalWrite(BuzzerPin, HIGH);
    delay(2000);
    digitalWrite(BuzzerPin, LOW);
    setState(CALIB);
  }
  while (state == CALIB){
    // initialize the sensors and calibrate them
    // setup the bmp sensor
    if(bmpSensor.begin()){
      status |= BMP_h;
    }
    // setup the imu
    if (setupIMU()){
      status |= IMU_h;
    }
    
    // play calibration start tone
    playCalibrationStartTone(); // takes 1 second
    for (int i = 0; i < 10; i++){
      groundAltitude += bmpSensor.getAltitude();
      delay(10);
    }
    groundAltitude /= 10;

    SerialRaspi.println("GROUND_ALTITUDE:" + String(groundAltitude));
    
    
    // IMU will take 500 samples to calibrate , each 5ms , total 2.5 seconds
    calibrateIMU(500); 

    // we also check ejection Charge Continutity
    checkAllEjectionChargeContinuity(true);
    // play a tone to indicate that the calibration is done
    playCalibrationStartTone(); // takes 1 second

    setState(IDLE);
  }
  while (state == IDLE) {
    // read the data from the sensors
    Data data = updateDataWithoutGPS();
    
    
    // pack data
    String packed_data = packDATA(data);
    // send the data to the raspberry pi
    SerialRaspi.println(packed_data);
    // send the data to Telemetry 
    e32.sendMessage(packed_data);
    if((BMP_h & status) && (bmpSensor.getAltitude()-groundAltitude>200) || (IMU_h & status) && (getHeightIMU()>200)){
        setState(FLIGHT);
    }
  }
  /*
  In Flight Mode we only take check bmp altitude and imu Height , IMU tipover
  and buffer that data , and send data
  */
  while(state == FLIGHT){
    // update Data 
    // read the data from the sensors
    Data data = updateDataWithoutGPS();
    
    // pack data
    String packed_data = packDATA(data);
    // send the data to the raspberry pi
    SerialRaspi.println(packed_data);
    // send the data to Telemetry 
    e32.sendMessage(packed_data);
    // TODO : if imu not working , do other thing to enter drouge mode
    // TODO : do other checks
    if(data.vel_bmp <1 || isRocketTippingOver() || data.vel_imu <1){
      status |= TIP_OVER;
      timeof_tip_over = millis();
      // send tip over message to raspi
      SerialRaspi.println("TIP OVER");
      e32.sendMessage("TIP OVER");
      SerialRaspi.println(timeof_tip_over);
      // update continutity status to make things ready for drouge 
      checkAllEjectionChargeContinuity(true);
      setState(DROUGE);
    }
  }
  while (state == DROUGE)
  {
    // wait for half second after tip over , and deploy drouge.
    if (millis()-timeof_tip_over > 500){
      triggerDrogueEjectionCharges();
      delay(100); // wait for 100ms and check if deployment was success
      int status_d = checkDrogueEjectionCharges();
      if (!status_d){
        // drouge deoplyment is confirmed
        status |= ECrd_h; // update ejection charge drouge deployment to 1
        status &= ~ECd_h; // update ejection charge drouge continuity to 0
        SerialRaspi.println("DROUGE Deployed");
        e32.sendMessage("DROUGE Deployed");
        setState(PARACHUTE);
      }
      else {
        // pray
      }
    }
    // return out of drouge state to parachute if drouge is not deployed in few seconds 
    if (millis()-timeof_tip_over > 5000){
      SerialRaspi.println("TIMEOUT of DROUGE , Entering parachute mode");
      e32.sendMessage("TIMEOUT of DROUGE , Entering parachute mode");
    }
   
  }
  while(state == PARACHUTE){
    // we send all data , turn on gps , and wait for height to reach below 500m , and deploy parachute , if it does not deploy we try backup
    Data data = updateDataWithoutGPS();
    if (data.bmpAltitude < 500 && data.vel_imu < SAFE_PARACHUTE_VEL){
      // we try to deoply parachute
      triggerMainEjectionCharges();
      delay(100); // wait for 100ms and check if deployment was success
      int status_d = checkMainEjectionCharges();
      if (!status_d){
        // main deoplyment is confirmed
        status |= ECrm_h; // update ejection charge main deployment to 1
        status &= ~ECm_h; // update ejection charge main continuity to 0
        SerialRaspi.println("MAIN Parachute Deployed, Enter Recovery");
        e32.sendMessage("MAIN parachue Deployed, Enter Recovery");
        setState(RECOVERY);
      }
      else {
        SerialRaspi.println("MAIN Parachute Deploy FAILED , trying backup ");
        e32.sendMessage("MAIN parachue Deploy FAILED , trying backup");
        // try backup
        triggerBackupEjectionCharges();
        delay(100); // wait for 100ms and check if deployment was success
        int status_d = checkBackupEjectionCharges();
        if (!status_d){
          // backup deoplyment is confirmed
          status |= ECrb_h; // update ejection charge backup deployment to 1
          status &= ~ECb_h; // update ejection charge backup continuity to 0
          SerialRaspi.println("BACKUP Parachute Deployed, Enter Recovery");
          e32.sendMessage("BACUKUP parachue Deployed, Enter Recovery");
          setState(RECOVERY);
        }
        else {
          // pray
        }
      }
    }
    // read gps 
    GPSData gps_d = readGPSData();

     // pack data
     String packed_data = packDATA(data) + packGPSDATA(gps_d);
     // send the data to the raspberry pi
     SerialRaspi.println(packed_data);
     // send the data to Telemetry 
     e32.sendMessage(packed_data);

     // delay 1 sec if vel is <safe Parachute vel
     if(data.vel_bmp<SAFE_PARACHUTE_VEL){
      delay(1000);
     } 
  }
  while (state== RECOVERY)
  {
    Data data = updateDataWithoutGPS();
     // read gps 
     GPSData gps_d = readGPSData();

     // pack data
     String packed_data = packDATA(data) + packGPSDATA(gps_d);
     // send the data to the raspberry pi
     SerialRaspi.println(packed_data);
     // send the data to Telemetry 
     e32.sendMessage(packed_data);


     if(data.vel_bmp<SAFE_PARACHUTE_VEL){
      delay(1000);
     } 

     //check if altitute is within 10 m of base altitude , we start beeping buzzer
     if (data.bmpAltitude - groundAltitude <10) {
        pinMode(BuzzerPin,HIGH);
        delay(500);
        pinMode(BuzzerPin,LOW);
        delay(500);
     }
     // do other things if vel is high
  }
  

}