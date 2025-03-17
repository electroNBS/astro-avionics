#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_ICM20649.h>
#include <Arduino.h>
#include <math.h>

// Structure to hold IMU readings
struct IMUReading {
    double roll;
    double yaw;
    double pitch;
    double acceleration;
};

// Function prototypes
void calibrateIMU(int samples = 500);
IMUReading readIMU();

#endif