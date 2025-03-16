#include <Wire.h>
#include <Adafruit_BMP3XX.h>

#define bmpSDA A4 
#define bmpSCL A5

/*
This function reads the BMP390 sensor and returns the altitude in meters.
*/

Adafruit_BMP3XX bmp;

float getAltitude()
{
    if (!bmp.performReading()) {
        Serial.println("Faild to read from BMP390 sensor!");
        return -1; // Return -1 in case of failure
    }

    float pressure = bmp.pressure; // Pressure in Pascals
    float temperature = bmp.temperature; // Temperature in Celsius

    // Convert pressure to altitude using the barometric formula
    float seaLevelPressure = 101325; // Standard sea-level pressure in Pascals

    //using the standard barometric formula h = (T0/L)(1-(P/P0)^(RL/gM))
    // T0 is sea level temp, L is temp. lapse rate, P0 is sea level pressure, R is gas const,
    // M is molar mass of air
    float altitude = 44330 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));

    return altitude;
}

void setup()
{
    // start serial port at 115200 bps:
    Serial.begin(115200);
    Wire.begin(bmpSDA, bmpSCL); // Start I2C with defined pins

    // setup bmp sensor
    if (!bmp.begin_I2C()) { // Initialize BMP390 over I2C
        Serial.println("Could not find a valid BMP390 sensor, check wiring!");
        //makes the prog enter an infinite loop, stops execution if sensor is faulty
        while (1);
    }
    
    //oversampling to reduce noise and improve accuracy
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);

    //filtering to reduce noise
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
}

void loop()
{
    // Print altitude
    float altitude = getAltitude();
    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println(" meters");
    // wait for a second
    delay(1000);
}