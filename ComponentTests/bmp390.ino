

/*
This function reads the BMP390 sensor and returns the altitude in meters.
*/
int getAltitude()
{
    // This is a placeholder function that returns a fixed value
    // TOTO: Implement the actual function
    return 100;
}

void setup()
{
    // start serial port at 115200 bps:
    Serial.begin(115200);

    // setup bmp sensor
    // TOTO: Implement the actual setup below



}

void loop()
{
    // Print altitude
    int altitude = getAltitude();
    Serial.print("Altitude: ");
    Serial.println(altitude);
    // wait for a second
    delay(1000);
}