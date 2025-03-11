#define gpaTX D0 
#define gpaRX D1

struct GPSData {
    double latitude;
    double longitude;
};

GPSData readGPS() {
    GPSData data = {0.0, 0.0};
    // TOTO: Implement the actual function below
    return data;
}

void setup() {
    // start serial port at 115200 bps:
    Serial.begin(115200);
    // setup gps sensor
    // TOTO: Implement the actual setup below

}

void loop() {
    // Print latitude and longitude
    GPSData reading = readGPS();
    Serial.print("Latitude: ");
    Serial.println(latitude.latitude);
    Serial.print("Longitude: ");
    Serial.println(longitude.longitude);
    // wait for a second
    delay(1000);
}