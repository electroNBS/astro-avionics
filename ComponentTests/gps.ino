#include <SoftwareSerial.h>

int D1,D0;


// Define GPS TX and RX pins
const int gps_RX = D1;  // GPS TX pin connected to Arduino RX 
const int gps_TX = D0;  // GPS RX pin connected to Arduino TX 

// Create a SoftwareSerial object for GPS communication
SoftwareSerial gpsSerial(gps_RX, gps_TX);

// Structure to store parsed GPS data
struct GPSData {
    double latitude;
    double longitude;
    bool isValid;
};

// Function to read and parse GPS data
GPSData readGPS() {
    GPSData data = {0.0, 0.0, false};  // Initialize with default values

    // Read a line of data from the GPS module
    if (gpsSerial.available()) {
        String rawData = gpsSerial.readStringUntil('\n');  // Read until newline

        // Simulate parsing (replace this with actual parsing logic if needed)
        // Example: Assume raw data is in the format "latitude,longitude"
        int commaIndex = rawData.indexOf(',');  // Find the comma separator
        if (commaIndex > 0) {
            data.latitude = rawData.substring(0, commaIndex).toDouble();  // Extract latitude
            data.longitude = rawData.substring(commaIndex + 1).toDouble();  // Extract longitude
            data.isValid = true;  // Mark data as valid
        }
    }

    return data;
}

void setup() {
    // Start serial communication for debugging
    Serial.begin(115200);

    // Start GPS serial communication
    gpsSerial.begin(9600);

    Serial.println("Initializing GPS...");
}

void loop() {
    // Read and parse GPS data
    GPSData data = readGPS();

    // Check if GPS data is valid and print it
    if (data.isValid) {
        Serial.print("Latitude: ");
        Serial.println(data.latitude, 6);  // Print latitude with 6 decimal places
        Serial.print("Longitude: ");
        Serial.println(data.longitude, 6); // Print longitude with 6 decimal places
    } else {
        Serial.println("No valid GPS data available.");
    }

    // Wait for 1 second before the next reading
    delay(1000);
}
