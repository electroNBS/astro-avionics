
struct IMUReading{
    double roll;
    double yaw;
    double pitch;
    double acceleration;
};

IMUReading readIMU(){
    IMUReading reading = {0.0, 0.0, 0.0, 0.0};
    // TOTO: Implement the actual function below
    return reading;
}

void setup(){
    // start serial port at 115200 bps:
    Serial.begin(115200);
    // setup imu sensor
    // TOTO: Implement the actual setup below
}

void loop(){
    // Print acceleration
    IMUReading reading = readIMU();
    Serial.print("Acceleration: ");
    Serial.println(reading.acceleration);
    Serial.print("Roll: ");
    Serial.println(reading.roll);
    Serial.print("Yaw: ");
    Serial.println(reading.yaw);
    Serial.print("Pitch: ");
    Serial.println(reading.pitch);
    // wait for a second
    delay(1000);

}