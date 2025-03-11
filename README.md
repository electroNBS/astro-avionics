# astro-avionics

Astro club BITS Pilani's flight computer code

## Flight Computer Code Plan

1. Write out the cases and conditions of what happens when
2. Write the possible states and cases for transition between states & draw diagram of state machine
3. Write test code for individual components (i.e. taking readings from GPS, BMP390, IMU, communication with E32, Raspi, triggering relay, triggering buzzer, testing ejection charges)
4. Test the code
5. Back to step 3 until code works
6. Implement the state machine using state tables with placeholder function blocks for individual components
7. Integrate the test code for individual components from step 3 to implement the placeholder functions
8. Test the code with everything

### Individual Components and Functions to Implement

1. GPS Readings (getCoordinates)
2. BMP390 reading (getAltitude)
3. IMU reading (getRollPitchYaw,getAcceleration)
4. E32 communication (sendBytes function)
5. Raspi communication (sendBytes, readBytes)
6. Continuity Check (checkContinuity main, drogue, backup)
7. Trigger Ejection Charges (trigger main, drogue, backup)
8. Trigger Ejection Charges (from Raspi) (trigger main, drogue, backup)
9. BMP390 readings (from Raspi) (getAltitude)
10. Arduino communication (from Raspi) (readBytes, sendBytes)

### Parts to Implement After Code Integration (Not Required in State Machine)

1. packTelemetryDataE32() // function combines all data (which is decided to be sent) into a byte array
2. packLogsForRaspi() // combines all data into byte array
3. storeDataFromArduinoToSDCardRaspi() // unpacks data and stores on sd card
4. storeCameraFootagetoSDCardRaspi()

### Code Structure

Individual components test code will be written in Arduino IDE as .ino

State machine and onwards will be written in PlatformIO in VSCode

Raspi code will be written in Python

### State Machine

