# astro-avionics

Astro club BITS Pilani's flight computer code

---

### Arduino progress

[ ] - Make individual .cpp and .h files for each sensor and task

[ ] - Include them in main and finalize the logic

[ ] - Debugging and testing

### Raspi progress

[x] - Finalize the logic and implement it in python

[ ] - Debugging and testing

---

## Flight Computer Code Plan

1. Write out the cases and conditions of what happens when
2. Write the possible states and cases for transition between states & draw diagram of state machine
3. Write test code for individual components (i.e. taking readings from GPS, BMP390, IMU, communication with E32, Raspi, triggering relay, triggering buzzer, testing ejection charges)
4. Test the code
5. Back to step 3 until code works
6. Implement the state machine using state tables with placeholder function blocks for individual components
7. Integrate the test code for individual components from step 3 to implement the placeholder functions
8. Test the code with everything

### Flight regimes

Use a flight state variable and a switch case in loop to run the correct functions for each flight regime

ex:

enum State = {BOOT, READY, FLIGHT, DESCENT, LANDING, FAILURE}

State flightstate = BOOT

void loop(){
    switch BOOT:
    switch READY:
    .....
}

### Sensor states

Use a state variable(integer) to get and set states of individual sensors

#define RPI     (1 << 0)
#define GPS     (1 << 1)
#define LORA    (1 << 2)
#define BMP     (1 << 3)
#define IMU     (1 << 4)

int sensorStatus = 0         // This is a global variable

// To manipulate and check statuses

sensorStatus |= RPI          // Set status to true 
sensorStatus &= ~LORA        // Set status to false
sensorStatus ^= BMP          // Toggle status
(sensorStatus & IMU) != 0    // Get if a status is true

### Individual Components and Functions to Implement

1. GPS Readings (getCoordinates, check alive)
2. BMP390 reading (getAltitude, check alive)
3. IMU reading (getRollPitchYaw,getAcceleration, check alive, checkIsTipOver)
4. E32 communication and check (sendBytes function, check alive)
5. Raspi communication (sendBytes, readBytes, check alive)
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

### Instructions to Open PlatformIO project

1. Install PlatformIO extentions on arduino
2. Restart VSCode
3. Click on its symbol -> pick symbol , open the arduino folder
4. click on build symbol
