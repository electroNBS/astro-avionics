/*
E32 - 
D11 - M0
D12 - M1
A3 - E32 RX
A2 - E32 TX
*/
#define e32RX A3
#define e32TX A2
#define e32M0 D11
#define e32M1 D12

void setup(){
    // setup the serial communication
    Serial.begin(115200);

    // setup E32
}

// This function sends a message(byte array) to the E32 module
void sendMessage(byte message[], int length){
    // TOTO: Implement the actual function below
}

void loop(){
    // TOTO: Implement the actual loop below
    Serial.println("Sending message");
    byte message[] = {0x01, 0x02, 0x03};
    sendMessage(message, 3);
    
    delay(1000);
}