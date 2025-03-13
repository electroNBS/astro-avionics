Nano Connections:
GPS -
D0 - GPS Tx
D1 - GPS Rx
Relays -
D9 - Main
D8 - Drogue
D7 - Backup
Continuity Check - 
Main - 
D5 - Power
A0 - Detect
Drogue - 
D4 - Power
D6 - Detect
Backup - 
D3 - Power
D2 - Detect
BMP390 - 
A4 - SDA
A5 - SCL
D10 - Power
IMU -
A4 - SDA
A5 - SCL
Buzzer - 
A1 - Power
E32 - 
D11 - M0
D12 - M1
A3 - E32 RX
A2 - E32 TX
Raspi - 
A6 - Raspi RX
A7 - Raspi TX

Raspi Connections:
GPS - 
13 - Force on
Relays - 
22 - Backup
27 - Drogue
17 - Main
Leds  - 
25 - Status
24 - Tel
BMP390 - 
SDA (GPIO2) - SDA
SCL (GPIO3) - SCL
Arduino - 
RX - A6
TX - A7

Continuity checking and ejection charge mechanism 

First power on respective power pin 
Delay of few 100 micro sec
Read input from respective read pin 
When detect is high ejection charge is not connected 
When low ejection charge is connected 
Update state 
Set power pin back to low

Check Continuity once every 300-400 cycles 


To deploy ejection charges When certain conditions meet set respective relay pins high

---

## Mission Requirements
1. Check if all modules are working
2. 