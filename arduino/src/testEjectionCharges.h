#ifndef testEjectionCharges_H
#define testEjectionCharges_H

#define relayMain 9
#define relayDrogue 8
#define relayBackup 7

void setupRelays();
void triggerRelay(int relayPin);

#endif