#include <SoftwareSerial.h>

// Rewrite main to include acknowledge system, take into account the delayed start of rpi
// The rpi will wait for 20 seconds before entering failure mode, make sure some data is being sent every 10 seconds