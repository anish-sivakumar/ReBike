#include "functions.h"

void setup() {
  displayInit();
  // canInit(); Commented out until CAN functionality is integrated to avoid compilation errors
  pinModesInit();
  timerISRInit();
  Serial.begin(9600);
}

void loop() {
  // The loop function is left empty as all processing is done in the ISR
}