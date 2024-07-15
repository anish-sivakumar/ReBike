#include "functions.h"

void setup() {
  displayInit();
  // canInit();
  pinModesInit();
  timerISRInit();
  Serial.begin(9600);
}

void loop() {
  // The loop function is left empty as all processing is done in the ISR
}