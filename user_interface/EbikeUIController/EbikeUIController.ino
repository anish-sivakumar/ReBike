// #include "bitmaps.h"
#include "functions.h"
#include "globals.h"

// #include <U8g2lib.h>              // Library for handling OLED display
// #include <SPI.h>                  // Include the SPI library
#include <Toggle.h>               // Include the Toggle library for handling button states
// #include <FlexCAN_T4.h>           // Include for CAN communication

void setup() {

  displayInit();

  canInit();

  // // Initialize I/O pins as inputs
  // pinMode(THROTTLE_SPEED_ADJUSTMENT, INPUT);

  // // Initialize the button toggles
  // activateRegen.begin(E_BRAKE_ACTIVATED);
  // toggleRegen.begin(REGEN_METHOD_TOGGLE_BUTTON);

  // // Set analog read resolution (default is 10 bits on the Teensy)
  // analogReadResolution(10); // Adjust resolution as needed (10 bits gives values from 0-1023)

  }

void loop() {

  // Monitor Throttle Toggle Regen Method Button Press
  // Monitor 







  // Update the display to show the latest values
  updateDisplay();

  // // Read analog values for Throttle Increase/Decrease user inputs
  // int UDANALOG = analogRead(THROTTLE_SPEED_ADJUSTMENT);

  // // Poll the state of the Regenerative Braking Method Toggle Button
  // activateRegen.poll(E_BRAKE_ACTIVATED);

  // // Poll the state of the Regenerative Braking Method Toggle Button
  // toggleRegen.poll(REGEN_METHOD_TOGGLE_BUTTON);

  // // Check if the the mechanical break has been pulled to activate regenerative braking
  // if (activateRegen.onPress()) {

  //   // Activate regenerative braking
  //   activateRegenRecovery();

  // }

  // // Check if the toggle regenerative braking button has been pressed
  // if (toggleRegen.onPress()) {

  //   // Change the active regenerative braking method
  //   toggleRegenMethod();

  // }
  
}
