#include "bitmaps.h"
#include "functions.h"
#include "globals.h"

#include <U8g2lib.h>              // Library for handling OLED display
#include <SPI.h>                  // Include the SPI library
#include <Toggle.h>               // Include the Toggle library for handling button states


void setup() {

  // Initialize the OLED display
  u8g2.begin();

  Serial.begin(9600);

  // Display the logo during setup
  u8g2.firstPage();
  do {
    // Draw logo during system startup
    u8g2.drawBitmap(0, 0, 128/8, 64, epd_bitmap_REBIKE_Logo);
  } while (u8g2.nextPage());

  // Wait for 4 seconds to show the logo
  delay(4000);

  // Initialize I/O pins as inputs
  pinMode(THROTTLE_SPEED_ADJUSTMENT, INPUT);

  // Initialize the button toggles
  activateRegen.begin(E_BRAKE_ACTIVATED);
  toggleRegen.begin(REGEN_METHOD_TOGGLE_BUTTON);

  // Set analog read resolution (default is 10 bits on the Teensy)
  analogReadResolution(10); // Adjust resolution as needed (10 bits gives values from 0-1023)

  }

void loop() {

  // Read analog values for Throttle Increase/Decrease user inputs
  int UDANALOG = analogRead(THROTTLE_SPEED_ADJUSTMENT);

  // Poll the state of the Regenerative Braking Method Toggle Button
  activateRegen.poll(E_BRAKE_ACTIVATED);

  // Poll the state of the Regenerative Braking Method Toggle Button
  toggleRegen.poll(REGEN_METHOD_TOGGLE_BUTTON);

  // Functions to read incoming signals from the motor controller and BMS to update various
  // system parameters for UI display (speed, power, temperature, battery range)
  updateSpeed();
  updatePower();
  updateTemp();
  updateBatteryRange();

  // Check if the the mechanical break has been pulled to activate regenerative braking
  if (activateRegen.onPress()) {

    // Activate regenerative braking
    activateRegenRecovery();

  }

  // Check if the toggle regenerative braking button has been pressed
  if (toggleRegen.onPress()) {

    // Change the active regenerative braking method
    toggleRegenMethod();

  }

  // Update the display to show the latest values
  updateDisplay();
  
}
