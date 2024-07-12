#include "functions.h"

void setup() {

  displayInit();

  canInit();

}

void loop() {

  // Monitor Throttle Toggle Regen Method Button Press
  // Monitor Throttle Increase/Decrease Speed Button Press
  // If any of the throttle buttons are pressed:
  // - Send a CAN message to the motor controller to execute the request
  // - Update the corresponding variable to update the display value

  // Monitor eBrake Regenerative Braking Activation
  
  updateUserInputs();

  checkBrakeStateChange();

  cjec






  // Update the display to show the latest values
  updateDisplay();
  
}
