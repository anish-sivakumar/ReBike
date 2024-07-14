#include "functions.h"
#include "globals.h"

// State variables for debouncing
volatile bool previousRegenState = HIGH; // Previous state of the REGEN_METHOD_TOGGLE pin
volatile bool previousBrakeState = HIGH; // Previous state of the E_BRAKE_ENGAGED pin

int throttleState = 0; // Current throttle state
int previousThrottleState = 0; // Previous throttle state

void setup() {
  displayInit();
  canInit();
  pinModesInit();
  timerISRInit();
}

void loop() {
  // The loop function is left empty as all processing is done in the ISR
}

void timerISR() {
  // Static variables to store the last time an event was handled
  static unsigned long lastThrottleTime = 0;
  static unsigned long lastRegenTime = 0;
  static unsigned long lastBrakeTime = 0;
  unsigned long currentTime = millis(); // Get the current time in milliseconds

  bool throttleInputDetected = pollThrottleState(previousThrottleState, lastThrottleTime);
  if (throttleInputDetected) {
    handleThrottleInput(throttleState);
  }

  bool regenMethodChanged = pollRegenMethodState(previousRegenState, lastRegenTime);
  if (regenMethodChanged) {
    regenMethod = (regenMethod == 1) ? 2 : 1; // Toggle between regen method 1 and 2
  }
  
  bool brakeActivated = pollBrakeState(previousBrakeState, lastBrakeTime);
  if (brakeActivated) {
    activatedRegen = (activatedRegen == 0) ? 1 : 0; // Engage regenerative braking
  }

  // Check if any flag is set to send CAN message
  if (throttle_flag == 1) {
    sendCANMSG(); // Send CAN message if any flag is set
  }

  updateDisplay();
}

bool pollThrottleState(int &previousThrottleState, unsigned long &lastThrottleTime) {
  unsigned long currentTime = millis(); // Get the current time in milliseconds

  // Poll THROTTLE_SPEED_INPUT pin (Analog read)
  int currentThrottleState = analogRead(THROTTLE_SPEED_INPUT);

  if (currentTime - lastThrottleTime > 25) { // Debounce time of 25ms
    if (currentThrottleState != previousThrottleState) {
      // Check for throttle up or down based on analog values
      if (currentThrottleState <= 1) {
        throttleState = +1; // Increase throttle request
      } else if (currentThrottleState == 63) {
        throttleState = -1; // Decrease throttle request
      } else {
        throttleState = 0; // No change
      }
      lastThrottleTime = currentTime; // Update last throttle event time
      previousThrottleState = currentThrottleState; // Update previous throttle state
      return true; // Return true when throttle state changes
    }
  }

  return false; // Return false if no change or debounce not passed
}

void handleThrottleInput(int inputRequest) {
  if (inputRequest == 1) {
    // Increase throttle if not at maximum
    if (throttle == 100) {
      return;
    } else {
      throttle += 5; // Increase throttle by 5
      throttle_flag = 1; // Set throttle flag
    }
  } else if (inputRequest == -1) {
    // Decrease throttle if not at minimum
    if (throttle == 0) {
      return;
    } else {
      throttle -= 5; // Decrease throttle by 5
      throttle_flag = 1; // Set throttle flag
    }
  }
  msg.buf[0] = throttle; // Update CAN message buffer with new throttle value
}

bool pollRegenMethodState(bool previousRegenState, unsigned long &lastRegenTime) {
  unsigned long currentTime = millis();

  // Poll REGEN_METHOD_TOGGLE pin (Digital read)
  bool currentRegenState = digitalRead(REGEN_METHOD_TOGGLE);

  // Check for falling edge and debounce
  if (currentRegenState == LOW && previousRegenState == HIGH) {
    if (currentTime - lastRegenTime > 25) { // Debounce time of 25ms
      lastRegenTime = currentTime; // Update last regen event time
      return true; // Return true when regen state changes
    }
  }

  // Update previous regen state if no state change
  previousRegenState = currentRegenState;

  return false; // Return false if no change or debounce not passed
}

bool pollBrakeState(bool previousBrakeState, unsigned long &lastBrakeTime) {
  unsigned long currentTime = millis(); // Get the current time in milliseconds

  // Poll E_BRAKE_ENGAGED pin (Digital read)
  bool currentBrakeState = digitalRead(E_BRAKE_ENGAGED);
  
  // Check for falling edge and debounce
  if (currentBrakeState == LOW && previousBrakeState == HIGH) {
    if (currentTime - lastBrakeTime > 25) { // Debounce time of 25ms
      lastBrakeTime = currentTime; // Update last brake event time
      return true; // Return true when brake state changes
    }
  }

  // Update previous brake state if no state change
  previousBrakeState = currentBrakeState;
  
  return false; // Return false if no change or debounce not passed
}