#include "functions.h"

// State variables for debouncing
volatile bool previousRegenState = HIGH; // Previous state of the REGEN_METHOD_TOGGLE pin
volatile bool previousBrakeState = HIGH; // Previous state of the E_BRAKE_ENGAGED pin

int throttleState = 0; // Current throttle state
int previousThrottleState = 0; // Previous throttle state

void setup() {
  displayInit();
  canInit();
  pinModesInit();
  timerISRinit();
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

  // Poll THROTTLE_SPEED_INPUT pin (Analog read)
  int currentThrottleState = analogRead(THROTTLE_SPEED_INPUT);
  if (currentTime - lastThrottleTime > 25) { // Debounce time of 25ms
    if (currentThrottleState != previousThrottleState) {
      // Check for throttle up or down based on analog values
      if (currentThrottleState <= 1) {
        handleThrottleInput(+1); // Increase throttle request
      } else if (currentThrottleState == 63) {
        handleThrottleInput(-1); // Decrease throttle request
      }
      throttleState = currentThrottleState; // Update current throttle state
      lastThrottleTime = currentTime; // Update last throttle event time
    }
    previousThrottleState = currentThrottleState; // Update previous throttle state
  }

  // Poll REGEN_METHOD_TOGGLE pin (Digital read)
  bool currentRegenState = digitalRead(REGEN_METHOD_TOGGLE);
  if (currentRegenState == LOW && previousRegenState == HIGH) {
    // Detect falling edge and check debounce time
    if (currentTime - lastRegenTime > 25) { // Debounce time of 25ms
      handleToggleRegenInput(); // Handle regen toggle input
      lastRegenTime = currentTime; // Update last regen event time
    }
  }
  previousRegenState = currentRegenState; // Update previous regen state

  // Poll E_BRAKE_ENGAGED pin (Digital read)
  bool currentBrakeState = digitalRead(E_BRAKE_ENGAGED);
  if (currentBrakeState == LOW && previousBrakeState == HIGH) {
    // Detect falling edge and check debounce time
    if (currentTime - lastBrakeTime > 25) { // Debounce time of 25ms
      handleEBrakeInput(); // Handle e-brake input
      lastBrakeTime = currentTime; // Update last brake event time
    }
  }
  previousBrakeState = currentBrakeState; // Update previous brake state

  // Check if any flag is set to send CAN message
  if (throttle_flag == 1 || regen_toggle_flag == 1 || regen_active_flag == 1) {
    sendCANMSG(); // Send CAN message if any flag is set
  }

  updateDisplay();
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

void handleToggleRegenInput() {
  regenMethod = (regenMethod == 1) ? 2 : 1; // Toggle between regen method 1 and 2
  toggle_regen_flag = 1; // Set regen toggle flag
  msg.buf[1] = regenMethod; // Update CAN message buffer with new regen method
}

void handleEBrakeInput() {
  activatedRegen = (activatedRegen == 0) ? 1 : 0; // Toggle e-brake activation
  regen_active_flag = 1; // Set regen active flag
  msg.buf[2] = activatedRegen; // Update CAN message buffer with new e-brake status
}
