#include "functions.h"
#include "logging.h"
#include <TimerOne.h>   // Include for Timer ISR configuration
#include <SPI.h>
#include "can.h"

// Global variables - only to be accessed within this file
uint16_t speed = 0;                    // Current speed
int8_t throttle = 0;                 // Current throttle percentage
int16_t power = 0;                    // Current power value
uint16_t temp = 0;                     // Current temperature
uint16_t batterySOC = 0;               // Battery state of charge of the e-bike

int regenMethod = 1;              // State of active regenerative braking
bool activatedRegen = false;           // Regenerative braking engaged
bool logging_enabled;

LoggingData loggingData;

// time
uint16_t timestampList[4];


// State variables for debouncings
volatile bool previousRegenState = HIGH; // Previous state of the REGEN_METHOD_TOGGLE pin
volatile bool currentIncreaseThrottleState = LOW; // Increase throttle request
volatile bool currentDecreaseThrottleState = LOW; // Increase throttle request
volatile bool previousIncreaseThrottleState = HIGH; // Previous state of the REGEN_METHOD_TOGGLE pin
volatile bool previousDecreaseThrottleState = HIGH; // Previous state of the REGEN_METHOD_TOGGLE pin

// Can variables and declarations
// FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can0;
CAN_message_t msg;                      // CAN message structure
int throttle_flag = 0;                  // Flag to indicate if throttle was changed
extern bool CAN_MB0_BikeStatus_Flag;    // Flag to indicate if CAN msg status changed
extern bool CAN_MB1_MotorVoltages_Flag; // Flag to indicate if CAN msg status changed
extern bool CAN_MB2_RealCurrents_Flag;  // Flag to indicate if CAN msg status changed
extern bool CAN_MB3_CalcValues_Flag;    // Flag to indicate if CAN msg status changed
extern bool CAN_MB4_BmsSoc_Flag;        // Flag to indicate if CAN msg status changed
int8_t throttleInput = 0; // value received from motor controller for verification purposes
uint16_t batterySOH = 0; // can track if we want, no real purpose unles we did extensive testing over a long time


// declaration of timer ISR 
void timerISR();

void setup() {
  Serial.begin(115200); // Set Serial debug baud rate to 115200

  // Activating logging is breaking the screen SPI, commented out for now
  // logging_enabled = loggingInit(LOGGING_CS_PIN);
  displayInit();
  canInit();
  pinModesInit();

  Timer1.initialize(10000); // Initialize Timer1 to trigger ISR at 100 Hz
  Timer1.attachInterrupt(timerISR); // Attach timerISR function to begin the timerISR loop
}

void loop() {
  // The loop function is left empty as all processing is done in the ISR
}

void timerISR() {
  // Static variables to store the last time an event was handled
  static unsigned long lastThrottleTime = 0;
  static unsigned long lastRegenTime = 0;
  unsigned long currentTime = millis(); // Get the current time in milliseconds

  // Poll THROTTLE_SPEED_INPUT pin (Analog read)
  int currentThrottleValue = analogRead(THROTTLE_SPEED_INPUT);

  // Determine throttle state based on current throttle value
  if (currentThrottleValue < 20) {
    currentIncreaseThrottleState = HIGH;  // Increase throttle request
    currentDecreaseThrottleState = LOW;
  } else if (currentThrottleValue > 52 && currentThrottleValue < 58) {
    currentIncreaseThrottleState = LOW;   // Decrease throttle request
    currentDecreaseThrottleState = HIGH;
  } else {
    currentIncreaseThrottleState = LOW;   // No throttle change
    currentDecreaseThrottleState = LOW;
    throttle_flag = 0;
  }

  // Check for throttle increase with debouncing
  if (currentIncreaseThrottleState != previousIncreaseThrottleState &&
      currentIncreaseThrottleState == HIGH &&
      currentTime - lastThrottleTime > 25) { // Debounce time of 25ms
    throttle_flag = handleThrottleInput(1, throttle, activatedRegen);  // Increase throttle request
    lastThrottleTime = currentTime; // Update last throttle event time
  }

  // Check for throttle decrease with debouncing
  if (currentDecreaseThrottleState != previousDecreaseThrottleState &&
      currentDecreaseThrottleState == HIGH &&
      currentTime - lastThrottleTime > 25) { // Debounce time of 25ms
    throttle_flag = handleThrottleInput(2, throttle, activatedRegen); // Decrease throttle request
    lastThrottleTime = currentTime; // Update last throttle event time
  }

  // Update previous throttle state for next comparison
  previousIncreaseThrottleState = currentIncreaseThrottleState;
  previousDecreaseThrottleState = currentDecreaseThrottleState;

  // Poll REGEN_METHOD_TOGGLE pin (Digital read)
  bool currentRegenState = digitalRead(REGEN_METHOD_TOGGLE);
  if (currentRegenState == LOW && previousRegenState == HIGH) {
    // Detect falling edge and check debounce time
    if (currentTime - lastRegenTime > 25) { // Debounce time of 25ms
      regenMethod = (regenMethod == 1) ? 2 : 1; // Toggle between regen method 1 and 2
      lastRegenTime = currentTime; // Update last regen event time
    }
  }
  previousRegenState = currentRegenState; // Update previous regen state

  // Poll E-BRAKE_ACTIVATED pin (Digital read)
  bool currentBrakeState = digitalRead(E_BRAKE_ENGAGED);
  if (currentBrakeState == LOW) {
    // As long as the brake is engaged, activate regen
    throttle_flag = handleThrottleInput(3, throttle, activatedRegen); // Ebrake request
  } else {
    if (activatedRegen) {
      throttle = 0; // Reset throttle to zero when eBrake is released
      activatedRegen = false; // When the brake is released, deactivate regen
    }
  }

  // receive values from CAN messages if on canbus
  if(CANPendingBikeStatusMsg()){
    CANGetBikeStatus(timestampList[0], speed, temp, throttleInput, loggingData.error);
  }
  if(CANPendingMotorVoltagesMsg()){
    CANGetMotorVoltages(timestampList[1], loggingData.vDC, loggingData.vA, loggingData.vB);
  }
  if(CANPendingRealCurrentsMsg()){
    CANGetRealCurrents(timestampList[2], loggingData.iDC, loggingData.iA, loggingData.iB);
  }
  if(CANPendingCalcValuesMsg()){
    CANGetCalcValues(timestampList[3], loggingData.iqRef, loggingData.iqFdb, power);
  }
  if(CANPendingBmsSocMsg()){
    CANGetBmsSoc(batterySOC, batterySOH);
  }
  // send throttle value to controller
  CANSendThrottleMsg(throttle);

  // Update display with current status
  updateDisplay(throttle, speed, power, temp, batterySOC, regenMethod);

  // reset throttle flag
  throttle_flag = false;
}
