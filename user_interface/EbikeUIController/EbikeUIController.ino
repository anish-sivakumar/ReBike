#include "functions.h"
#include "logging.h"
#include <TimerOne.h>  // Include for Timer ISR configuration
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

LoggingData loggingData;

// time
uint16_t timestampList[4] = {};
uint16_t timestampExpected = 0;

LoggingData LD;

// State variables for debouncings
volatile bool previousRegenState = HIGH;             // Previous state of the REGEN_METHOD_TOGGLE pin
volatile bool currentIncreaseThrottleState = LOW;    // Increase throttle request
volatile bool currentDecreaseThrottleState = LOW;    // Increase throttle request
volatile bool previousIncreaseThrottleState = HIGH;  // Previous state of the REGEN_METHOD_TOGGLE pin
volatile bool previousDecreaseThrottleState = HIGH;  // Previous state of the REGEN_METHOD_TOGGLE pin

// Can variables
CAN_message_t msg;                      // CAN message structure
int throttle_flag = 0;                  // Flag to indicate if throttle was changed
int8_t throttleInput = 0; // value received from motor controller for verification purposes
uint16_t batterySOH = 0; // can track if we want, no real purpose unles we did extensive testing over a long time


// declaration of timer ISR
void timerISR();

void setup() {
  Serial.begin(115200); // Set Serial debug baud rate to 115200

  loggingInit(LOGGING_CS_PIN);
  displayInit();
  canInit();
  pinModesInit();

  Timer1.initialize(20000);          // Initialize Timer1 to trigger ISR at 50 Hz
  Timer1.attachInterrupt(timerISR);  // Attach timerISR function to begin the timerISR loop
}

void loop() {
  // The loop function is left empty as all processing is done in the ISR
}

void timerISR() {

  // Static variables to store the last time an event was handled
  static unsigned long lastThrottleTime = 0;
  static unsigned long lastRegenTime = 0;
  unsigned long currentTime = millis();  // Get the current time in milliseconds

  // Poll THROTTLE_SPEED_INPUT pin (Analog read)
  int currentThrottleValue = analogRead(THROTTLE_SPEED_INPUT);
  Serial.println(currentThrottleValue);
  // Determine throttle state based on current throttle value
  if (currentThrottleValue < 20) {
    currentIncreaseThrottleState = HIGH;  // Increase throttle request
    currentDecreaseThrottleState = LOW;
  } else if (currentThrottleValue > 52 && currentThrottleValue < 70) {
    currentIncreaseThrottleState = LOW;  // Decrease throttle request
    currentDecreaseThrottleState = HIGH;
  } else {
    currentIncreaseThrottleState = LOW;  // No throttle change
    currentDecreaseThrottleState = LOW;
    throttle_flag = 0;
  }

  // Check for throttle increase with debouncing
  if (currentIncreaseThrottleState != previousIncreaseThrottleState &&
      currentIncreaseThrottleState == HIGH &&
      currentTime - lastThrottleTime > 25) { // Debounce time of 25ms
    handleThrottleInput(INCREASE, throttle, activatedRegen, DIGITAL);  // Increase throttle request
    lastThrottleTime = currentTime; // Update last throttle event time
  }

  // Check for throttle decrease with debouncing
  if (currentDecreaseThrottleState != previousDecreaseThrottleState &&
      currentDecreaseThrottleState == HIGH &&
      currentTime - lastThrottleTime > 25) { // Debounce time of 25ms
    handleThrottleInput(DECREASE, throttle, activatedRegen, DIGITAL); // Decrease throttle request
    lastThrottleTime = currentTime; // Update last throttle event time
  }

  // Update previous throttle state for next comparison
  previousIncreaseThrottleState = currentIncreaseThrottleState;
  previousDecreaseThrottleState = currentDecreaseThrottleState;

  // Poll REGEN_METHOD_TOGGLE pin (Digital read)
  bool currentRegenState = digitalRead(REGEN_METHOD_TOGGLE);
  if (currentRegenState == LOW && previousRegenState == HIGH) {
    // Detect falling edge and check debounce time
    if (currentTime - lastRegenTime > 25) {      // Debounce time of 25ms
      regenMethod = (regenMethod == 1) ? 2 : 1;  // Toggle between regen method 1 and 2
      lastRegenTime = currentTime;               // Update last regen event time
    }
  }
  previousRegenState = currentRegenState;  // Update previous regen state

  // Poll E-BRAKE_ACTIVATED pin (Analog read)
  int currentBrakeState = map(analogRead(E_BRAKE), 280, 980, 0, -100);
  // Clamp the value of analog brake to -100.
  currentBrakeState = (currentBrakeState < -100) ? -100 : currentBrakeState;
  if (currentBrakeState <= -5) { //analog ebrake sits at -2 at above mapping
    throttle = (int8_t)currentBrakeState;
    if (regenMethod == 1) {
      handleThrottleInput(REGEN, throttle, activatedRegen, DIGITAL); // Digital Ebrake request
    } 
    else if (regenMethod == 2) {
      handleThrottleInput(REGEN, throttle, activatedRegen, ANALOG); // Analog Ebrake request
    }
    
  } else {
    if (activatedRegen) {
      throttle = 0;            // Reset throttle to zero when eBrake is released
      activatedRegen = false;  // When the brake is released, deactivate regen
    }
  }
  // Serial.println("Throttle: ");
  // Serial.println(throttle);

  bool gotNewTimestamp = false;
  if(CANPendingBikeStatusMsg()){  // receive values from CAN messages if there are pending messages
    CANGetBikeStatus(timestampList[0], speed, temp, throttleInput, loggingData.error);
    gotNewTimestamp = true;
  }
  if(CANPendingMotorVoltagesMsg()){
    CANGetMotorVoltages(timestampList[1], loggingData.vDC, loggingData.vA, loggingData.vB);
    gotNewTimestamp = true;
  }
  if(CANPendingRealCurrentsMsg()){
    CANGetRealCurrents(timestampList[2], loggingData.iDC, loggingData.iA, loggingData.iB);
    gotNewTimestamp = true;
  }
  if(CANPendingCalcValuesMsg()){
    CANGetCalcValues(timestampList[3], loggingData.iqRef, loggingData.iqFdb, power);
    gotNewTimestamp = true;
  }
  if(CANPendingBmsSocMsg()){
    CANGetBmsSoc(batterySOC, batterySOH);
    //Serial.print(batterySOC); Serial.println(batterySOH);
  }
  if(CANPendingBmsVIMsg()){
    CANGetBmsVI(loggingData.vBMS, loggingData.iBMS);
  }
  // send throttle value to controller
  CANSendThrottleMsg(throttle);

  // Do a logging step if were ready. If theres no logging to do, update the screen.
  if (gotNewTimestamp && loggingStepReady(timestampList, timestampExpected)){
    loggingStepWrite(timestampExpected, throttle, speed, power, temp, batterySOC, regenMethod, loggingData);
    timestampExpected = loggingNextExpectedTimestamp(timestampList);
  } else {
    // Update display with current status
    updateDisplay(-100, speed, -800, temp, batterySOC, regenMethod);
  }

  // reset throttle flag
  throttle_flag = false;
}
