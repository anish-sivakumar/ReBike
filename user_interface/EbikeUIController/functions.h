/*
 * functions.h
 *
 * This header file contains the declarations of functions used in the REBIKE project.
 * These functions handle various aspects of the bike's control system, including speed adjustment,
 * power updates, temperature monitoring, battery range updates, and display updates.
 */

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

/////////////////////
// DISPLAY SECTION //
/////////////////////

#include <U8g2lib.h>      // Include the U8g2 library for OLED display

// Define OLED pin connections
#define OLED_CLK            13    // SCLK - Pin 3
#define OLED_MOSI           11    // SDA - Pin 4
#define OLED_RESET          23    // RES - Pin 5
#define OLED_DC             5     // DC - Pin 6
#define OLED_CS             10    // CS - Pin 7

//////////////////////
// CONTROLS SECTION //
//////////////////////

typedef enum tagUserInputRequest {
  INCREASE,
  DECREASE,
  REGEN
} UserInputRequest;

typedef enum tagRegenMethod {
  DIGITAL,
  ANALOG
} RegenMethod;

// Define Throttle connections
#define THROTTLE_SPEED_INPUT   18    // Pin for throttle increase/decrease speed user input (Analog pin)
#define REGEN_METHOD_TOGGLE    19    // Pin for regenerative braking method toggle

// Define E-Brake pin connections
#define E_BRAKE_ENGAGED        16    // Pin for E-Brake activation

/////////////////
// CAN SECTION // 
/////////////////
#include <FlexCAN_T4.h>

#define SYSTEM_PARAMS_ID  (uint32_t)0x330  // Motor parameters CAN identifier


// Initializes the OLED display during system startup
void displayInit();

// Initializes pin modes for the inputs with internal pull-up resistors
void pinModesInit();

// Handles the input request for throttle, updating the system state accordingly
bool handleThrottleInput(UserInputRequest inputRequest, int8_t &throttle, bool &activatedRegen, RegenMethod regenMethod ); 

// Updates system parameters that show the motor parameters (speed, power, temperature, battery percentage)
void updateSystemParams(const CAN_message_t &msg, uint16_t &speed); 

// Updates the display with the latest system information
void updateDisplay(int throttle, int speed, int power, int temp, int batterySOC, int regenMethod);

#endif // FUNCTIONS_H
