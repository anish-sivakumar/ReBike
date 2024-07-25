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

// Define Controls connections
#define THROTTLE_SPEED_INPUT  18    // Pin for throttle increase/decrease speed user input (Analog pin)
#define REGEN_METHOD_TOGGLE   19    // Pin for regenerative braking method toggle
#define E_BRAKE               17    // Pin for E-Brake activation


// Initializes the OLED display during system startup
void displayInit();

// Initializes pin modes for the inputs with internal pull-up resistors
void pinModesInit();

// Handles the input request for throttle, updating the system state accordingly
bool handleThrottleInput(UserInputRequest inputRequest, int8_t &throttle, bool &activatedRegen, RegenMethod regenMethod ); 

// Updates the display with the latest system information
void updateDisplay(int8_t throttle, uint16_t speed, int16_t power, uint16_t temp, uint16_t batterySOC, int regenMethod);

#endif // FUNCTIONS_H
