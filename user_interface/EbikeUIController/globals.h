/*
 * globals.h
 *
 * This header file contains the global variables and objects used in the REBIKE project.
 * It includes declarations for variables related to speed, throttle, power, temperature,
 * battery range, and regenerative braking. Additionally, it declares strings used for
 * displaying these values and includes the Toggle library for handling button states.
 */

#ifndef GLOBALS_H
#define GLOBALS_H

#include <Toggle.h>  // Include the Toggle library for handling button states
#include <U8g2lib.h>  // Include the U8g2 library for OLED display

// Define OLED pin connections
#define OLED_CLK    13    // SCLK - Pin 3
#define OLED_MOSI   11    // SDA - Pin 4
#define OLED_RESET  23    // RES - Pin 5
#define OLED_DC     5     // DC - Pin 6
#define OLED_CS     10    // CS - Pin 7

// Create an instance for a 128x64 display with software SPI
U8G2_SSD1306_128X64_NONAME_1_4W_SW_SPI u8g2(U8G2_R0, /* clk=*/ OLED_CLK, /* data=*/ OLED_MOSI, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RESET);

// Define E-Brake pin connections
#define E_BRAKE_ACTIVATED   22    // Pin for E-Brake activation

// Define Throttle connections
#define REGEN_METHOD_TOGGLE_BUTTON  19    // Pin for regenerative braking method toggle
#define THROTTLE_SPEED_ADJUSTMENT   18    // Pin for throttle speed adjustment

// Global variables
int speed = 0;            // Current speed
uint8_t throttle = 0;     // Max throttle percentage
int power = 0;            // Current power value
int temp = 0;             // Current temperature
int batterySOC = 0;       // Battery state of charge of the e-bike
int activeRegen = 1;      // State of active regenerative braking
CAN_message_t msg;				// declare CAN message 
int throttle_flag = 0;		// flag to indicate if throttle value has changed
int regen_flag = 0;				// flag to indicate if regen method has changed 

//define CAN IDs
const uint32_t Motor_ID = 0x350; // Motor parameters CAN identifier, byte 0 and 1 = Power of the e-bike system, byte 2 and 3 = Speed of the e-bike system, byte 4 and 5 = Temperature of the e-bike system 
const uint32_t Soc_ID = 0x355; // Battery state of charge CAN identifier

// Strings for displaying values
char speed_string[10];        // String to display speed
int speed_string_length;      // Length of the speed string

char throttle_string[10];     // String to display throttle value
int throttle_string_length;   // Length of the throttle string

char power_string[10];        // String to display power value
int power_string_length;      // Length of the power string

char temp_string[10];         // String to display temperature
int temp_string_length;       // Length of the temperature string

char battery_string[10];      // String to display battery range
int battery_string_length;    // Length of the battery string

// Button objects for regenerative braking
Toggle activateRegen(E_BRAKE_ACTIVATED);  // Button that activates regenerative braking
Toggle toggleRegen(REGEN_METHOD_TOGGLE_BUTTON);  // Button to toggle regenerative braking method

#endif // GLOBALS_H
