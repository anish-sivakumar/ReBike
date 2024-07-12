/*
 * functions.h
 *
 * This header file contains the declarations of functions used in the REBIKE project.
 * These functions handle various aspects of the bike's control system, including speed adjustment,
 * power updates, temperature monitoring, battery range updates, and display updates.
 */


#ifndef FUNCTIONS_H
#define FUNCTIONS_H


// Initializes the OLED display during system startup
void displayInit();

// Initializes CAN communication
void canInit();

// Function to  Sends the current throttle value and active regeneration mode via CAN to motor controller
void sendThrottleRegenMSG();

// Function to update the display variable that shows the motor parameters (speed, power, temperature)
void updateMotorParams(const CAN_message_t &msg);

// Function to update the display variable that shows the current battery range
void updateBatterySOC(const CAN_message_t &msg);

// Updates the display with the latest information.
void updateDisplay(void);

#endif // FUNCTIONS_H
