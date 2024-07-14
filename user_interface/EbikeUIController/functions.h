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

// Initializes pin modes for the inputs with internal pull-up resistors
void pinModesInit();

// Initialize Timer to trigger ISR at 100 Hz (every 10 milliseconds)
void timerISRInit();

void timerISR();

// Sends CAN message containing the current state of throttle, regenerative braking method, and activated regenerative braking status. 
void sendCANMSG();

// Update display variables that show the motor parameters (speed, power, temperature, battery percentage)
void updateSystemParams();

// Updates the display with the latest system information
void updateDisplay();

#endif // FUNCTIONS_H
