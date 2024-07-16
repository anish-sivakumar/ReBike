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

// Sets up the CAN bus communication, configuring necessary parameters for communication
// void canInit(); Commented out until CAN functionality is integrated to avoid compilation errors

// Initializes pin modes for the inputs with internal pull-up resistors
void pinModesInit();

// Initialize Timer to trigger ISR at 100 Hz (every 10 milliseconds)
void timerISRInit();

// ISR to be executed every 10 milliseconds, triggered by the timer
void timerISR();

// Handles the input request for throttle, updating the system state accordingly
void handleThrottleInput(int inputRequest); 

// Sends CAN message containing the current state of throttle, regenerative braking method, and activated regenerative braking status
void sendCANMSG(); 

// Updates system parameters that show the motor parameters (speed, power, temperature, battery percentage)
void updateSystemParams(); 

// Updates the display with the latest system information
void updateDisplay(); 

#endif // FUNCTIONS_H
