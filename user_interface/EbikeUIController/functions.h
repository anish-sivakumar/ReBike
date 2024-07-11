/*
 * functions.h
 *
 * This header file contains the declarations of functions used in the REBIKE project.
 * These functions handle various aspects of the bike's control system, including speed adjustment,
 * power updates, temperature monitoring, battery range updates, and display updates.
 */


#ifndef FUNCTIONS_H
#define FUNCTIONS_H


// Adjusts the speed request by a given adjustment value.
void adjustSpeedRequest(int adjustment);

// Updates the current speed reading.
void updateSpeed(void);

// Updates the current power reading.
void updatePower(void);

// Updates the current temperature reading.
void updateTemp(void);

// Updates the battery range estimation.
void updateBatteryRange(void);

// Activates regenerative recovery mode.
void activateRegenRecovery(void);

// Toggles between different regenerative braking methods.
void toggleRegenMethod(void);

// Updates the display with the latest information.
void updateDisplay(void);

#endif // FUNCTIONS_H
