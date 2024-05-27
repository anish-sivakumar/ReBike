/**
 * ui_types.h
 * 
 * Type definitions of user interface components (pushbutton, LED, potentiometer)
 * 
 * Component: external interface
 */

/* *********************************************************************
 *
 * Motor Control Application Framework
 * R7/RC37 (commit 116330, build on 2023 Feb 09)
 *
 * (c) 2017 - 2023 Microchip Technology Inc. and its subsidiaries. You may use
 * this software and any derivatives exclusively with Microchip products.
 *
 * This software and any accompanying information is for suggestion only.
 * It does not modify Microchip's standard warranty for its products.
 * You agree that you are solely responsible for testing the software and
 * determining its suitability.  Microchip has no obligation to modify,
 * test, certify, or support the software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH
 * MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY
 * APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL,
 * PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF
 * ANY KIND WHATSOEVER RELATED TO THE USE OF THIS SOFTWARE, THE
 * motorBench(R) DEVELOPMENT SUITE TOOL, PARAMETERS AND GENERATED CODE,
 * HOWEVER CAUSED, BY END USERS, WHETHER MICROCHIP'S CUSTOMERS OR
 * CUSTOMER'S CUSTOMERS, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
 * CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
 * OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
 * SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
 * THESE TERMS.
 *
 * *****************************************************************************/

#ifndef __UI_TYPES_H
#define __UI_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Bit flags for UI elements
 */
typedef enum
{
    MCAF_UI_BTN1_PRESSED    = 0x01,   /** history of whether button1 was pressed */
    MCAF_UI_BTN2_PRESSED    = 0x02,   /** history of whether button2 was pressed */
    MCAF_UI_LED1_ON         = 0x04,   /** we want LED 1 to be turned on */
    MCAF_UI_LED2_ON         = 0x08,   /** we want LED 2 to be turned on */
    MCAF_UI_REVERSE         = 0x10,   /** true if we want to spin in reverse */
    MCAF_UI_DIRECTION_CHANGED = 0x20,  /** direction change event has occurred */

    MCAF_UI_BOTH_LEDS_ON = MCAF_UI_LED1_ON | MCAF_UI_LED2_ON,   /** we want both LEDs to be turned on */
} MCAF_UI_FLAGS;

/*
 * Bit positions for the above bit flags (except BOTH_LEDS_ON)
 */
enum
{
    MCAF_UI_BTN1_PRESSED_BIT = 0,
    MCAF_UI_BTN2_PRESSED_BIT = 1,
    MCAF_UI_LED1_ON_BIT      = 2,
    MCAF_UI_LED2_ON_BIT      = 3,
    MCAF_UI_REVERSE_BIT      = 4,
    MCAF_UI_DIRECTION_CHANGED_BIT = 5,
};

/**
 * UI LED duty cycles
 */
enum
{
    MCUILD_OFF  = 0,
    MCUILD_LOW  = 1,
    MCUILD_HIGH = 15,        
    MCUILD_ON   = 16,
    
    MCUILD_PERIOD = MCUILD_ON,
};

/**
 * State variables for UI indicators (LEDs)
 */
typedef struct tagUIIndicator
{
    uint16_t code;           /** display code, used with isError = true */
    uint16_t codeDigit;      /** internal state for error code display */
    uint8_t  duty1;          /** duty cycle for LED 1, used with isError = false */
    uint8_t  duty2;          /** duty cycle for LED 2, used with isError = false */
    uint8_t  pause;          /** pause count for isError = true */
    uint8_t  tickCounter;    /** tick counter for generating adjustable-duty-cycle pulses */
    uint8_t  tickPeriod;     /** tick period */
    bool     isError;        /** whether the code is an error */
} MCAF_UI_INDICATOR_STATE;

/**
 * Setup information for potentiometer scaling 
 */
typedef struct
{
    int16_t potScalingFullRange;
    int16_t constantTermFullRange;
    int16_t constantTermHalfRange;    
} MCAF_POT_SCALING;

/**
 * UI state variables
 */
typedef struct tagUIData
{
    bool     run;                            /** determines whether we want the motor to run */
    bool     exitFaultState;                 /** determines whether we want to clear any ongoing fault condition
                                              *  and exit the fault state */
    uint16_t flags;                          /** one or more bits from MCAF_UI_FLAGS */
    /**
     * counter to keep track of how many ISR executions have occurred, 
     * used here for timing purposes
     */
    uint16_t isrCount;                       
    /** indicator state */
    MCAF_UI_INDICATOR_STATE indicatorState;
    struct {
        int16_t gain;
        int16_t offset;
    } velocityScaling;
} MCAF_UI_DATA;

#ifdef __cplusplus
}
#endif

#endif /* __UI_TYPES_H */