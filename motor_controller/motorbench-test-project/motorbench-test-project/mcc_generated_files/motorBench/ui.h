/**
 * ui.h
 * 
 * Management of user interface components (pushbutton, LED, potentiometer)
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

#ifndef __UI_H
#define __UI_H

#include <stdbool.h>
#include <stdint.h>
#include "ui_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes UI state variables.
 * @param pui UI state
 */
void MCAF_UiInit(volatile MCAF_UI_DATA *pui);

/**
 * Reinitializes UI state variables when we are restarting
 * (exiting an error or debug state)
 * @param pui UI state
 */
inline static void MCAF_UiRestart(volatile MCAF_UI_DATA *pui)
{
    pui->run = false;
    pui->indicatorState.tickPeriod = MCUILD_PERIOD;
    pui->indicatorState.isError = false;    
}

/**
 * Executes tasks needed to run at the ISR rate
 * @param pui UI state
 */
void MCAF_UiStepIsr(volatile MCAF_UI_DATA *pui);

/**
 * Executes tasks needed to run in the main loop
 * @param pui UI state
 */
void MCAF_UiStepMain(volatile MCAF_UI_DATA *pui);

/**
 * Returns true at each UI tick.
 * @param pui UI state
 * @return true when the UI tick is occurring.
 */
inline static bool MCAF_UiTick(const volatile MCAF_UI_DATA *pui)
{
    return pui->isrCount == 0;
}

/**
 * Updates LEDs
 * @param p UI indicator state
 * @return a bitflag mask consisting of MCAF_UI_LED1_ON and MCAF_UI_LED2_ON
 */
uint16_t MCAF_UiCalculateIndicatorState(volatile MCAF_UI_INDICATOR_STATE *p);

/**
 * Setup UI to flashes an error code in a loop.
 * 
 * @param pindstate indicator states
 * @param code error code to display
 */
void MCAF_UiSetupFlashErrorCode(volatile MCAF_UI_INDICATOR_STATE *pindstate, uint16_t code);

/**
 * Record only new error codes
 *
 * @param pindstate indicator states
 * @param code
 */
inline static void MCAF_UiRecordNewError(volatile MCAF_UI_INDICATOR_STATE *pindstate, uint16_t code)
{
    if (!pindstate->isError)
    {
        MCAF_UiSetupFlashErrorCode(pindstate, code);
    }
}

/**
 * Returns true if there is an error condition
 * 
 * @param pindstate indicator state data
 * @return true if an error condition exists
 */
inline static bool MCAF_UiIsError(volatile const MCAF_UI_INDICATOR_STATE *pindstate)
{
    return pindstate->isError;
}

/**
 * Gets a copy of the error code
 * 
 * @param pindstate indicator state data
 * @return error code
 */
inline static uint16_t MCAF_UiGetErrorCode(volatile const MCAF_UI_INDICATOR_STATE *pindstate)
{
    return pindstate->code;
}

/**
 * Places a request to clear the fault condition and exit the fault state.
 * @param pui UI state data
 */
inline static void MCAF_UiExitFaultState(volatile MCAF_UI_DATA *pui)
{
    pui->exitFaultState = true;
}

/**
 * Flashes a severe error code forever in an infinite loop.
 * We disable interrupts and never return; the only way to stop the error-flashing
 * routine is to reset the microcontroller.
 * 
 * @param code error code to display
 */
void __attribute__((naked, noreturn)) MCAF_UiFlashErrorCodeForever(uint16_t code);

/**
 * Check whether there was a recent direction change.
 * 
 * @param pui UI state
 * @return true if a direction change was pending
 */
inline static bool MCAF_TestAndClearDirectionChangeFlag(volatile MCAF_UI_DATA *pui)
{
    if (pui->flags & MCAF_UI_DIRECTION_CHANGED)
    {
        pui->flags &= ~MCAF_UI_DIRECTION_CHANGED;
        return true;
    }
    else
    {   
        return false;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* __UI_H */