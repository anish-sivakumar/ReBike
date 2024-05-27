/**
 * ui.c
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

/*
 * __delay_us() requires FCY to be defined before #including libpic30.h
 * hal.h needs to be included before libpic30.h
 */
 
#include "ui.h"
#include "util.h"
#include "hal.h"
#include <libpic30.h>
#include "parameters/operating_params.h"
#include "parameters/timing_params.h"
#include "mcaf_watchdog.h"
#include "board_service.h"

enum
{
    MCUI_PAUSE_TIME_END_OF_CODE = 3,
    MCUI_PAUSE_TIME_END_OF_DIGIT = 1,
    MCUI_INDICATOR_TICK_PERIOD_UILOOPS = 16, /** LED flash repeat time in UI loops */
    
    MCUI_ERROR_CODE_TICK_PERIOD_UILOOPS = 6, /** error code tick time in UI loops */
    MCUI_ERROR_CODE_LOOP_DELAY_MICROSECONDS = 625, /** error code tick time in us */
    MCUI_ERROR_CODE_LOOP_DELAYS_PER_TICK = 100, /** number of delays per tick */
};

void MCAF_UiInit(volatile MCAF_UI_DATA *pui)
{
    pui->run = false;
    pui->exitFaultState = false;
    pui->isrCount = 0;
    pui->flags = 0;
    pui->indicatorState.tickCounter = 0;
    pui->indicatorState.tickPeriod = MCUI_INDICATOR_TICK_PERIOD_UILOOPS;
    pui->indicatorState.isError = false;
    pui->indicatorState.pause = 0;
    pui->indicatorState.duty1 = 0;
    pui->indicatorState.duty2 = 0;
    pui->indicatorState.codeDigit = 0;
    
    pui->velocityScaling.gain   = MCAF_VELOCITY_COMMAND_MAX - MCAF_VELOCITY_COMMAND_MIN;
    pui->velocityScaling.offset = MCAF_VELOCITY_COMMAND_MIN;
}

void MCAF_UiStepIsr(volatile MCAF_UI_DATA *pui)
{
    pui->isrCount++;
}

void MCAF_UiStepMain(volatile MCAF_UI_DATA *pui)
{
    if (pui->isrCount >= MCAF_UI_LOOP_TIME_IN_ISRS)
    {
        pui->isrCount = 0; /* reset counter to 0 to restart */

        uint16_t leds = MCAF_UiCalculateIndicatorState(&pui->indicatorState);
        if (leds & MCAF_UI_LED1_ON)
        {
            HAL_LedGp1_Activate();
        }
        else
        {
            HAL_LedGp1_Deactivate();
        }
        if (leds & MCAF_UI_LED2_ON)
        {
            HAL_LedGp2_Activate();
        }
        else
        {
            HAL_LedGp2_Deactivate();
        }

    }
}

/**
 * Updates the LEDs when there is an error, with error-code blink behavior.
 * @param p indicator state
 * @return bitmask containing LED flags
 */
inline uint16_t MCAF_UiUpdateIndicatorErrorCode(volatile MCAF_UI_INDICATOR_STATE *p)
{
    /*
     * Display error code flash pattern, based around 1/2 period blinks.
     * We work in base 4, display 1/2/3/4 blinks per digit,
     * pauses between digits and a long pause at end.
     */

    /* At the beginning of each tick period, update what we're doing. */
    if (p->tickCounter == 0)
    {
        if (p->pause > 0)
        {
            /*
             * When we are pausing,
             * - turn LEDs off the entire time,
             * - wait until the end of the period
             * - decrement pause counter
             * - when we are done pausing:
             *     if we're all done displaying,
             *       start a long pause and then start over
             *     otherwise go to next digit
             */
            --p->pause;
        }
        else
        {
            /*
             * at end of tick period,
             * pause after we have displayed N+1 blinks
             * where N is the least significant digit in base 4
             */
            if (p->codeDigit == 0)
            {
                p->pause = MCUI_PAUSE_TIME_END_OF_CODE;
                p->codeDigit = p->code;
                /*
                 * if we're all done displaying,
                 * start a long pause and then start over
                 */
            }
            else if ((p->codeDigit & 0x03) == 0)
            {
                p->pause = MCUI_PAUSE_TIME_END_OF_DIGIT;
                p->codeDigit >>= 2;
                /* at the end of each digit, go to the next and pause */
            }
            else
            {
                --p->codeDigit;
            }
        }
    }

    /*
     * if not pausing:
     * blink LEDs on at 50% duty cycle
     */
    if ((p->pause == 0)
            && (p->tickCounter < (p->tickPeriod >> 1)))
    {
        return MCAF_UI_BOTH_LEDS_ON;
    }            
    else
    {
        return 0;
    }
}

/**
 * Updates the LEDs when there is not an error, with duty-cycle blink behavior.
 * @param p indicator state
 * @return bitmask containing LED flags
 */
inline uint16_t MCAF_UiUpdateIndicatorNoErrorCode(volatile MCAF_UI_INDICATOR_STATE *p)
{
    uint16_t result = 0;
    if (p->tickCounter < p->duty1)
    {
        result |= MCAF_UI_LED1_ON;
    }
    if (p->tickCounter < p->duty2)
    {
        result |= MCAF_UI_LED2_ON;
    }
    return result;
}

uint16_t MCAF_UiCalculateIndicatorState(volatile MCAF_UI_INDICATOR_STATE *p)
{   
    /* update indicator */
    if (++p->tickCounter >= p->tickPeriod)
    {
        p->tickCounter = 0;
    }
    
    if (p->isError)
    {
        return MCAF_UiUpdateIndicatorErrorCode(p);
    }
    else /* no error */
    {
        return MCAF_UiUpdateIndicatorNoErrorCode(p);        
    }
}

/* inline applies to the use of this function in MCAF_UiFlashErrorCodeForever,
 * not from external callers */
inline void MCAF_UiSetupFlashErrorCode(volatile MCAF_UI_INDICATOR_STATE *pindstate, uint16_t code)
{
    pindstate->code = code;
    pindstate->codeDigit = 0;
    pindstate->isError = true;
    pindstate->pause = 0;
    pindstate->tickPeriod = MCUI_ERROR_CODE_TICK_PERIOD_UILOOPS;    
}

/* Reserve dedicated memory for an indicator in case of severe errors. */
static MCAF_UI_INDICATOR_STATE errorIndicatorState;
void __attribute__((naked, noreturn)) MCAF_UiFlashErrorCodeForever(uint16_t code)
{    
    __builtin_disable_interrupts();
    MCAF_UiSetupFlashErrorCode(&errorIndicatorState, code);
    HAL_PWM_Outputs_Disable();
    
    MCAF_CareForWatchdog();

    while (true)
    {
        uint16_t leds = MCAF_UiCalculateIndicatorState(&errorIndicatorState);
        if (leds & MCAF_UI_LED1_ON)
        {
            HAL_LedGp1_Activate();
        }
        else
        {
            HAL_LedGp1_Deactivate();
        }
        if (leds & MCAF_UI_LED2_ON)
        {
            HAL_LedGp2_Activate();
        }
        else
        {
            HAL_LedGp2_Deactivate();
        }    
      
        int i;
        for (i = 0; i < MCUI_ERROR_CODE_LOOP_DELAYS_PER_TICK; ++i)
        {
            MCAF_CareForWatchdog();
            __delay_us(MCUI_ERROR_CODE_LOOP_DELAY_MICROSECONDS);
        }
    }
}
