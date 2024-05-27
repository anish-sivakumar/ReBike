/**
 * board_service.c
 * 
 * Provides hardware-independent board service components with interfaces
 * extending into the HAL.
 * 
 * Component: HAL
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

#include <stdbool.h>
#include "board_service.h"
#include "error_codes.h"
#include "ui.h"
#include "parameters/hal_params.h"
#include "parameters/timing_params.h"

/**
 * Initializes the state variables for a given button.
 * @param pButtonData pointer to a BOARD_BUTTON_DATA_T struct
 */
static void MCAF_ButtonInit(volatile BOARD_BUTTON_DATA_T *pButtonData)
{
    pButtonData->buttonState = BOARD_BUTTON_UNPRESSED;
    pButtonData->shortButtonPress = false;
    pButtonData->longButtonPress = false;
    pButtonData->counterC1 = 0;
}

/**
 * Runs the button handler to debounce button inputs and 
 * check for short and long press events.
 * @param pButtonData pointer to a BOARD_BUTTON_DATA_T struct
 * @param rawInput is the raw GPIO signal from the board switch
 */
static void MCAF_ButtonService(volatile BOARD_BUTTON_DATA_T *pButtonData, bool rawInput)
{   
    switch(pButtonData->buttonState)
    {
        default:
        case BOARD_BUTTON_UNPRESSED:
        pButtonData->counterC1++;
        if (rawInput)
        {
            if (pButtonData->counterC1 >= MCAF_BUTTON_DEBOUNCE_TIME)
            {
                pButtonData->buttonState = BOARD_BUTTON_PRESSED;
                pButtonData->counterC1 = 0;
            }
        }
        else
        {
            pButtonData->buttonState = BOARD_BUTTON_UNPRESSED;
            pButtonData->counterC1 = 0;
        }
        break;
        
        case BOARD_BUTTON_PRESSED:
        pButtonData->counterC1++;
        if (pButtonData->counterC1 >= MCAF_BUTTON_LONG_PRESS_TIME)
        {
            pButtonData->longButtonPress = true;
            pButtonData->buttonState = BOARD_BUTTON_LONGPRESS;
        }
        else if (!rawInput)
        {
            pButtonData->shortButtonPress = true;
            pButtonData->counterC1 = 0;
            pButtonData->buttonState = BOARD_BUTTON_UNPRESSED;
        }
        break;
        
        case BOARD_BUTTON_LONGPRESS:
        if (!rawInput)
        {
            pButtonData->counterC1 = 0;
            pButtonData->buttonState = BOARD_BUTTON_UNPRESSED;
        }
        break;
    }
}

void MCAF_BoardServiceInit(MCAF_BOARD_DATA *pboard)
{
    pboard->configComplete = true;
    pboard->runtimeState = HAL_BOARD_READY;
    pboard->isrCount = 0;
    MCAF_ButtonInit(&pboard->sw1);
    MCAF_ButtonInit(&pboard->sw2);
}

void MCAF_BoardServiceStepMain(MCAF_BOARD_DATA *pboard)
{
    
}

void MCAF_BoardServiceTasks(MCAF_BOARD_DATA *pboard)
{
    MCAF_ButtonService(&pboard->sw1, HAL_ButtonGp1RawInput());
    MCAF_ButtonService(&pboard->sw2, HAL_ButtonGp2RawInput());
}

bool MCAF_ButtonGp1_EventGet(const MCAF_BOARD_DATA *pboard)
{
    return pboard->sw1.shortButtonPress;
}

bool MCAF_ButtonGp2_EventGet(const MCAF_BOARD_DATA *pboard)
{
    if (HAL_hasTwoButtons())
    {
        return pboard->sw2.shortButtonPress;
    }
    else
    {
        return pboard->sw2.longButtonPress;
    }
}

void MCAF_ButtonGp1_EventClear(MCAF_BOARD_DATA *pboard)
{
    pboard->sw1.shortButtonPress = false;
}

void MCAF_ButtonGp2_EventClear(MCAF_BOARD_DATA *pboard)
{
    if (HAL_hasTwoButtons())
    {
        pboard->sw2.shortButtonPress = false;
    }
    else
    {
        pboard->sw2.longButtonPress = false;
    }
}

void MCAF_BootstrapChargeInit(MCAF_BOARD_DATA *pboard)
{
    pboard->bootstrapDutycycle[0] = 0;
    pboard->bootstrapDutycycle[1] = 0;
    pboard->bootstrapDutycycle[2] = 0;
    pboard->delayCount = 0;
    pboard->bootstrapState = 0;
}

static inline uint16_t minimumDutyCycleForBootstrapCharging(void)
{
    return HAL_PARAM_PWM_PERIOD_COUNTS - HAL_PARAM_MIN_LOWER_DUTY_COUNTS;
}

bool MCAF_BootstrapChargeStepIsr(MCAF_BOARD_DATA *pboard)
{
    bool returnState = false;
    
    switch(pboard->bootstrapState)
    {
        case MCBS_IDLE_START:
            /* Override high-side PWMx to LOW and set 
             * low-side PWMx to 0% duty cycle */
            HAL_PWM_Outputs_Disable();
            pboard->bootstrapDutycycle[0] = HAL_PARAM_PWM_PERIOD_COUNTS;
            pboard->bootstrapDutycycle[1] = HAL_PARAM_PWM_PERIOD_COUNTS;
            pboard->bootstrapDutycycle[2] = HAL_PARAM_PWM_PERIOD_COUNTS;
            HAL_PWM_UpperTransistorsOverride_Low();
            HAL_PWM_LowerTransistorsOverride_Disable();
            
            pboard->delayCount = MCAF_BOARD_BOOTSTRAP_INITIAL_DELAY;
            pboard->bootstrapState = MCBS_WAIT_INITIAL;
            break;
            
        case MCBS_WAIT_INITIAL:
            /* Wait for a preset duration of time before starting to
             * charge the Phase-A bootstrap */
            if (pboard->delayCount == 0)
            {   
                pboard->bootstrapState = MCBS_PHASE_A_SETUP_CHARGING;
            }
            break;

        case MCBS_PHASE_A_SETUP_CHARGING:
            pboard->bootstrapDutycycle[0] = minimumDutyCycleForBootstrapCharging();
            pboard->delayCount = MCAF_BOARD_BOOTSTRAP_PHASE_DELAY;
            pboard->bootstrapState = MCBS_PHASE_A_CHARGING;
            break;
            
        case MCBS_PHASE_A_CHARGING:
            /* Wait for a preset duration of time to let the bootstrap drive
             * charge the Phase-A bootstrap capacitor */
            if (pboard->delayCount == 0)
            {
                pboard->bootstrapState = MCBS_PHASE_B_SETUP_CHARGING;
            }
            break;
            
        case MCBS_PHASE_B_SETUP_CHARGING:
            pboard->bootstrapDutycycle[1] = minimumDutyCycleForBootstrapCharging();
            pboard->delayCount = MCAF_BOARD_BOOTSTRAP_PHASE_DELAY;
            pboard->bootstrapState = MCBS_PHASE_B_CHARGING;
            break;
            
        case MCBS_PHASE_B_CHARGING:
            /* Wait for a preset duration of time to let the bootstrap drive
             * charge the Phase-B bootstrap capacitor */
            if (pboard->delayCount == 0)
            {
                pboard->bootstrapState = MCBS_PHASE_C_SETUP_CHARGING;
            }
            break;

        case MCBS_PHASE_C_SETUP_CHARGING:
            pboard->bootstrapDutycycle[2] = minimumDutyCycleForBootstrapCharging();
            pboard->delayCount = MCAF_BOARD_BOOTSTRAP_PHASE_DELAY;
            pboard->bootstrapState = MCBS_PHASE_C_CHARGING;
            break;
            
        case MCBS_PHASE_C_CHARGING:
            /* Wait for a preset duration of time to let the bootstrap drive
             * charge the Phase-C bootstrap capacitor */
            if (pboard->delayCount == 0)
            {
                pboard->bootstrapState = MCBS_BOOTSTRAP_COMPLETE;
            }
            break;
            
        case MCBS_BOOTSTRAP_COMPLETE:
            /* Bootstap sequence is complete, wait in this state */
            returnState = true;
            break;
    }
    HAL_PWM_DutyCycle_Set(pboard->bootstrapDutycycle);
    
    if (pboard->delayCount > 0)
    {
        pboard->delayCount--;
    }
    
    return returnState;
}