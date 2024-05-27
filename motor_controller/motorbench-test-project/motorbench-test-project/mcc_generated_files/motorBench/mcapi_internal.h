/**
 * mcapi_internal.h
 * 
 * Internal/private interfaces used by MCAPI to interact with MCAF
 * 
 * Component: MCAPI
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

#ifndef __MCAPI_INTERNAL_H
#define __MCAPI_INTERNAL_H

#include <stdint.h>
#include <stdbool.h>
#include "mcapi_types.h"
#include "system_state.h"
#include "foc.h"
#include "util.h"
#include "state_machine.h"
#include "sat_PI.h"
#include "adc_compensation.h"
#include "ui.h"
#include "ui_types.h"
#include "fault_detect.h"
#include "error_codes.h"
#include "filter.h"
#include "parameters/mcapi_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Private function to clear the specified fault flags
 * @param pMotor motor data
 */
inline static void handleFaultFlags(MCAF_MOTOR_DATA *pMotor)
{
    volatile MCAPI_MOTOR_DATA *pApiData = &pMotor->apiData;
    const uint16_t faultsToClear = pApiData->faultClearFlags;
    pApiData->faultFlags = UTIL_ClearBits(pApiData->faultFlags, faultsToClear);
    pApiData->faultClearFlags = 0;
}

/**
 * private function to handle all fault conditions in MCAF
 * @param pMotor motor data
 */
inline static void handleFaults(MCAF_MOTOR_DATA *pMotor)
{
    volatile MCAPI_MOTOR_DATA *pApiData = &pMotor->apiData;
    const uint16_t errorCode = MCAF_UiGetErrorCode(&pMotor->ui.indicatorState);

    /* Read all active fault flags from MCAF */
    if (MCAF_IsOvercurrentFlagSet(&pMotor->faultDetect))
    {
        pApiData->faultFlags = UTIL_SetBits(pApiData->faultFlags,
                                            MCAPI_FAULT_FLAG_OVERCURRENT);
    }
    if (MCAF_IsUndervoltageFlagSet(&pMotor->faultDetect))
    {
        pApiData->faultFlags = UTIL_SetBits(pApiData->faultFlags,
                                            MCAPI_FAULT_FLAG_UNDERVOLTAGE);
    }
    if (MCAF_IsOvervoltageFlagSet(&pMotor->faultDetect))
    {
        pApiData->faultFlags = UTIL_SetBits(pApiData->faultFlags,
                                            MCAPI_FAULT_FLAG_OVERVOLTAGE);
    }
    if (MCAF_IsOvertemperatureFlagSet(&pMotor->faultDetect))
    {
        pApiData->faultFlags = UTIL_SetBits(pApiData->faultFlags,
                                            MCAPI_FAULT_FLAG_OVERTEMPERATURE);
    }
    if (MCAF_UiIsError(&pMotor->ui.indicatorState))
    {
        /* check for any fault other than over-current, 
         * over-voltage, under-voltage, and overtemperature faults */
        if ((errorCode != ERR_NO_ERROR)&&
            (errorCode != ERR_HW_OVERCURRENT)&&
            (errorCode != ERR_DCLINK_OVERVOLTAGE)&&
            (errorCode != ERR_DCLINK_UNDERVOLTAGE)&&
            (errorCode != ERR_OVERTEMPERATURE))
        {
            pApiData->faultFlags = UTIL_SetBits(pApiData->faultFlags,
                                                MCAPI_FAULT_FLAG_MOTOR_DRIVE);
        }
    }    
    
    /* Handle faults that are currently active and request MCAF to
     * exit its fault state if there are no active fault flags */
    handleFaultFlags(pMotor);
    if (pApiData->faultFlags == MCAPI_FAULT_FLAG_NO_FAULT)
    {
        MCAF_UiExitFaultState(&pMotor->ui);
    }
}

/**
 * Ensure that the speed is limited to at least velocityMinimum in magnitude
 * @param pApiData MCAPI data
 * @return speed limited to at least the specified minimum magnitude
 */
inline static MCAF_U_VELOCITY_MECH MCAF_private_limit_speed_command(const volatile MCAPI_MOTOR_DATA *pApiData)
{
    const MCAF_U_VELOCITY_MECH velocityReference = pApiData->velocityReference;
    MCAF_U_VELOCITY_MECH velocityCmd = 
                UTIL_LimitMinimumS16(UTIL_Abs16(velocityReference),
                                     pApiData->velocityMinimum);
    if (velocityReference < 0)
        velocityCmd = -velocityCmd;
    return velocityCmd;
}

/**
 * Private function that handles the User Interface related interactions 
 * between MCAF and MC API, which are motor start/stop and velocity reference.
 * @param pMotor motor data
 */
inline static void handleUI(MCAF_MOTOR_DATA *pMotor)
{
    volatile MCAPI_MOTOR_DATA *pApiData = &pMotor->apiData;
    
    /* handle start/stop motor requests from the API */
    if (pApiData->runMotorRequest)
    {
        pMotor->ui.run = true;
        pApiData->runMotorRequest = false;
    }
    if (pApiData->stopMotorRequest)
    {
        pMotor->ui.run = false;
        pApiData->stopMotorRequest = false;
    }
    
    if (pApiData->velocityReferencePrevious != pApiData->velocityReference)
    {
        /* determine if the direction of velocity reference changed */
        if (!UTIL_BothNegativeOrNonnegative(pApiData->velocityReferencePrevious,
                                           pApiData->velocityReference))
        {
            pMotor->ui.flags ^= MCAF_UI_REVERSE;
            pMotor->ui.flags |= MCAF_UI_DIRECTION_CHANGED;
        }
    }
    pApiData->velocityReferencePrevious = pApiData->velocityReference;
    
    pMotor->velocityControl.velocityCmdApi = MCAF_private_limit_speed_command(pApiData);
}

/**
 * Private function that reads motor status from MCAF and 
 * returns an abstracted MCAPI_MOTOR_STATE type of status value.
 * @param pMotormotor data
 * @return abstracted motor status
 */
inline static MCAPI_MOTOR_STATE determineMotorStatus(MCAF_MOTOR_DATA *pMotor)
{
    switch (pMotor->state)
    {
        case MCSM_STOPPED: 
            return MCAPI_MOTOR_STOPPED;
            
        case MCSM_STARTING:
            return MCAPI_MOTOR_STARTING;
            
        case MCSM_RUNNING:
            return MCAPI_MOTOR_RUNNING;
            
        case MCSM_STOPPING:
            return MCAPI_MOTOR_STOPPING;
            
        case MCSM_RESTART:
            return MCAPI_MOTOR_STOPPING;
            
        case MCSM_FAULT:
            return MCAPI_MOTOR_FAULT;
            
        case MCSM_TEST_DISABLE:
            return MCAPI_MOTOR_DIAGSTATE;
            
        case MCSM_TEST_ENABLE:
            return MCAPI_MOTOR_DIAGSTATE;
            
        case MCSM_TEST_RESTART:
            return MCAPI_MOTOR_DIAGSTATE;
            
        default:
            return MCAPI_MOTOR_DIAGSTATE;
    }
}

/**
 * Private function that updates the operating parameters of motor
 * @param pMotor motor data
 */
inline static void updateMotorOperatingParameters(MCAF_MOTOR_DATA *pMotor)
{
    volatile MCAPI_MOTOR_DATA *pApiData = &pMotor->apiData;
    
    pApiData->currentLimitIqLower = MCAF_CurrentLimitIqLowerGet(pMotor);
    pApiData->currentLimitIqUpper = MCAF_CurrentLimitIqUpperGet(pMotor);
    pApiData->dcLinkVoltage = MCAF_GetDcLinkVoltage(pMotor);
    pApiData->iqFiltered = pMotor->apiFeedback.iqFiltered;
    pApiData->isMagSquaredFiltered = pMotor->apiFeedback.isSquaredFiltered;
    pApiData->velocityMeasured = pMotor->omegaElectrical;
}

/**
 * Executes one step of the MCAPI interaction with MCAF state data.
 * This function is NOT intended to be called by the application.
 * @param pMotor pointer to motor data structure
 */
static inline void MCAF_ApiServiceIsr(MCAF_MOTOR_DATA *pMotor)
{
    volatile MCAPI_MOTOR_DATA *pApiData = &pMotor->apiData;
    if (!pApiData->apiBusy)
    {
        handleFaults(pMotor);
        handleUI(pMotor);
        pApiData->motorStatus = determineMotorStatus(pMotor);
        updateMotorOperatingParameters(pMotor);
    }
}

/**
 * Weak definition of Prolog ADC ISR user function
 * that can be used for purposes such as functional safety.
 */
void __attribute__((weak)) MCAPI_AdcIsrProlog(void);

/**
 * Weak definition of Epilog ADC ISR user function
 * that can be used for purposes such as functional safety.
 */
void __attribute__((weak)) MCAPI_AdcIsrEpilog(void);

/**
 * Initializes the filter state variables used for calculating
 * filtered values of motor currents for MCAPI usage.
 * @param pmotor motor data
 */
inline static void MCAF_InitializeCurrentFilter(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_LpfFilterInitx16(&pmotor->apiFeedback.isSquaredFiltState);
    MCAF_LpfFilterInitx16(&pmotor->apiFeedback.iqFiltState);
    pmotor->apiFeedback.isSquared = 0;
    pmotor->apiFeedback.isSquaredFiltered = 0;
    pmotor->apiFeedback.iqFiltered = 0;
    pmotor->apiFeedback.isSquaredFiltState.coeff = KFILTER_IS_SQUARED;
    pmotor->apiFeedback.iqFiltState.coeff = KFILTER_IQ;
}

/**
 * Executes one step of the filter routines that are used to calculate
 * filtered values of motor currents for MCAPI usage.
 * @param pmotor motor data
 */
inline static void MCAF_CalculateFilteredCurrent(MCAF_MOTOR_DATA *pmotor)
{
    MCAPI_FEEDBACK_SIGNALS *pApiFeedback = &pmotor->apiFeedback;
    
    pApiFeedback->iqFiltered = MCAF_LpfFilterx16(&pApiFeedback->iqFiltState,
                                                    pmotor->idq.q);
    pApiFeedback->isSquared = UTIL_SignedSqr(pmotor->idq.q) + 
                                    UTIL_SignedSqr(pmotor->idq.d);
    pApiFeedback->isSquaredFiltered = MCAF_LpfFilterx16(&pApiFeedback->isSquaredFiltState,
                                                        pApiFeedback->isSquared);
}

#ifdef __cplusplus
}
#endif

#endif /* __MCAPI_INTERNAL_H */