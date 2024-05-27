/**
 * foc.h
 * 
 * Module to include field-oriented-control functions
 * 
 * Component: FOC
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

#ifndef __FOC_H
#define __FOC_H

#include <stdbool.h>
#include "system_state.h"
#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize FOC state variables
 * 
 * @param pmotor motor state
 * @return <code>true</code> if the function succeeds
 */
bool MCAF_FocInit(MCAF_MOTOR_DATA *pmotor);

/**
 * Executes forward-path signal calculations of field-oriented control.
 * Includes control loops and forward modulation (Park, Clarke, SVM/ZSM)
 * 
 * @param pmotor motor state
 */
void MCAF_FocStepIsrForwardPath(MCAF_MOTOR_DATA *pmotor);

/**
 * Executes feedback-path signal calculations of field-oriented control.
 * Includes Park, Clarke transforms, estimators, and sine/cosine calculations.
 * 
 * @param pmotor motor state 
 */
void MCAF_FocStepIsrFeedbackPath(MCAF_MOTOR_DATA *pmotor);

/**
 * Executes non-critical tasks towards the end of the ISR
 * 
 * @param pmotor motor state
 */
void MCAF_FocStepIsrNonCriticalTask(MCAF_MOTOR_DATA *pmotor);

/**
 * Executes required calculations for FOC in the main loop.
 * 
 * @param pmotor motor state.
 */
void MCAF_FocStepMain(MCAF_MOTOR_DATA *pmotor);

/**
 * Executes one step of offset calibration
 * 
 * @param pmotor motor data
 */
void MCAF_FocCalibrateCurrentOffsets(MCAF_MOTOR_DATA *pmotor);

/**
 * Reads the ADC channels for FOC.
 * 
 * This should occur at the beginning of the ADC ISR.
 * Reads the dedicated S&H for A- and B-phase currents,
 * then the mux'd ADC channel.
 * 
 * @param pmotor motor data structure
 */
void MCAF_FocReadADC(MCAF_MOTOR_DATA *pmotor);

/**
 * Initialize integrators at startup
 * 
 * @param pmotor motor data structure
 */
inline static void MCAF_FocInitializeIntegrators(MCAF_MOTOR_DATA *pmotor)
{
    /* zero out integrators */
    pmotor->idCtrl.integrator = 0;
    pmotor->iqCtrl.integrator = 0;
    pmotor->omegaCtrl.integrator = 0;    
}

/**
 * Reinitialize motor controllers on restart
 * (when exiting error / debug states)
 * @param pmotor motor data structure
 */
inline static void MCAF_FocRestart(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->initialization.ready = false;
    if (pmotor->initialization.faultLatchDelay == 0)
    {
        pmotor->initialization.faultLatchDelay = 1;
    }
}

/**
 * Returns whether the ADC initialization is ready
 *
 * @param pmotor motor data structure
 * @return true if ADC initialization is ready
 */
inline static bool MCAF_ADCInitializationReady(const MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->initialization.ready;
}

/**
 * Returns whether the HW overcurrent flag is valid
 * 
 * @param pmotor motor state data
 * @return true if HW overcurrent flag is valid
 */
inline static bool MCAF_OvercurrentHWFlagValid(const MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->initialization.faultLatchDelay == 0;
}
/**
 * Clears HW flag when it is possible to do so.
 * This requires a delay straddling one or more PWM cycles.
 * Call only once per PWM cycle.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_OvercurrentHWFlagAttemptClear(MCAF_MOTOR_DATA *pmotor)
{
    if (--pmotor->initialization.faultLatchDelay == 0)
    {
        /* Two step fault clearing process is required for EP devices. 
           Occurs during PWM cycle at which fault was detected
           Step 1: Disable fault mode latch
           Occurs at the next PWM period
           Step 2: Enable fault mode latch and clear fault interrupt flag
        */
        HAL_PWM_FaultStatus_Clear();
        HAL_PWM_FaultClearEnd();
    }    
}


#ifdef __cplusplus
}
#endif

#endif /* __FOC_H */
