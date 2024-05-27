/**
 * isr.c
 * 
 * Interrupt Service Routine entry points
 * 
 * Component: main application
 */

/* ********************************************************************
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

#include <stdint.h>
#include "system_state.h"
#include "state_machine.h"
#include "diagnostics.h"
#include "hal.h"
#include "monitor.h"
#include "mcaf_watchdog.h"
#include "test_harness.h"
#include "ui.h"
#include "mcapi_internal.h"
#include "current_measure.h"
/**
 * motor state variables, accessed directly
 * 
 * The ISR does not receive any arguments so we need access to this somehow.
 * One alternative: we keep a private pointer here and provide an accessor
 * function (declared in an isr.h file), for example:
 * 
 * <code>
 * static MCAF_MOTOR_DATA *pmotor_state;
 * 
 * void MCAF_RegisterISRState(MCAF_MOTOR_DATA *pstate)
 * {
 *    pmotor_state = pstate;
 * }
 * 
 * // ISR now accesses *pmotor_state instead 
 * </code>
 */
extern MCAF_MOTOR_DATA motor;
/** system data, accessed directly */
extern MCAF_SYSTEM_DATA systemData;
/** watchdog state, accessed directly */
extern volatile MCAF_WATCHDOG_T watchdog;

/**
 * Executes tasks in the ISR for ADC interrupts.
 * 
 * This is the primary ISR for high-speed computation.
 * It occurs at the same frequency as the PWM waveforms,
 * and is triggered after the ADC acquisition completes.
 * 
 * GPIO test point output is activated during this ISR for timing purposes.
 */
void __attribute__((interrupt, auto_psv)) HAL_ADC_ISR(void)
{
    if (MCAF_AdcIsrPrologEnabled())
    {
        MCAPI_AdcIsrProlog();
    }
    HAL_TestpointGp1_Activate();
#ifdef MCAF_TEST_PROFILING   
    motor.testing.timestampReference = HAL_ProfilingCounter_Get();
#endif
#if MCAF_SINGLE_CHANNEL_SUPPORT    
    motor.currentMeasure.adcTriggerState = MCAF_SINGLE_CHANNEL_FIRST_TRIGGER;   // resetting the trigger state next time DC link current ISR is triggered                                     
#endif
    MCAF_SystemStateMachine_StepIsr(&motor); // data is read from ADC buffer every ISR
    HAL_ADC_InterruptFlag_Clear(); // interrupt flag must be cleared after data is read from ADC buffer
    MCAF_UiStepIsr(&motor.ui);
    MCAF_MonitorStepIsr(&motor);
    MCAF_WatchdogManageIsr(&watchdog);
    HAL_ADC_StepIsrCallback();
    MCAF_CalculateFilteredCurrent(&motor);
    MCAF_ApiServiceIsr(&motor);

    /* Test and diagnostics code are always the lowest-priority routine within 
     * this ISR; diagnostics code should always be last.
     */
    MCAF_TestHarnessStepIsr(&systemData.testing);
    MCAF_CaptureTimestamp(&motor.testing, MCTIMESTAMP_DIAGNOSTICS);
    MCAF_DiagnosticsStepIsr();
    MCAF_CaptureTimestamp(&motor.testing, MCTIMESTAMP_END_OF_ISR);
    HAL_TestpointGp1_Deactivate();
    if (MCAF_AdcIsrEpilogEnabled())
    {
        MCAPI_AdcIsrEpilog();
    }
}

#if MCAF_SINGLE_CHANNEL_SUPPORT   
void __attribute__((interrupt, auto_psv)) HAL_ADC_SINGLE_CHANNEL_ISR(void)
{
    MCAF_SingleChannelCurrentMeasure(&motor);
    
    ADC1_IndividualChannelInterruptFlagClear(MCAF_ADC_DCLINK_CURRENT);
}
#endif