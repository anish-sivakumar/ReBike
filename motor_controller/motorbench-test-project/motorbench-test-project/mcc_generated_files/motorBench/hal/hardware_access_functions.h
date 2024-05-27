/**
 *   hardware_access_functions.h
 *
 *  This module provides hardware access function support.
 *
 *  Component: HAL
 */
 
/*
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
 *
 ******************************************************************************/
 
#ifndef __HAF_H
#define __HAF_H

#define MCC_MELODY

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "hardware_access_functions_types.h"
#include "hardware_access_functions_params.h"
#include "../parameters/hal_params.h"
#include "../parameters/options.h"

#ifdef MCC_MELODY
#include "adc/adc1.h"
#include "cmp/cmp1.h"
#include "adc/adc_features.h"
#include "system/clock.h"
#include "system/interrupt.h"
#include "system/pins.h"
#include "pwm_hs/pwm.h"
#include "pwm_hs/pwm_features.h"
#include "system/reset.h"
#include "system/reset_types.h"

#include "adc/adc1.h"
#include "pwm_hs/pwm.h"
#include "qei/qei1.h"
#include "timer/tmr1.h"
#include "timer/sccp1.h"
#include "uart/uart1.h"
#include "opa/opa1.h"
#include "opa/opa2.h"
#include "opa/opa3.h"

#include "system/system.h"
#include "system/system_types.h"
#include "system/traps.h"
#include "system/watchdog.h"

#else

#include "adc1.h"
#include "adc_module_features.h"
#include "clock.h"
#include "interrupt_manager.h"
#include "mcaf_button1.h"
#include "mcaf_button2.h"
#include "mcaf_led1.h"
#include "mcaf_led2.h"
#include "pin_manager.h"
#include "pwm.h"
#include "pwm_module_features.h"
#include "reset.h"
#include "reset_types.h"
#include "sccp1_tmr.h"
#include "system.h"
#include "system_types.h"
#include "tmr1.h"
#include "traps.h"
#include "uart1.h"
#include "watchdog.h"
#include "qei1.h"
#include "qei_temp_hal.h"
#include "opa.h"

#endif

#ifdef __cplusplus  // Provide C++ Compatibility
extern "C" {
#endif

// This is the instruction cycle frequency required for libpic30.h
#define FCY  CLOCK_InstructionFrequencyGet()

/**
  Section: ISR helper macros
 */
#define HAL_ADC_ISR                     _ADCAN15Interrupt
#define MCAF_ADC_CHANNEL_USED_FOR_ISR   MCAF_ADC_DCLINK_VOLTAGE

/** Interrupt priorities used in MCAF */
enum {
    MCAF_PRIORITY_ADC = 6,                /** Primary motor control ISR priority */
    MCAF_PRIORITY_ADC_SINGLECHANNEL = 5,  /** ISR priority for DC link current measurement(single-channel) */
    MCAF_PRIORITY_TMR = 4                 /** Periodic timer tick ISR priority */
};

/**
  Section: Hardware Access Functions
 */

/**
  Sub-section: PWM Module Access Functions
*/

/**
 * Enables the PWM module.
 * Summary: Enables the whole of the PWM module.
 * @example
 * <code>
 * HAL_PWM_ModuleEnable();
 * </code>
 */
inline static void HAL_PWM_ModuleEnable(void) { 
#ifdef MCC_MELODY
    MCC_PWM_Enable();
#else
    PWM_Enable();
#endif
}

/**
 * Sets the master period cycle for all PWM to synchronize.
 * @param period PWM period value, expressed as pwm max counts
 * @example
 * <code>
 * HAL_PWM_SetPeriodIdentical(3500);
 * </code>
 * Note the use of (period-1)
 * FRM DS70005320D formula,  PGxPER = (Fpgx_clk/Fpwm) - 1
 */
inline static void HAL_PWM_SetPeriodIdentical(uint16_t period)
{
#ifdef MCC_MELODY
    MCC_PWM_MasterPeriodSet(period-1);
#else
    PWM_MasterPeriodSet(period-1);
#endif    
}

/**
 * Sets identical PWM deadtime values for Centeraligned PWM mode on all three phases of Motor #1.
 * @param dt PWM deadtime value
 * @example
 * <code>
 * HAL_PWM_SetDeadtimeIdentical_Motor1(140);
 * </code>
 */
inline static void HAL_PWM_SetDeadtimeIdentical_Motor1(uint16_t dt)
{
#ifdef MCC_MELODY
    MCC_PWM_DeadTimeSet(MOTOR1_PHASE_A, dt);
    MCC_PWM_DeadTimeSet(MOTOR1_PHASE_B, dt);
    MCC_PWM_DeadTimeSet(MOTOR1_PHASE_C, dt);    
#else
    PWM_DeadTimeSet(MOTOR1_PHASE_A,dt);
    PWM_DeadTimeSet(MOTOR1_PHASE_B,dt);
    PWM_DeadTimeSet(MOTOR1_PHASE_C,dt);
#endif    
}

/**
 * Clears the PWM fault interrupt status for Motor #1.
 * @example
 * <code>
 * HAL_PWM_FaultStatus_Clear();
 * </code>
 */
inline static void HAL_PWM_FaultStatus_Clear(void)
{
#ifdef MCC_MELODY
    MCC_PWM_GeneratorEventStatusClear(MOTOR1_PHASE_A, PWM_GENERATOR_INTERRUPT_FAULT);
    MCC_PWM_GeneratorEventStatusClear(MOTOR1_PHASE_B, PWM_GENERATOR_INTERRUPT_FAULT);
    MCC_PWM_GeneratorEventStatusClear(MOTOR1_PHASE_C, PWM_GENERATOR_INTERRUPT_FAULT);
#else
    PWM_GeneratorEventStatusClear(MOTOR1_PHASE_A,PWM_GENERATOR_INTERRUPT_FAULT);
    PWM_GeneratorEventStatusClear(MOTOR1_PHASE_B,PWM_GENERATOR_INTERRUPT_FAULT);
    PWM_GeneratorEventStatusClear(MOTOR1_PHASE_C,PWM_GENERATOR_INTERRUPT_FAULT);
#endif    
}

/**
 * Gets the status of PWM fault interrupt for Motor #1.
 * @example
 * <code>
 * HAL_PWM_FaultStatus_Get();
 * </code>
 */
inline static bool HAL_PWM_FaultStatus_Get(void)
{
#ifdef MCC_MELODY
    return MCC_PWM_GeneratorEventStatusGet(MOTOR1_PHASE_A,PWM_GENERATOR_INTERRUPT_FAULT);
#else
    return PWM_GeneratorEventStatusGet(MOTOR1_PHASE_A,PWM_GENERATOR_INTERRUPT_FAULT);
#endif    
}

/**
 * Sets up the trigger designated to initiate sampling of analog channels. The trigger is
 * set such that the sampling of motor currents occurs at the center of the low side pulse.
 * @example
 * <code>
 * HAL_PWM_SetADCTrigger();
 * </code>
 */
inline static void HAL_PWM_SetADCTrigger()
{
#ifdef MCC_MELODY
    MCC_PWM_TriggerACompareValueSet(MOTOR1_PHASE_A,
                                (HAL_PARAM_DEADTIME_COUNTS >> 1) + HAL_PARAM_ADC_TRIGGER_DELAY);
#else
    PWM_TriggerACompareValueSet(MOTOR1_PHASE_A,
                                (HAL_PARAM_DEADTIME_COUNTS >> 1) + HAL_PARAM_ADC_TRIGGER_DELAY);
#endif
}

/**
 * Disables PWM override on the three low-side transistors for Motor #1.
 * Summary: Disables PWM override on the three low-side transistors for Motor #1.
 * @example
 * <code>
 * HAL_PWM_LowerTransistorsOverride_Disable();
 * </code>
 */
inline static void HAL_PWM_LowerTransistorsOverride_Disable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_OverrideLowDisable(MOTOR1_PHASE_A);
    MCC_PWM_OverrideLowDisable(MOTOR1_PHASE_B);
    MCC_PWM_OverrideLowDisable(MOTOR1_PHASE_C);
#else
    PWM_OverrideLowDisable(MOTOR1_PHASE_A);
    PWM_OverrideLowDisable(MOTOR1_PHASE_B);
    PWM_OverrideLowDisable(MOTOR1_PHASE_C);
#endif
}

/**
 * Enables PWM override to state LOW on the three low-side transistors for Motor #1.
 * Summary: Enables PWM override to state LOW on the three low-side transistors for Motor #1.
 * @example
 * <code>
 * HAL_PWM_LowerTransistorsOverride_Low();
 * </code>
 */
inline static void HAL_PWM_LowerTransistorsOverride_Low(void)
{
#ifdef MCC_MELODY
    /* Set PWM override data to 0b00 */
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_A,0);
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_B,0);
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_C,0);
    
    /* Enable PWM override */    
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_A);
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_B);
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_C);
#else
    /* Set PWM override data to 0b00 */
    PWM_OverrideDataSet(MOTOR1_PHASE_A,0);
    PWM_OverrideDataSet(MOTOR1_PHASE_B,0);
    PWM_OverrideDataSet(MOTOR1_PHASE_C,0);
    
    /* Enable PWM override */    
    PWM_OverrideLowEnable(MOTOR1_PHASE_A);
    PWM_OverrideLowEnable(MOTOR1_PHASE_B);
    PWM_OverrideLowEnable(MOTOR1_PHASE_C);
#endif    
}

    
/**
 * Disables PWM override on the three high-side transistors for Motor #1.
 * Summary: Disables PWM override on the three high-side transistors for Motor #1.
 * @example
 * <code>
 * HAL_PWM_UpperTransistorsOverride_Disable();
 * </code>
 */
inline static void HAL_PWM_UpperTransistorsOverride_Disable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_OverrideHighDisable(MOTOR1_PHASE_A);
    MCC_PWM_OverrideHighDisable(MOTOR1_PHASE_B);
    MCC_PWM_OverrideHighDisable(MOTOR1_PHASE_C);
#else
    PWM_OverrideHighDisable(MOTOR1_PHASE_A);
    PWM_OverrideHighDisable(MOTOR1_PHASE_B);
    PWM_OverrideHighDisable(MOTOR1_PHASE_C);
#endif
}

/**
 * Enables PWM override to state LOW on the three high-side transistors for Motor #1.
 * Summary: Enables PWM override to state LOW on the three high-side transistors for Motor #1.
 * @example
 * <code>
 * HAL_PWM_UpperTransistorsOverride_Low();
 * </code>
 */
inline static void HAL_PWM_UpperTransistorsOverride_Low(void)
{
#ifdef MCC_MELODY
    /* Set PWM override data to 0b00 */
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_A,0);
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_B,0);
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_C,0);

    /* Enable PWM override */
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_A);
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_B);
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_C);
#else
    /* Set PWM override data to 0b00 */
    PWM_OverrideDataSet(MOTOR1_PHASE_A,0);
    PWM_OverrideDataSet(MOTOR1_PHASE_B,0);
    PWM_OverrideDataSet(MOTOR1_PHASE_C,0);

    /* Enable PWM override */
    PWM_OverrideHighEnable(MOTOR1_PHASE_A);
    PWM_OverrideHighEnable(MOTOR1_PHASE_B);
    PWM_OverrideHighEnable(MOTOR1_PHASE_C);
#endif
}

/**
 * Sets identical duty cycle values on three phases of Motor #1.
 * Summary: Sets identical duty cycle values on three phases of Motor #1.
 * @param dc Duty cycle value
 * @example
 * <code>
 * HAL_PWM_DutyCycle_SetIdentical(500);
 * </code>
 */
inline static void HAL_PWM_DutyCycle_SetIdentical(uint16_t dc)
{
#ifdef MCC_MELODY
    #if MCAF_SINGLE_CHANNEL_SUPPORT 
    MCC_PWM_PhaseSet(MOTOR1_PHASE_A,dc);
    MCC_PWM_PhaseSet(MOTOR1_PHASE_B,dc);
    MCC_PWM_PhaseSet(MOTOR1_PHASE_C,dc);
    #endif
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_A,dc);
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_B,dc);
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_C,dc);
#else
    #if MCAF_SINGLE_CHANNEL_SUPPORT 
    PWM_PhaseSet(MOTOR1_PHASE_A,dc);
    PWM_PhaseSet(MOTOR1_PHASE_B,dc);
    PWM_PhaseSet(MOTOR1_PHASE_C,dc);
    #endif
    PWM_DutyCycleSet(MOTOR1_PHASE_A,dc);
    PWM_DutyCycleSet(MOTOR1_PHASE_B,dc);
    PWM_DutyCycleSet(MOTOR1_PHASE_C,dc);
#endif
}

/**
 * Writes three unique duty cycle values to the PWM duty cycle registers
 * corresponding to Motor #1.
 * Summary: Writes to the PWM duty cycle registers corresponding to Motor #1.
 * @param pdc Pointer to the array that holds duty cycle values
 * @example
 * <code>
 * HAL_PWM_DutyCycleRegister_Set(&pdcMotor1);
 * </code>
 */
inline static void HAL_PWM_DutyCycleRegister_Set(const uint16_t *pdc)
{
#ifdef MCC_MELODY
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_A,pdc[0]);
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_B,pdc[1]);
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_C,pdc[2]);
#else
    PWM_DutyCycleSet(MOTOR1_PHASE_A,pdc[0]);
    PWM_DutyCycleSet(MOTOR1_PHASE_B,pdc[1]);
    PWM_DutyCycleSet(MOTOR1_PHASE_C,pdc[2]);
#endif
}

/**
 * Writes three unique phase values to the PWM phase registers
 * corresponding to Motor #1.
 * Summary: Writes to the PWM phase registers corresponding to Motor #1.
 * @param phase Pointer to the array that holds phase values
 * @example
 * <code>
 * HAL_PWM_PhaseRegister_Set(&phaseMotor1);
 * </code>
 */
inline static void HAL_PWM_PhaseRegister_Set(const uint16_t *phase)
{
#ifdef MCC_MELODY
    MCC_PWM_PhaseSet(MOTOR1_PHASE_A,phase[0]);
    MCC_PWM_PhaseSet(MOTOR1_PHASE_B,phase[1]);
    MCC_PWM_PhaseSet(MOTOR1_PHASE_C,phase[2]);
#else
    PWM_PhaseSet(MOTOR1_PHASE_A,phase[0]);
    PWM_PhaseSet(MOTOR1_PHASE_B,phase[1]);
    PWM_PhaseSet(MOTOR1_PHASE_C,phase[2]);
#endif
}

/**
 * Writes three unique duty cycle values to both halves of the PWM duty cycles corresponding to Motor #1.
 * Summary: Writes to both halves of the PWM duty cycles corresponding to Motor #1.
 * @param pdc Pointer to the array that holds duty cycle values
 * @example
 * <code>
 * HAL_PWM_DutyCycle_Set(*pdc);
 * </code>
 */
inline static void HAL_PWM_DutyCycle_Set(const uint16_t *pdc)
{
    #if MCAF_SINGLE_CHANNEL_SUPPORT
    HAL_PWM_PhaseRegister_Set(pdc);
    #endif 
    HAL_PWM_DutyCycleRegister_Set(pdc);
}

/**
 * Writes six unique duty cycle values to the first half and second half of PWM duty cycles
 * corresponding to Motor #1.
 * Summary: Writes to the first-half and second-half of PWM duty cycles corresponding to Motor #1.
 * @param firstHalf Pointer to the array that holds information to set the first half of PWM duty cycle.
 * @param secondHalf Pointer to the array that holds information to set the second half of PWM duty cycle.
 * @example
 * <code>
 * HAL_PWM_DutyCycleDualEdge_Set(&firstHalf, &secondHalf);
 * </code>
 */
inline static void HAL_PWM_DutyCycleDualEdge_Set(const uint16_t *firstHalf, const uint16_t *secondHalf)
{
    HAL_PWM_PhaseRegister_Set(firstHalf);
    HAL_PWM_DutyCycleRegister_Set(secondHalf);
}

/**
* Begins the PWM fault clearing process for all PWM instances 
* Summary: Begins fault clearing process.
* <code>
* HAL_PWM_FaultClearBegin();
* </code>
*/
inline static void HAL_PWM_FaultClearBegin(void)
{
#ifdef MCC_MELODY
    #if (PWM_FAULT_LATCH_SOFTWARE_CLEAR_FEATURE_AVAILABLE)
    {
        MCC_PWM_FaultModeLatchClear(MOTOR1_PHASE_A);
        MCC_PWM_FaultModeLatchClear(MOTOR1_PHASE_B);
        MCC_PWM_FaultModeLatchClear(MOTOR1_PHASE_C);
    }
    #else
    {
        MCC_PWM_FaultModeLatchDisable(MOTOR1_PHASE_A);
        MCC_PWM_FaultModeLatchDisable(MOTOR1_PHASE_B);
        MCC_PWM_FaultModeLatchDisable(MOTOR1_PHASE_C);
    }
    #endif
#else
    #if (PWM_FAULT_LATCH_SOFTWARE_CLEAR_FEATURE_AVAILABLE)
    {
        PWM_FaultModeLatchClear(MOTOR1_PHASE_A);
        PWM_FaultModeLatchClear(MOTOR1_PHASE_B);
        PWM_FaultModeLatchClear(MOTOR1_PHASE_C);
    }
    #else
    {
        PWM_FaultModeLatchDisable(MOTOR1_PHASE_A);
        PWM_FaultModeLatchDisable(MOTOR1_PHASE_B);
        PWM_FaultModeLatchDisable(MOTOR1_PHASE_C);
    }
    #endif
#endif
}

/**
* Ends the PWM fault clearing process for all PWM instances 
* Summary: Ends fault clearing process.
* <code>
* HAL_PWM_FaultClearEnd();
* </code>
*/
inline static void HAL_PWM_FaultClearEnd(void)
{
#ifdef MCC_MELODY
    #if (PWM_FAULT_LATCH_SOFTWARE_CLEAR_FEATURE_AVAILABLE)
    {
        // no action required
    }
    #else
    {
        MCC_PWM_FaultModeLatchEnable(MOTOR1_PHASE_A);
        MCC_PWM_FaultModeLatchEnable(MOTOR1_PHASE_B);
        MCC_PWM_FaultModeLatchEnable(MOTOR1_PHASE_C);
    }
    #endif
#else
    #if (PWM_FAULT_LATCH_SOFTWARE_CLEAR_FEATURE_AVAILABLE)
    {
        // no action required
    }
    #else
    {
        PWM_FaultModeLatchEnable(MOTOR1_PHASE_A);
        PWM_FaultModeLatchEnable(MOTOR1_PHASE_B);
        PWM_FaultModeLatchEnable(MOTOR1_PHASE_C);
    }
    #endif
#endif
}


/**
 * Maintains the Motor #1 low-side transistors at the requested duty cycle while
 * keeping high-side transistors OFF.
 * Summary: Maintains the Motor #1 PWM outputs to an idle state with minimal impact to the motor.
 * @param pwmPeriodCount PWM period count
 * @param dc Duty cycle value for the low-side transistors
 * @example
 * <code>
 * HAL_PWM_LowerTransistorsDutyCycle_Set(3500,220);
 * </code>
 */
inline static void HAL_PWM_LowerTransistorsDutyCycle_Set(uint16_t pwmPeriodCount, uint16_t dc)
{
    HAL_PWM_UpperTransistorsOverride_Low();

    uint16_t dutyCycleLowSide = pwmPeriodCount;
    dutyCycleLowSide -= dc;

    HAL_PWM_DutyCycle_SetIdentical(dutyCycleLowSide);
}

/**
 * Disable the PWM channels assigned for Motor #1 by overriding them to low state.
 * Summary: Disable the PWM channels assigned for Motor #1 by overriding them to low state.
 * @example
 * <code>
 * HAL_PWM_Outputs_Disable();
 * </code>
 */
inline static void HAL_PWM_Outputs_Disable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_A,0);
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_B,0);
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_C,0);   
    
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_A);
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_A);    
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_B);
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_B);    
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_C);
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_C);    
#else
    PWM_OverrideDataSet(MOTOR1_PHASE_A,0);
    PWM_OverrideDataSet(MOTOR1_PHASE_B,0);
    PWM_OverrideDataSet(MOTOR1_PHASE_C,0);   
    
    PWM_OverrideHighEnable(MOTOR1_PHASE_A);
    PWM_OverrideLowEnable(MOTOR1_PHASE_A);    
    PWM_OverrideHighEnable(MOTOR1_PHASE_B);
    PWM_OverrideLowEnable(MOTOR1_PHASE_B);    
    PWM_OverrideHighEnable(MOTOR1_PHASE_C);
    PWM_OverrideLowEnable(MOTOR1_PHASE_C);    
#endif
}

/**
 * Disable the PWM channels assigned for Phase-A of Motor #1 by overriding them to low state.
 * Summary: Disable the PWM channels assigned for Phase-A of Motor #1 by overriding them to low state.
 * @example
 * <code>
 * HAL_PWM_PhaseAOutput_Disable();
 * </code>
 */
inline static void HAL_PWM_PhaseAOutput_Disable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_A,0);  
    
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_A);
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_A);
#else
    PWM_OverrideDataSet(MOTOR1_PHASE_A,0);  
    
    PWM_OverrideHighEnable(MOTOR1_PHASE_A);
    PWM_OverrideLowEnable(MOTOR1_PHASE_A);
#endif    
}

/**
 * Enable the PWM channels assigned for Phase-A of Motor #1.
 * Summary: Enable the PWM channels assigned for Phase-A of Motor #1.
 * @example
 * <code>
 * HAL_PWM_PhaseAOutput_Enable();
 * </code>
 */
inline static void HAL_PWM_PhaseAOutput_Enable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_OverrideHighDisable(MOTOR1_PHASE_A);
    MCC_PWM_OverrideLowDisable(MOTOR1_PHASE_A);
#else
    PWM_OverrideHighDisable(MOTOR1_PHASE_A);
    PWM_OverrideLowDisable(MOTOR1_PHASE_A);
#endif
}

/**
 * Disable the PWM channels assigned for Phase-B of Motor #1 by overriding them to low state.
 * Summary: Disable the PWM channels assigned for Phase-B of Motor #1 by overriding them to low state.
 * @example
 * <code>
 * HAL_PWM_PhaseBOutput_Disable();
 * </code>
 */
inline static void HAL_PWM_PhaseBOutput_Disable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_B,0);
    
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_B);
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_B);
#else
    PWM_OverrideDataSet(MOTOR1_PHASE_B,0);
    
    PWM_OverrideHighEnable(MOTOR1_PHASE_B);
    PWM_OverrideLowEnable(MOTOR1_PHASE_B);
#endif
}

/**
 * Enable the PWM channels assigned for Phase-B of Motor #1.
 * Summary: Enable the PWM channels assigned for Phase-B of Motor #1.
 * @example
 * <code>
 * HAL_PWM_PhaseBOutput_Enable();
 * </code>
 */
inline static void HAL_PWM_PhaseBOutput_Enable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_OverrideHighDisable(MOTOR1_PHASE_B);
    MCC_PWM_OverrideHighDisable(MOTOR1_PHASE_B);
#else
    PWM_OverrideHighDisable(MOTOR1_PHASE_B);
    PWM_OverrideLowDisable(MOTOR1_PHASE_B);
#endif
}

/**
 * Disable the PWM channels assigned for Phase-C of Motor #1 by overriding them to low state.
 * Summary: Disable the PWM channels assigned for Phase-C of Motor #1 by overriding them to low state.
 * @example
 * <code>
 * HAL_PWM_PhaseCOutput_Disable();
 * </code>
 */
inline static void HAL_PWM_PhaseCOutput_Disable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_C,0);
    
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_C);
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_C);
#else
    PWM_OverrideDataSet(MOTOR1_PHASE_C,0);
    
    PWM_OverrideHighEnable(MOTOR1_PHASE_C);
    PWM_OverrideLowEnable(MOTOR1_PHASE_C);
#endif
}

/**
 * Enable the PWM channels assigned for Phase-C of Motor #1.
 * Summary: Enable the PWM channels assigned for Phase-C of Motor #1.
 * @example
 * <code>
 * HAL_PWM_PhaseCOutput_Enable();
 * </code>
 */
inline static void HAL_PWM_PhaseCOutput_Enable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_OverrideHighDisable(MOTOR1_PHASE_C);
    MCC_PWM_OverrideLowDisable(MOTOR1_PHASE_C);
#else
    PWM_OverrideHighDisable(MOTOR1_PHASE_C);
    PWM_OverrideLowDisable(MOTOR1_PHASE_C);
#endif
}

/**
 * Enables the PWM channels assigned for Motor #1 by disabling the PWM override function.
 * Summary: Enables the PWM channels assigned for Motor #1 by disabling the PWM override function.
 * @example
 * <code>
 * HAL_PWM_Outputs_Enable();
 * </code>
 */
inline static void HAL_PWM_Outputs_Enable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_OverrideHighDisable(MOTOR1_PHASE_A);
    MCC_PWM_OverrideLowDisable(MOTOR1_PHASE_A);

    MCC_PWM_OverrideHighDisable(MOTOR1_PHASE_B);
    MCC_PWM_OverrideLowDisable(MOTOR1_PHASE_B);
    
    MCC_PWM_OverrideHighDisable(MOTOR1_PHASE_C);
    MCC_PWM_OverrideLowDisable(MOTOR1_PHASE_C);
#else
    PWM_OverrideHighDisable(MOTOR1_PHASE_A);
    PWM_OverrideLowDisable(MOTOR1_PHASE_A);

    PWM_OverrideHighDisable(MOTOR1_PHASE_B);
    PWM_OverrideLowDisable(MOTOR1_PHASE_B);
    
    PWM_OverrideHighDisable(MOTOR1_PHASE_C);
    PWM_OverrideLowDisable(MOTOR1_PHASE_C);
#endif
}

/**
  Sub-section: GPIO Module Access Functions
*/

void HAL_Initialize(void);

/**
 * This function is generated and returns whether or not a development board has two buttons.
 * @return true = two buttons are present, false = two buttons are not present
 */
inline static bool HAL_hasTwoButtons(void)
{
    return true;
}

/**
 * Activates LED-GP1.
 * Summary: Activates LED-GP1.
 * @example
 * <code>
 * HAL_LedGp1_Activate();
 * </code>
 */
inline static void HAL_LedGp1_Activate(void)
{
#ifdef MCC_MELODY
    MCAF_LED1_SetHigh();
#else
    MCAF_LED1_On();
#endif    
}

/**
 * Deactivates LED-GP1.
 * Summary: Deactivates LED-GP1.
 * @example
 * <code>
 * HAL_LedGp1_Deactivate();
 * </code>
 */
inline static void HAL_LedGp1_Deactivate(void)
{
#ifdef MCC_MELODY
    MCAF_LED1_SetLow();
#else
    MCAF_LED1_Off();
#endif    
}

/**
 * Activates LED-GP2.
 * Summary: Activates LED-GP2.
 * @example
 * <code>
 * HAL_LedGp2_Activate();
 * </code>
 */
inline static void HAL_LedGp2_Activate(void)
{
#ifdef MCC_MELODY
    MCAF_LED2_SetHigh();
#else
    MCAF_LED2_On();
#endif  
}

/**
 * Deactivates LED-GP2.
 * Summary: Deactivates LED-GP2.
 * @example
 * <code>
 * HAL_LedGp2_Deactivate();
 * </code>
 */
inline static void HAL_LedGp2_Deactivate(void)
{
#ifdef MCC_MELODY
    MCAF_LED2_SetLow();
#else
    MCAF_LED2_Off();
#endif    
}

/**
 * Activates Testpoint-GP1.
 * Summary: Activates Testpoint-GP1.
 * @example
 * <code>
 * HAL_TestpointGp1_Activate();
 * </code>
 */
inline static void HAL_TestpointGp1_Activate(void) { MCAF_TESTPOINT1_SetHigh(); }

/**
 * Deactivates Testpoint-GP1.
 * Summary: Deactivates Testpoint-GP1.
 * @example
 * <code>
 * HAL_TestpointGp1_Deactivate();
 * </code>
 */
inline static void HAL_TestpointGp1_Deactivate(void) { MCAF_TESTPOINT1_SetLow(); }

/**
 * Get the raw GPIO input from button1.
 * @return bool raw GPIO state where true = button is pressed, false = button is not pressed
 */
inline static bool HAL_ButtonGp1RawInput(void)
{
#ifdef MCC_MELODY    
    return !MCAF_BUTTON1_GetValue();
#else
    return MCAF_BUTTON1_IsPressed(); 
#endif
}

/**
 * Get the raw GPIO input from button2.
 * @return bool raw GPIO state where true = button is pressed, false = button is not pressed
 */
inline static bool HAL_ButtonGp2RawInput(void)
{
#ifdef MCC_MELODY    
    return !MCAF_BUTTON2_GetValue();
#else
    return MCAF_BUTTON2_IsPressed();
#endif
}

/**
  Sub-section: ADC Module Access Functions
*/


/**
 * This routine is a call back function to be called every ADC ISR.
 */
void HAL_ADC_StepIsrCallback(void);

/**
 * Get the ADC channel number corresponding to the potentiometer analog input for motor #1.
 * @return ADC channel number
 */
inline static uint16_t HAL_ADC_ChannelPotentiometer(void) { return MCAF_ADC_POTENTIOMETER; }

/**
 * Get the ADC channel number corresponding to the DC link voltage sense analog input for motor #1.
 * @return ADC channel number
 */
inline static uint16_t HAL_ADC_ChannelDclink(void) { return MCAF_ADC_DCLINK_VOLTAGE; }

/**
 * Get the ADC channel number corresponding to phase current A analog input for motor #1.
 * @return ADC channel number
 */
inline static uint16_t HAL_ADC_ChannelIphaseA(void) { return MCAF_ADC_PHASEA_CURRENT; }

/**
 * Get the ADC channel number corresponding to phase current B analog input for motor #1.
 * @return ADC channel number
 */
inline static uint16_t HAL_ADC_ChannelIphaseB(void) { return MCAF_ADC_PHASEB_CURRENT; }

/**
 * @deprecated as of MCAF R6
 * Get the ADC channel number corresponding to sum phase/DC link current analog input for motor #1.
 * @return ADC channel number
 */
inline static uint16_t HAL_ADC_ChannelIphaseSum(void) { return 0; }

/**
 * Is the phase A current analog input available?
 * @return whether the phase A current analog input is available
 */
inline static bool HAL_ADC_IsAvailableIphaseA(void)
{
    return true;
}

/**
 * Get the ADC result corresponding to the phase A current analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValueIphaseA(void)
{ 
#ifdef MCC_MELODY
    return MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEA_CURRENT);
#else
    return ADC1_ConversionResultGet(MCAF_ADC_PHASEA_CURRENT);
#endif
}

/**
 * Is the phase B current analog input available?
 * @return whether the phase B current analog input is available
 */
inline static bool HAL_ADC_IsAvailableIphaseB(void)
{
    return true;
}

/**
 * Get the ADC result corresponding to the phase B current analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValueIphaseB(void)
{ 
#ifdef MCC_MELODY
    return MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEB_CURRENT);
#else
    return ADC1_ConversionResultGet(MCAF_ADC_PHASEB_CURRENT);
#endif
}

/**
 * Is the phase C current analog input available?
 * @return whether the phase C current analog input is available
 */
inline static bool HAL_ADC_IsAvailableIphaseC(void)
{
    return false;
}

/**
 * Get the ADC result corresponding to the phase C current analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValueIphaseC(void)
{ 
    return 0;
}

/**
 * Is the phase A voltage analog input available?
 * @return whether the phase A voltage analog input is available
 */
inline static bool HAL_ADC_IsAvailableVphaseA(void)
{
    return false;
}

/**
 * Get the ADC result corresponding to the phase A voltage analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValueVphaseA(void)
{ 
    return 0;
}

/**
 * Is the phase B voltage analog input available?
 * @return whether the phase B voltage analog input is available
 */
inline static bool HAL_ADC_IsAvailableVphaseB(void)
{
    return false;
}

/**
 * Get the ADC result corresponding to the phase B voltage analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValueVphaseB(void)
{ 
    return 0;
}

/**
 * Is the phase C voltage analog input available?
 * @return whether the phase C voltage analog input is available
 */
inline static bool HAL_ADC_IsAvailableVphaseC(void)
{
    return false;
}

/**
 * Get the ADC result corresponding to the phase C voltage analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValueVphaseC(void)
{ 
    return 0;
}

/**
 * Is the DC link current analog input available?
 * @return whether the DC link current analog input is available
 */
inline static bool HAL_ADC_IsAvailableIbus(void)
{
    return false;
}

/**
 * Get the ADC result corresponding to the DC link current analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValueIbus(void)
{ 
    return 0;
}

/**
 * Is the DC link voltage analog input available?
 * @return whether the DC link voltage analog input is available
 */
inline static bool HAL_ADC_IsAvailableDclink(void)
{
    return true;
}

/**
 * Get the ADC result corresponding to the DC link voltage analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValueDclink(void)
{ 
#ifdef MCC_MELODY
    return MCC_ADC_ConversionResultGet(MCAF_ADC_DCLINK_VOLTAGE);
#else
    return ADC1_ConversionResultGet(MCAF_ADC_DCLINK_VOLTAGE);
#endif
}

/**
 * Is the potentiometer voltage analog input available?
 * @return whether the potentiometer voltage analog input is available
 */
inline static bool HAL_ADC_IsAvailablePotentiometer(void)
{
    return true;
}

/**
 * Get the ADC result corresponding to the potentiometer voltage analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValuePotentiometer(void)
{ 
#ifdef MCC_MELODY
    return MCC_ADC_ConversionResultGet(MCAF_ADC_POTENTIOMETER);
#else
    return ADC1_ConversionResultGet(MCAF_ADC_POTENTIOMETER);
#endif
}

/**
 * Is the bridge temperature voltage analog input available?
 * @return whether the bridge temperature voltage analog input is available
 */
inline static bool HAL_ADC_IsAvailableBridgeTemperature(void)
{
    return false;
}

/**
 * Get the ADC result corresponding to the bridge temperature voltage analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValueBridgeTemperature(void)
{ 
    return 0;
}

/**
 * Is the absolute reference voltage analog input available?
 * @return whether the absolute reference voltage analog input is available
 */
inline static bool HAL_ADC_IsAvailableAbsoluteReferenceVoltage(void)
{
    return false;
}

/**
 * Get the ADC result corresponding to the absolute reference voltage analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValueAbsoluteReferenceVoltage(void)
{ 
    return 0;
}

/**
 * @deprecated as of MCAF R6
 * Get the ADC result corresponding to sum phase analog input for motor #1.
 * @return ADC result value
 */
inline static uint16_t HAL_ADC_ValueIphaseSum(void) { return 0; }

/**
* Enables the ADC module.
* Summary: Enables the whole of the ADC module.
* @example
* <code>
* HAL_ADC_Enable();
* </code>
*/
inline static void HAL_ADC_Enable(void)
{
#ifdef MCC_MELODY
    MCC_ADC_Enable();
#else
    ADC1_Enable();
#endif
}

/**
* Clears the ADC interrupt flag.
* @example
* <code>
* HAL_ADC_InterruptFlag_Clear();
* </code>
*/
inline static void HAL_ADC_InterruptFlag_Clear(void)
{
#ifdef MCC_MELODY
    #if (ADC_INDIVIDUAL_CHANNEL_INTERRUPT_FEATURE_AVAILABLE)
        // In order to clear the ADC ISR flag one must read the associated
        // ADC buffer. This dummy read is to ensure the ISR flag will be cleared.
        MCC_ADC_ConversionResultGet(MCAF_ADC_CHANNEL_USED_FOR_ISR);
        MCC_ADC_IndividualChannelInterruptFlagClear(MCAF_ADC_CHANNEL_USED_FOR_ISR);
    #else
        MCC_ADC_InterruptFlagClear();
    #endif
#else
    #if (ADC_INDIVIDUAL_CHANNEL_INTERRUPT_FEATURE_AVAILABLE)
        // In order to clear the ADC ISR flag one must read the associated
        // ADC buffer. This dummy read is to ensure the ISR flag will be cleared.
        ADC1_ConversionResultGet(MCAF_ADC_CHANNEL_USED_FOR_ISR);
        ADC1_IndividualChannelInterruptFlagClear(MCAF_ADC_CHANNEL_USED_FOR_ISR);
    #else
        ADC1_InterruptFlagClear();   
    #endif
#endif
}

/**
* Enables the ADC interrupt flag.
* @example
* <code>
* HAL_ADC_Interrupt_Enable();
* </code>
*/
inline static void HAL_ADC_Interrupt_Enable(void)
{
#ifdef MCC_MELODY
    #if (ADC_INDIVIDUAL_CHANNEL_INTERRUPT_FEATURE_AVAILABLE)
        MCC_ADC_IndividualChannelInterruptEnable(MCAF_ADC_CHANNEL_USED_FOR_ISR);
    #else
        MCC_ADC_InterruptEnable(); 
    #endif
#else
    #if (ADC_INDIVIDUAL_CHANNEL_INTERRUPT_FEATURE_AVAILABLE)
        ADC1_IndividualChannelInterruptEnable(MCAF_ADC_CHANNEL_USED_FOR_ISR);
    #else
        ADC1_InterruptEnable(); 
    #endif
#endif
}

/**
 * Sets the ADC interrupt priority value.
 */
inline static void HAL_ADC_IndividualChannelInterruptPrioritySet(void)
{
#ifdef MCC_MELODY
    MCC_ADC_IndividualChannelInterruptPrioritySet(MCAF_ADC_CHANNEL_USED_FOR_ISR, MCAF_PRIORITY_ADC);
#else
    // already configured in SYSTEM_Initialize()
#endif
}

/**
 * Converts a signed input to an unsigned value.
 * Adding 0x8000 to input maps -32k to 32k -> 0 to 65k.
 * @param input input to be converted to unsigned value
 * @return unsigned value
*/
inline static int16_t HAL_ADC_UnsignedFromSignedInput(int16_t input) { return input + 0x8000; }

/**
  Sub-section: Interrupt Module Access Functions
*/

/**
* Returns the interrupt vector number.
* @example
* <code>
* vecNum = HAL_InterruptVector_Get();
* </code>
*/
inline static uint16_t HAL_InterruptVector_Get(void) { return _VECNUM; }

/**
  Sub-section: DMA Module Access Functions
*/

/**
 * Handles the DMA Error trap and returns whether or not it was properly handled
 * @return true = DMA Error was handled, false = DMA Error was not handled
 */
bool HAL_DMA_ErrorHandler(void);

/**
  Sub-section: UART Module Access Functions
*/

/**
* Initializes the UART module
*/
inline static void HAL_UART_Initialize(void) 
{
#ifdef MCC_MELODY
    MCC_UART_Initialize();
#else
    UART1_Initialize();
#endif
}

/**
 * Writes data to the UART tx buffer
 * @param data data to be written to the buffer
 */
inline static void HAL_UART_Write(uint8_t data)
{
#ifdef MCC_MELODY
    MCC_UART_Write(data);
#else
    UART1_Write(data);
#endif
}

/**
 * Reads data from the UART rx buffer
 * @return data received from the UART rx buffer
 */
inline static uint8_t HAL_UART_Read(void)
{
#ifdef MCC_MELODY
    return MCC_UART_Read();
#else
    return UART1_Read();
#endif
}

/**
 * States whether or not the UART rx buffer contains data
 * @return true = UART rx buffer has data false = UART rx buffer does not have data
 */
inline static bool HAL_UART_IsRxReady(void)
{
#ifdef MCC_MELODY
    return MCC_UART_IsRxReady();
#else
    return UART1_IsRxReady();
#endif
}

/**
 * States whether or not the UART tx buffer is ready for more data
 * @return true = UART tx buffer can accept more data false = UART tx buffer can not accept more data
 */
inline static bool HAL_UART_IsTxReady(void)
{
#ifdef MCC_MELODY
    return MCC_UART_IsTxReady();
#else
    return UART1_IsTxReady();
#endif
}


/**
  Sub-section: Watchdog Module Access Functions
*/

/**
* Clears the Watchdog timer
*/
inline static void HAL_WATCHDOG_Timer_Clear(void) { WATCHDOG_TimerClear(); }

/**
* Enables the Watchdog timer
*/
inline static void HAL_WATCHDOG_Timer_Enable(void) { WATCHDOG_TimerSoftwareEnable(); }

/**
  Sub-section: CORCON Module Access Functions
*/

/**
 * Gets the CORCON register value
 * @return CORCON register value
 */
inline static uint16_t HAL_CORCON_RegisterValue_Get(void) { return SYSTEM_CORCONRegisterValueGet(); }

/**
 * Sets the value of the CORCON register
 * @param reg_value register value to be set for the CORCON register
 */
inline static void HAL_CORCON_RegisterValue_Set(uint16_t reg_value) { SYSTEM_CORCONRegisterValueSet(reg_value); }

/**
* Initializes the CORCON module
*/
inline static void HAL_CORCON_Initialize(void) { SYSTEM_CORCONModeOperatingSet(CORCON_MODE_ENABLEALLSATNORMAL_ROUNDBIASED); }

/**
  Sub-section: TMR Module Access Functions
*/

/**
* Starts the profiling timer used to time the various operations
*/
inline static void HAL_ProfilingCounter_Start(void) {
#ifdef MCC_MELODY
    MCC_TMR_PROFILE_Start();
#else
    SCCP1_TMR_Start(); 
#endif    
}

/**
 * Gets the timer value of the profiling timer
 * @return profiling timer counter value
 */
inline static uint16_t HAL_ProfilingCounter_Get(void)
{
#ifdef MCC_MELODY
    return MCC_TMR_PROFILE_Counter16BitGet();
#else
    return SCCP1_TMR_Counter16BitPrimaryGet();
#endif    
}

/**
 * Sets the MCC_TMR_TICK interrupt priority value.
 */
inline static void HAL_TMR_TICK_InterruptPrioritySet(void)
{
#ifdef MCC_MELODY
    MCC_TMR_TICK_InterruptPrioritySet(MCAF_PRIORITY_TMR);
#else
    // already configured in SYSTEM_Initialize()
#endif
}

/**
 * Overrides the default callback function for the application timer
 * interrupt.
 * @param callback function handle
 */
inline static void HAL_TMR_TICK_SetCallbackFunction(void (*handler)(void))
{
#ifdef MCC_MELODY
    MCC_TMR_TICK_TimeoutCallbackRegister(handler);
#else
    // no action required
#endif
}

/**
 * Starts the application timer.
 */
inline static void HAL_TMR_TICK_Start(void)
{
#ifdef MCC_MELODY
    MCC_TMR_TICK_Start();
#else
    TMR1_Start();
#endif
}

/**
  Sub-section: QEI Module Access Functions
*/

/**
* Initializes the QEI module
*/
inline static void HAL_QEI_Initialize(void)
{
#ifdef MCC_MELODY
    // already initialized in SYSTEM_Initialize()
#else
    QEI1_Initialize();
#endif
}

/**
* Initializes the QEI position capture mode. This is only used
* when an encoder index signal is present. 
*/
inline static void HAL_QEI_PositionCaptureInit(void)
{
#ifdef MCC_MELODY        
    MCC_QEI_CounterModeSet(QEI_MODE_FREE_RUNNING);
    MCC_QEI_PositionCaptureEnable();
    MCC_QEI_PositionCaptureSet(0);
#else
    QEI_FreeRunningCounterModeSet();
    QEI_PositionCaptureEventEnable();
    QEI_PositionCaptureInit();
#endif    
}

/**
 * Sets the QEI module range between 0 and (counts_per_rev - 1).
 * 
 */
inline static void HAL_QEI_CountsPerRevolutionSet(uint16_t counts_per_rev)
{
#ifdef MCC_MELODY            
    MCC_QEI_ModuloRangeSet(counts_per_rev);
#else    
    QEI1_ModuloMode16bitSet(counts_per_rev);
#endif
}

/**
 * Enables the QEI module.
 */
inline static void HAL_QEI_Enable(void)
{
#ifdef MCC_MELODY
    MCC_QEI_Enable();
#else
    QEI1_ModuleEnable();
#endif
    
}

/**
 * Reads the QEI position count.
 * @return position count value
 */
inline static uint16_t HAL_QEI_PositionCountGet(void)
{
#ifdef MCC_MELODY
    return MCC_QEI_PositionCount16bitRead();
#else
    return QEI1_PositionCount16bitRead();
#endif    
}

/**
 * Reads the QEI position capture value. In Capture mode, the input signal is
 * used to capture the contents of the position register into the QEIxIC register.
 * When QEIxIC is used for position capture, an Index match event (QCAPEN = 1)
 * or a Home event (HCAPEN = 1) causes the QEIxIC register to store a copy of
 * the current Position Counter contents.
 * @return position capture value
 */
inline static uint16_t HAL_QEI_PositionCaptureGet(void)
{
#ifdef MCC_MELODY
    return MCC_QEI_PositionCapture16bitGet();
#else
    return QEI_PositionCaptureGet();
#endif    
}


/**
 * Selects input voltage range for all op-amps.
 */
inline static void HAL_OpAmpsInputVoltageRangeSelect(void)
{
}


/**
 * Enables all op-amps
 */
inline static void HAL_OpAmpsEnable(void) {
#ifdef MCC_MELODY
    MCC_OPA_IA_Enable();
    MCC_OPA_IB_Enable();
    MCC_OPA_IDC_Enable();
#else
    OPA1_Enable();
    OPA2_Enable();
    OPA3_Enable();
    OPA_Enable();
#endif    
}

/**
  Sub-section: CMP/DAC Module Access Functions
*/

/**
 * Sets the DAC data value.
 */
inline static void HAL_CMP_SetComparatorOvercurrentThreshold(const uint16_t dacValue)
{
#ifdef MCC_MELODY
    MCC_CMP_DACDataWrite(dacValue);
#else
    // already configured in SYSTEM_Initialize()
#endif
}

/**
 * Initializes ADC signals
 */
inline static void HAL_ADC_SignalsInit(void)
{
    volatile uint8_t * const trig_sources = (volatile uint8_t *)&ADTRIG0L;
    
    enum {
        NO_TRIGGER = 0x00,
        PWM1_TRIGGER1 = 0x04,   // DS70005349H page 322
        PWM1_TRIGGER2 = 0x05    // DS70005349H page 322, trigger source used for reading single channel current
    };

    trig_sources[10] = NO_TRIGGER;  // MCAF_ADC_PHASEC_CURRENT
    trig_sources[22] = NO_TRIGGER;  // MCAF_ADC_PHASEC_VOLTAGE
    trig_sources[17] = NO_TRIGGER;  // MCAF_ADC_PHASEA_VOLTAGE
    trig_sources[23] = NO_TRIGGER;  // MCAF_ADC_PHASEB_VOLTAGE
    trig_sources[ 4] = NO_TRIGGER;  // MCAF_ADC_DCLINK_CURRENT
    trig_sources[12] = NO_TRIGGER;  // MCAF_ADC_BRIDGE_TEMPERATURE
    trig_sources[ 0] = PWM1_TRIGGER1;  // MCAF_ADC_PHASEA_CURRENT
    trig_sources[ 1] = PWM1_TRIGGER1;  // MCAF_ADC_PHASEB_CURRENT
    trig_sources[15] = PWM1_TRIGGER1;  // MCAF_ADC_DCLINK_VOLTAGE
    trig_sources[11] = PWM1_TRIGGER1;  // MCAF_ADC_POTENTIOMETER
}

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// Suppress warning for use of deprecated functions here

/**
 * Initializes ADC resolution
 */
inline static void HAL_ADC_ResolutionInit(void) {
    // 12-bit ADC
    const uint16_t resolution_id = 3;
#ifdef MCC_MELODY
    MCC_ADC_ResolutionSet(resolution_id);
#else    
    ADC1_Core0ResolutionModeSet(resolution_id);
    ADC1_Core1ResolutionModeSet(resolution_id);
    ADC1_SharedCoreResolutionModeSet(resolution_id);
#endif
}

#pragma GCC diagnostic warning "-Wdeprecated-declarations"

// Internal use only within this file: set PWM mode of a PWM generator from PG1-PG4
inline static void HAL_PWM_ModeSetGenerator(uint16_t generator, uint16_t mode)
{
    switch (generator)
    {
        case 1:
            PG1CONLbits.MODSEL = mode;
            break;
        case 2:
            PG2CONLbits.MODSEL = mode;
            break;
        case 3:
            PG3CONLbits.MODSEL = mode;
            break;
        case 4:
            PG4CONLbits.MODSEL = mode;
            break;
    }
}

/**
 * Set PWM mode bits
 *
 *
 * @param mode desired MODSEL value
 */
inline static void HAL_PWM_ModeSet(uint16_t mode)
{
#ifdef MCC_MELODY
    MCC_PWM_ModeSet(MOTOR1_PHASE_A, mode);
    MCC_PWM_ModeSet(MOTOR1_PHASE_B, mode);
    MCC_PWM_ModeSet(MOTOR1_PHASE_C, mode);
#else
    HAL_PWM_ModeSetGenerator(MOTOR1_PHASE_A, mode);
    HAL_PWM_ModeSetGenerator(MOTOR1_PHASE_B, mode);
    HAL_PWM_ModeSetGenerator(MOTOR1_PHASE_C, mode);
#endif    
}

/** Set PWM mode to center-aligned single update */
inline static void HAL_PWM_ModeSingleUpdate(void)
{    
    const uint16_t mode = 0x4;
    HAL_PWM_ModeSet(mode);
}

/** Set PWM mode to center-aligned double update */
inline static void HAL_PWM_ModeDoubleUpdate(void)
{
    const uint16_t mode = 0x5;
    HAL_PWM_ModeSet(mode);
}

/** Set PWM mode to dual edge center-aligned updated once per cycle*/
inline static void HAL_PWM_ModeDualEdgeSingleUpdate(void)
{
    const uint16_t mode = 0x6;
    HAL_PWM_ModeSet(mode);
}

// Internal use only within this file: set PWM master phase select of a PWM generator from PG1-PG4
inline static void HAL_PWM_MasterPhaseSelectGenerator(uint16_t generator, uint16_t select)
{
    switch (generator)
    {
        case 1:
            PG1CONHbits.MPHSEL = select;
            break;
        case 2:
            PG2CONHbits.MPHSEL = select;
            break;
        case 3:
            PG3CONHbits.MPHSEL = select;
            break;
        case 4:
            PG4CONHbits.MPHSEL = select;
            break;
    }
}

/** 
 * Controls PWM phase either from master or local data
 * @param select master(1) or local(0) phase control
 */
inline static void HAL_PWM_MasterPhaseSelect(uint16_t select)
{
#ifdef MCC_MELODY
    MCC_PWM_PhaseSelect(MOTOR1_PHASE_A, select);
    MCC_PWM_PhaseSelect(MOTOR1_PHASE_B, select);
    MCC_PWM_PhaseSelect(MOTOR1_PHASE_C, select);
#else
    HAL_PWM_MasterPhaseSelectGenerator(MOTOR1_PHASE_A, select);
    HAL_PWM_MasterPhaseSelectGenerator(MOTOR1_PHASE_B, select);
    HAL_PWM_MasterPhaseSelectGenerator(MOTOR1_PHASE_C, select);
#endif
}

/** Enable PWM phase control from master (common) register*/
inline static void HAL_PWM_SelectMasterPhase(void)
{
    HAL_PWM_MasterPhaseSelect(1);
}

/** Enable PWM phase control from local (independent) register */
inline static void HAL_PWM_SelectLocalPhase(void)
{
    HAL_PWM_MasterPhaseSelect(0);
}

/** Enable ADC Trigger 1 for compare event with PGxTRIGA */
inline static void HAL_PWM_ADCTrigger1AEnable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_Trigger1Enable(MOTOR1_PHASE_A, PWM_TRIGGER_COMPARE_A);
#else
    PG1EVTLbits.ADTR1EN1 = 1;
#endif
}

/** Enable ADC Trigger 2 for compare event with PGxTRIGB */
inline static void HAL_PWM_ADCTrigger2BEnable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_Trigger2Enable(MOTOR1_PHASE_A, PWM_TRIGGER_COMPARE_B);
#else
    PG1EVTHbits.ADTR2EN2 = 1;
#endif
}

/** Disable ADC Trigger 2 for compare event with PGxTRIGB */
inline static void HAL_PWM_ADCTrigger2BDisable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_Trigger2Disable(MOTOR1_PHASE_A, PWM_TRIGGER_COMPARE_B);
#else
    PG1EVTHbits.ADTR2EN2 = 0;
#endif
}

/** Enable ADC Trigger 2 for compare event with PGxTRIGC */
inline static void HAL_PWM_ADCTrigger2CEnable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_Trigger2Enable(MOTOR1_PHASE_A, PWM_TRIGGER_COMPARE_C);
#else
    PG1EVTHbits.ADTR2EN3 = 1;
#endif
}

/** Disable ADC Trigger 2 for compare event with PGxTRIGC */
inline static void HAL_PWM_ADCTrigger2CDisable(void)
{
#ifdef MCC_MELODY
    MCC_PWM_Trigger2Disable(MOTOR1_PHASE_A, PWM_TRIGGER_COMPARE_C);
#else
    PG1EVTHbits.ADTR2EN3 = 0;
#endif
}

/** Set Priority for Bus Current AN Interrupt */
inline static void HAL_ADC_BusCurrentInterruptPrioritySet(void)
{
    // no action required
}

inline static void HAL_PWM_SetADCDualTrigger(int16_t a, int16_t b)
{
#ifdef MCC_MELODY
    MCC_PWM_TriggerBCompareValueSet(MOTOR1_PHASE_A, a);
    MCC_PWM_TriggerCCompareValueSet(MOTOR1_PHASE_A, b);
#else
    PWM_TriggerBCompareValueSet(MOTOR1_PHASE_A, a);
    PWM_TriggerCCompareValueSet(MOTOR1_PHASE_A, b);
#endif
}

/**
* Re-configures UART IO
* <code>
* HAL_UART_ReconfigureIoMapping();
* </code>
*/
inline static void HAL_UART_ReconfigureIoMapping(void)
{
    // no action required
}

/**
 * Set all interrupt priorities.
 */
inline static void HAL_InterruptPrioritySet(void)
{
    HAL_TMR_TICK_InterruptPrioritySet();
    HAL_ADC_IndividualChannelInterruptPrioritySet();
    if (MCAF_SingleChannelEnabled())
    {
        HAL_ADC_BusCurrentInterruptPrioritySet();
    }
}

#ifdef __cplusplus
}
#endif

#endif /* __HAF_H */
