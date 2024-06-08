/* 
 * File:   rb_isr.h
 * Author: siani
 *
 * Created on June 8, 2024, 3:32 PM
 */

#ifndef RB_ISR_H
#define	RB_ISR_H

#ifdef	__cplusplus
extern "C" {
#endif

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

#include "X2CScope.h"
#include "rb_control.h"
#include "rb_hall.h"

/**
 * motor state variables, accessed directly
 * 
 * The ISR does not receive any arguments so we need access to this somehow.
 */
extern RB_MOTOR_DATA PMSM;

/** system data, accessed directly */
extern MCAF_SYSTEM_DATA systemData;

/** watchdog state, accessed directly */
extern volatile MCAF_WATCHDOG_T watchdog;

extern volatile uint16_t ISR_testing; //temp

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
//void TEST_ISR(void)
{
   
    RB_HALL_Estimate();
   
  
   
   // 1. read current ADC buffer and Vdc ADC, apply offsets, and LPF if needed
   //RB_ADCRead(&PMSM.currentCalibration, &PMSM.iabc, &PMSM.vDC);
   
  
   
   HAL_ADC_InterruptFlag_Clear(); // interrupt flag must be cleared after data is read from buffer
   
   // 2. FOC Feedback path
   // like: MCAF_FocStepIsrFeedbackPath(pmotor), but break it up into clarke, and park
   
   // 3. Estimate Angle and Speed and save into main data structure
   // like: MCAF_CommutationPrepareStallDetectInputs, but use MC_HALL_Estimate code
   
   // 4. fault detection - understand this better
   // like: MCAF_MonitorSysDiagnose
   
   // 5. determine next state (simplify to running or faulted)
   
   // 6. If faulted, run fault handling code
   // like: MCAF_MotorControllerOnFault and MCAF_MotorControllerOnFaultInit
   
   // 7. If running as usual, run forward FOC code:
   // like: MCAF_FocStepIsrForwardPath, but we can break this up as below
    
    // 8. Do PI control
    // like: MCAF_VelocityAndCurrentControllerStep
    
    // 9. Calculate Sine(theta) and Cos(theta) like: MC_CalculateSineCosine
    
    // 10. Calculate Park Inverse like MC_TransformParkInverse
    
    // 11. Calculate Clarke Inverse like MC_TransformClarkeInverse
    // skip deadtime compensation stuff
    
    // 12. figure out if MCAF_ApplyDCLinkVoltageCompensation is needed
    
    // 13. do SVPWM like MC_CalculateZeroSequenceModulation or calculateManualZeroSequenceModulation
    
    // 14. scaling? MCAF_ScaleQ15(&pmotor->dabc, &pmotor->pwmDutycycle,HAL_PARAM_PWM_PERIOD_COUNTS);
    
    // 15. Figure out where duties are set and do it
   
    // 16. clear ADC
    //HAL_ADC_InterruptFlag_Clear(); // interrupt flag must be cleared after data is read from ADC buffer
   
    // Lastly: Diagnostics code are always the lowest-priority routine within
    X2CScope_Update();
}

/**
 * TMR1 timeout routine
 */
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; // reset interrupt flag
    RB_HALL_Reset();

}

/**
 * Hall sensor interrupt
 * 
 */
void RB_HALL_ISR(void)
{
    RB_HALL_StateChange();
    
}



#ifdef	__cplusplus
}
#endif

#endif	/* RB_ISR_H */

