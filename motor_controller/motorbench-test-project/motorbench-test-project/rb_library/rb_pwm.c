#include "rb_pwm.h"

#include "../mcc_generated_files/pwm_hs/pwm.h"
#include "../mcc_generated_files/pwm_hs/pwm_hs_types.h"
#include "../motorbench/parameters/hal_params.h"

/**
 * Configures the PWM module as required.
 * This function is intended to be called after initializing the PWM module
 * and before enabling the PWM module.
 */
void RB_PWMInit(void)
{
    /* Sets the master period cycle for all PWM to synchronize. FRM DS70005320D formula,  PGxPER = (Fpgx_clk/Fpwm) - 1 */
    MCC_PWM_MasterPeriodSet(HAL_PARAM_PWM_PERIOD_COUNTS-1);
    
    /* Sets identical PWM deadtime values for Centeraligned PWM mode on all three phases */
    MCC_PWM_DeadTimeSet(MOTOR1_PHASE_A, HAL_PARAM_DEADTIME_COUNTS);
    MCC_PWM_DeadTimeSet(MOTOR1_PHASE_B, HAL_PARAM_DEADTIME_COUNTS);
    MCC_PWM_DeadTimeSet(MOTOR1_PHASE_C, HAL_PARAM_DEADTIME_COUNTS);
    
    /* Sets up the trigger designated to initiate sampling of analog channels. The trigger is
         set such that the sampling of motor currents occurs at the center of the low side pulse. */
    MCC_PWM_TriggerACompareValueSet(MOTOR1_PHASE_A,
                                (HAL_PARAM_DEADTIME_COUNTS >> 1) + HAL_PARAM_ADC_TRIGGER_DELAY);
    
    /* Initializes PWM registers for normal operation and set a default "safe" duty cycle*/
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_A,HAL_PARAM_MIN_DUTY_COUNTS);
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_B,HAL_PARAM_MIN_DUTY_COUNTS);
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_C,HAL_PARAM_MIN_DUTY_COUNTS);
   
    /* Disable the PWM channels assigned by overriding them to low state. */
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_A,0);
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_B,0);
    MCC_PWM_OverrideDataSet(MOTOR1_PHASE_C,0);   
    
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_A);
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_A);   
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_B);
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_B);    
    MCC_PWM_OverrideHighEnable(MOTOR1_PHASE_C);
    MCC_PWM_OverrideLowEnable(MOTOR1_PHASE_C);
    
    // Do we need this?
//    HAL_PWM_FaultClearBegin();
    
    // to disable the override, for when we're ready, do this:
//    HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MIN_DUTY_COUNTS);
//    HAL_PWM_LowerTransistorsOverride_Disable();
//    HAL_PWM_UpperTransistorsOverride_Disable();
    
    // to set Duties do this:
//    uint16_t pwmDutyCycle[3];
//    constrainDutyCycleAsArray(pwmDutyCycle, &pmotor->pwmDutycycle, HAL_PARAM_MIN_DUTY_COUNTS);
//    HAL_PWM_DutyCycleRegister_Set(pwmDutyCycle);
}

