#include "rb_pwm.h"
#include "pwm_hs/pwm.h"
#include "pwm_hs/pwm_hs_types.h"
#include "parameters/hal_params.h"
#include "hal/hardware_access_functions.h"

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

void RB_PWMCapBootstrapInit(RB_BOOTSTRAP *pBootstrap) {
    pBootstrap->state = RBBS_IDLE;
    pBootstrap->delayCount = 0;
    pBootstrap->dutyA = 0;
    pBootstrap->dutyB = 0;
    pBootstrap->dutyC = 0;
}


bool RB_PWMCapBootstrapISRStep(RB_BOOTSTRAP *pBootstrap) {

    bool bootstrapDone = false;

    switch (pBootstrap->state) {
        case RBBS_IDLE:
            // Disable PWMs
            HAL_PWM_Outputs_Disable();
            // Specify duties
            pBootstrap->dutyA = HAL_PARAM_PWM_PERIOD_COUNTS;
            pBootstrap->dutyB = HAL_PARAM_PWM_PERIOD_COUNTS;
            pBootstrap->dutyC = HAL_PARAM_PWM_PERIOD_COUNTS;
            
            HAL_PWM_UpperTransistorsOverride_Low(); //upper switches off
            HAL_PWM_LowerTransistorsOverride_Disable(); //lower side PWM controlled
            
            // State transition
            pBootstrap->state = RBBS_INIT_WAIT;
            pBootstrap->delayCount = MCAF_BOARD_BOOTSTRAP_INITIAL_DELAY;
            break;
            
        case RBBS_INIT_WAIT:
            // Wait some time before charging 
            if (pBootstrap->delayCount == 0)
            {   
                pBootstrap->state = MCBS_PHASE_A_SETUP_CHARGING;
            }
            break;
            
        case RBBS_A_SETUP:
            // Set PWM A duty to enable charging on A
            pBootstrap->dutyA = RB_MIN_DUTY_FOR_BOOTSRAP_CHARGING;
            // State Transition
            pBootstrap->state= RBBS_A_CHARGING;
            pBootstrap->delayCount = MCAF_BOARD_BOOTSTRAP_PHASE_DELAY;
            break;
            
        case RBBS_A_CHARGING:
            // Wait to let the the Phase-A bootstrap capacitor charge
            // When the delay counter finishes, start charging B
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->state= RBBS_B_SETUP;
            }
            break;
        
        case RBBS_B_SETUP:
            // Set PWM B duty to enable charging on B
            pBootstrap->dutyB = RB_MIN_DUTY_FOR_BOOTSRAP_CHARGING;
            // State Transition
            pBootstrap->state= RBBS_B_CHARGING;
            pBootstrap->delayCount = MCAF_BOARD_BOOTSTRAP_PHASE_DELAY;
            break;
            
        case RBBS_B_CHARGING:
            // Wait to let the the Phase-B bootstrap capacitor charge
            // When the delay counter finishes, start charging C
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->state= RBBS_C_SETUP;
            }
            break;
            
        case RBBS_C_SETUP:
            // Set PWM C duty to enable charging on C
            pBootstrap->dutyC = RB_MIN_DUTY_FOR_BOOTSRAP_CHARGING;
            // State Transition
            pBootstrap->state= RBBS_C_CHARGING;
            pBootstrap->delayCount = MCAF_BOARD_BOOTSTRAP_PHASE_DELAY;
            break;
            
        case RBBS_C_CHARGING:
            // Wait to let the the Phase-C bootstrap capacitor charge
            // When the delay counter finishes, charging is done
            if (pBootstrap->delayCount == 0)
            {
                pBootstrap->state = RBBS_DONE;
            }
            break;  
            
        case RBBS_DONE:
            bootstrapDone = true;
            break;
    }
    
    // Final set duties
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_A, pBootstrap->dutyA);
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_B, pBootstrap->dutyB);
    MCC_PWM_DutyCycleSet(MOTOR1_PHASE_C, pBootstrap->dutyC);
    
    // Decrement delay counter
    if (pBootstrap->delayCount > 0)
    {
        pBootstrap->delayCount--;
    }
    
    return bootstrapDone;
}

