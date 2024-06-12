#include "rb_pwm.h"
#include "pwm_hs/pwm.h"

#include "parameters/hal_params.h"
#include "hal/hardware_access_functions.h"

void RB_PWMInit(void) 
{

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
            // Overrides, don't fully understand
            // First example project seems to do this differently 
            HAL_PWM_UpperTransistorsOverride_Low();
            HAL_PWM_LowerTransistorsOverride_Disable();
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

