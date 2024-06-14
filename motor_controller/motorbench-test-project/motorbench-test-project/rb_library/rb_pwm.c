#include "rb_pwm.h"

#include "pwm_hs/pwm_hs_types.h"
#include "parameters/hal_params.h"
#include "hal/hardware_access_functions.h"

/**
 * Frixed Frequency Sine PWM Variables
 */
uint16_t SineDutyCycle[297] = {2500, 2548, 2596, 2644, 2692, 2740, 2788, 2836, 2883, 2931, 2978, 3025, 3071, 3118, 3164, 3210, 3255,
3301, 3346, 3390, 3434, 3478, 3521, 3564, 3606, 3648, 3689, 3730, 3770, 3810, 3849, 3887, 3925,
3962, 3999, 4035, 4070, 4104, 4138, 4171, 4203, 4235, 4266, 4296, 4325, 4353, 4381, 4407, 4433,
4458, 4482, 4505, 4528, 4549, 4569, 4589, 4608, 4625, 4642, 4658, 4672, 4686, 4699, 4711, 4722,
4732, 4740, 4748, 4755, 4761, 4766, 4770, 4772, 4774, 4775, 4775, 4773, 4771, 4768, 4764, 4758,
4752, 4744, 4736, 4727, 4716, 4705, 4693, 4679, 4665, 4650, 4634, 4616, 4598, 4579, 4559, 4538,
4517, 4494, 4470, 4446, 4420, 4394, 4367, 4339, 4310, 4281, 4250, 4219, 4187, 4155, 4121, 4087,
4052, 4017, 3981, 3944, 3906, 3868, 3829, 3790, 3750, 3710, 3669, 3627, 3585, 3542, 3499, 3456,
3412, 3368, 3323, 3278, 3233, 3187, 3141, 3095, 3048, 3001, 2954, 2907, 2859, 2812, 2764, 2716,
2668, 2620, 2572, 2524, 2476, 2428, 2380, 2332, 2284, 2236, 2188, 2141, 2093, 2046, 1999, 1952,
1905, 1859, 1813, 1767, 1722, 1677, 1632, 1588, 1544, 1501, 1458, 1415, 1373, 1331, 1290, 1250,
1210, 1171, 1132, 1094, 1056, 1019, 983, 948, 913, 879, 845, 813, 781, 750, 719, 690,
661, 633, 606, 580, 554, 530, 506, 483, 462, 441, 421, 402, 384, 366, 350, 335,
321, 307, 295, 284, 273, 264, 256, 248, 242, 236, 232, 229, 227, 225, 225, 226,
228, 230, 234, 239, 245, 252, 260, 268, 278, 289, 301, 314, 328, 342, 358, 375,
392, 411, 431, 451, 472, 495, 518, 542, 567, 593, 619, 647, 675, 704, 734, 765,
797, 829, 862, 896, 930, 965, 1001, 1038, 1075, 1113, 1151, 1190, 1230, 1270, 1311, 1352,
1394, 1436, 1479, 1522, 1566, 1610, 1654, 1699, 1745, 1790, 1836, 1882, 1929, 1975, 2022, 2069,
2117, 2164, 2212, 2260, 2308, 2356, 2404, 2452};

volatile uint16_t RunningStateCounter = 0;
// phase indices below initialized 120* apart
volatile uint16_t phaseAIndex  = 0;
volatile uint16_t phaseBIndex  = 99;
volatile uint16_t phaseCIndex  = 198;
volatile uint16_t sineA  = 0;
volatile uint16_t sineB  = 0;
volatile uint16_t sineC  = 0;



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

void RB_FixedFrequencySinePWM(uint16_t freqDivider)
{
    RunningStateCounter++;
            
    // Sine Frequency = 1/ (X*(1/20000)*297), where phaseIndex++ if RunningStateCounter >= X
    // RPM = 120*f/52
    if (RunningStateCounter >= freqDivider) // 9 works
    {
        RunningStateCounter = 0;

        // retrieve sine() value
        sineA = SineDutyCycle[phaseAIndex];
        sineB = SineDutyCycle[phaseBIndex];
        sineC = SineDutyCycle[phaseCIndex];

        // set duties
        MCC_PWM_DutyCycleSet(MOTOR1_PHASE_A,sineA);
        MCC_PWM_DutyCycleSet(MOTOR1_PHASE_B,sineB);
        MCC_PWM_DutyCycleSet(MOTOR1_PHASE_C,sineC);

        // increment index of phases
        phaseAIndex++;
        phaseBIndex++;
        phaseCIndex++;

        // wrap around sine table
        if (phaseAIndex > 296)
        {
            phaseAIndex = 0;
        } else if (phaseBIndex > 296)
        {
            phaseBIndex = 0;
        } else if (phaseCIndex > 296)
        {
            phaseCIndex = 0;
        }
    }
}