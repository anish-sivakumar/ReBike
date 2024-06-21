/* 
 * File:   rb_pwm.h
 * Author: siani
 *
 * Created on June 8, 2024, 3:57 PM
 */

#ifndef RB_PWM_H
#define	RB_PWM_H

#include "board_service_types.h"
#include "library/mc-library/motor_control.h"

#ifdef	__cplusplus
extern "C" {
#endif

/**
 * Defines
 */    
    
// Simplified from the function minimumDutyCycleForBootstrapCharging()    
// Dont know why this is the case    
#define RB_MIN_DUTY_FOR_BOOTSRAP_CHARGING HAL_PARAM_MAX_DUTY_COUNTS   

/**
 * Typedefs
 */
    
/**
 * Bootstrap Capacitor Charging FSM states
 */    
typedef enum tagRB_BOOTSTRAP_STATE
{
    RBBS_IDLE = 0,
    RBBS_INIT_WAIT,
    RBBS_A_SETUP,
    RBBS_A_CHARGING,
    RBBS_B_SETUP,
    RBBS_B_CHARGING,
    RBBS_C_SETUP,
    RBBS_C_CHARGING,
    RBBS_DONE
} RB_BOOTSTRAP_STATE;


typedef struct tagRB_BOOTSTRAP
{
    // defines what step of the bootstrap charging sequence we are in
    RB_BOOTSTRAP_STATE state;
    
    // used to control delays in the bootstrap charging states
    uint16_t delayCount;
    
    // used to store the 3 PWM duties during bootstrap charging
    uint16_t dutyA;
    uint16_t dutyB;
    uint16_t dutyC;
    
    // true when bootstrap charging is complete
    bool done;

    
} RB_BOOTSTRAP;


typedef struct tagRB_SINE_PWM
{
    volatile uint16_t runningStateCounter;
    
    volatile uint16_t phaseAIndex; // phase indices 120 degrees apart
    volatile uint16_t phaseBIndex;
    volatile uint16_t phaseCIndex;
    volatile uint16_t sineA;
    volatile uint16_t sineB;
    volatile uint16_t sineC;
} RB_SINE_PWM;

/**
 * Functions
 */

/**
 * Initialized PWM module
 */
void RB_PWMInit(void);


/**
 * Initializes capacitor bootstrap object
 * @param pBootstrap
 */
void RB_PWMCapBootstrapInit(RB_BOOTSTRAP *pBootstrap);


/**
 * Function to perform capacitor bootstrap charging during motor starting
 */
void RB_PWMCapBootstrapISRStep(RB_BOOTSTRAP *pBootstrap);


/**
 * Initializes Sine LUT indices for SinePWM
 */
void RB_FixedFrequencySinePWMInit (void);


/**
 * Updates Duties based on Sine LUT
 * @param freqDivider
 */
void RB_FixedFrequencySinePWM(uint16_t freqDivider);

/**
 * Clip SVPWM Duties to [145, 4775]
 * @param pPwmDutycycle
 * @param min
 * @param max
 */
void RB_PWMDutyCycleAdjust(MC_DUTYCYCLEOUT_T *pPwmDutycycle, uint16_t min,uint16_t max);

/**
 * Sets duty cycle values to PWM registers
 * @param pPwmDutycycle
 */
void RB_PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *pPwmDutycycle);




#ifdef	__cplusplus
}
#endif

#endif	/* RB_PWM_H */

