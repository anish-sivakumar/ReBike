/* 
 * File:   rb_pwm.h
 * Author: siani
 *
 * Created on June 8, 2024, 3:57 PM
 */

#ifndef RB_PWM_H
#define	RB_PWM_H

#include "board_service_types.h"

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
    RBBS_IDLE,
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

    
} RB_BOOTSTRAP;

/**
 * Functions
 */

void RB_PWMInit(void);

/**
 * Function to perform capacitor bootstrap charging during motor starting
 */
bool RB_PWMCapBootstrapISRStep(RB_BOOTSTRAP *pBootstrap);


#ifdef	__cplusplus
}
#endif

#endif	/* RB_PWM_H */

