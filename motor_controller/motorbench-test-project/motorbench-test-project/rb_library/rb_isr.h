/* 
 * File:   rb_isr.h
 * Author: Chris Hyggen, Anish Sivakumar
 *
 * Created on June 8, 2024, 3:32 PM
 */

#ifndef RB_ISR_H
#define	RB_ISR_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "fault_detect.h"
    
#include "X2CScope.h"
#include "rb_control.h"
#include "rb_hall.h"
#include "rb_pwm.h"
 
typedef enum 
{   
    RBFSM_INIT = 0,
    RBFSM_STARTUP,
    RBFSM_RUNNING,
    RBFSM_FAULTED
        
} RB_FSM_STATE; 

/** Global Variables */
RB_MOTOR_DATA PMSM;
RB_FSM_STATE state;
RB_BOOTSTRAP bootstrap;

bool bootstrapDone;

/** system data, accessed directly */
extern MCAF_SYSTEM_DATA systemData;

/** watchdog state, accessed directly */
//extern volatile MCAF_WATCHDOG_T watchdog;


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
    
    switch(state){
                
        case RBFSM_INIT:
            
            MCAF_FaultDetectInit(&PMSM.faultDetect);
            RB_InitControlParameters(&PMSM);
            //MCAF_MotorControllerOnRestartInit(pmotor); not sure if IMPORTANT?
            MCAF_FaultDetectInit(&PMSM.faultDetect);
            RB_PWMCapBootstrapInit(&bootstrap);
            
            // might need this: HAL_PWM_FaultClearBegin();
            RB_FocInit(&PMSM);
            
            RB_FixedFrequencySinePWMInit(); //for testing
            
            state = RBFSM_STARTUP;
            break;
            
        case RBFSM_STARTUP:
            
            
            // charge bootstrap capacitors over multiple ISR steps
            bootstrapDone = RB_PWMCapBootstrapISRStep(&bootstrap);
            
            // next, take current measurements to calculate offset over multiple steps
            if (bootstrapDone)
            {
                RB_MeasureCurrentOffset(&PMSM.currentCalibration);
            }
            
            // move to next state once both are done
            if (bootstrapDone && PMSM.currentCalibration.calibrationComplete){
               
                RB_MeasureCurrentOffset(&PMSM.currentCalibration);

                // get ready to output PWM
                HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MIN_DUTY_COUNTS);
                HAL_PWM_UpperTransistorsOverride_Disable();
                MCAF_LED1_SetHigh();
                
                state = RBFSM_RUNNING;
            }
            
            break;
            
        case RBFSM_RUNNING:         
            
            RB_ADCRead(&PMSM.currentCalibration, &PMSM.iabc, &PMSM.vDC);
            
            // Sine Frequency = 1 / (X*(1/20000)*297)
            // RPM = 120*f/52
            RB_FixedFrequencySinePWM(9);  // 9,10 tested
            
            RB_HALL_Estimate();
          
            break;
            
        case RBFSM_FAULTED:
            break;      
        
    }
    
    HAL_ADC_InterruptFlag_Clear(); // interrupt flag must be cleared after data is read from buffer
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

void RB_ISR_StateInit(void){
    state = RBFSM_INIT;
    
}

#ifdef	__cplusplus
}
#endif

#endif	/* RB_ISR_H */

