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
#include "rb_board_ui.h"
 
typedef enum 
{   
    RBFSM_INIT = 0,
    RBFSM_STARTUP,
    RBFSM_RUNNING,
    RBFSM_STOPPING,
    RBFSM_FAULTED
        
} RB_FSM_STATE; 

/** ISR Variables - These are passed into library functions as needed*/
RB_MOTOR_DATA PMSM;
RB_FSM_STATE state;
RB_BOOTSTRAP bootstrap;
RB_BOARD_UI boardUI;
uint16_t speedLevel = 9;

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
    
    // This function will update the button states and the POT value. 
    RB_BoardUIService(&boardUI);
            
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
            RB_BoardUIInit(&boardUI);
            
            
            state = RBFSM_STARTUP;
            break;
            
        case RBFSM_STARTUP:
           
            if(!bootstrap.done)
            {   
                // charge bootstrap capacitors over multiple ISR steps
                RB_PWMCapBootstrapISRStep(&bootstrap);
                
            } else if ((bootstrap.done) && (!PMSM.currentCal.done))
            {
               // next, take current measurements to calculate offset over multiple steps
               RB_MeasureCurrentOffsetStepISR(&PMSM.currentCal); 
            
            } else if((bootstrap.done) && (PMSM.currentCal.done) && (boardUI.motorEnable.state))
            {
                // get ready to output PWM 
                HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MIN_DUTY_COUNTS);
                HAL_PWM_UpperTransistorsOverride_Disable();
                MCAF_LED1_SetHigh();
                
                state = RBFSM_RUNNING;
            }
            
            break;
            
        case RBFSM_RUNNING:         
            
            RB_ADCRead(&PMSM.currentCal, &PMSM.iabc, &PMSM.vDC);
            RB_HALL_Estimate();
            
            RB_FixedFrequencySinePWM(boardUI.potState);
            
            if (!boardUI.motorEnable.state){

                //Maintains the low-side transistors at low dc and high-side OFF.
                HAL_PWM_UpperTransistorsOverride_Low();
                HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MAX_DUTY_COUNTS);
                MCAF_LED1_SetLow();
                
                state = RBFSM_STOPPING;
            }

            break;
            
        case RBFSM_STOPPING:
            
            if(boardUI.motorEnable.state)
            {   
                // get ready to output PWM 
                HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MIN_DUTY_COUNTS);
                HAL_PWM_UpperTransistorsOverride_Disable();
                MCAF_LED1_SetHigh();
                
                state = RBFSM_RUNNING;
            }
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

