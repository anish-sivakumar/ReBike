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
#include "library/mc-library/motor_control.h"

// Specify PWM Period in micro seconds
#define LOOPTIME_MICROSEC       50
// Oscillator frequency (MHz) - 200MHz
#define FOSC_MHZ                200U     
// loop time in terms of PWM clock period (5000 - 1)
#define PWM_PERIOD              (uint16_t)(((LOOPTIME_MICROSEC*FOSC_MHZ)/2)-1)

    
 
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
RB_HALL_DATA hall;
RB_FSM_STATE state;
RB_BOOTSTRAP bootstrap;
RB_BOARD_UI boardUI;


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
    
            //MCAF_FaultDetectInit(&PMSM.faultDetect);
            RB_InitControlParameters(&PMSM);
            //MCAF_MotorControllerOnRestartInit(pmotor); not sure if IMPORTANT
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
                
            } else if ((bootstrap.done) && (!PMSM.currentCalib.done))
            {
               // next, take current measurements to calculate offset over multiple steps
               RB_ADCCalibrationStepISR(&PMSM.currentCalib); 
            
            } else if((bootstrap.done) && (PMSM.currentCalib.done) && (boardUI.motorEnable.state))
            {
                // Configure Hall ISRs and data
                RB_HALL_Init(&hall);
                
                // get ready to output PWM 
                HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MIN_DUTY_COUNTS);
                HAL_PWM_UpperTransistorsOverride_Disable();
                MCAF_LED1_SetHigh();
                
                state = RBFSM_RUNNING;
            }
            
            break;
            
        case RBFSM_RUNNING:         
            
        /*********************************************************************
        *       START OF FOC ALGORITHM                                       *
        *********************************************************************/  
            RB_ADCReadStepISR(&PMSM.currentCalib, &PMSM.iabc, &PMSM.vDC);
            HAL_ADC_InterruptFlag_Clear(); // interrupt flag must be cleared after data is read from buffer
            
            /* Calculate qId,qIq from qSin,qCos,qIa,qIb */
            MC_TransformClarke_InlineC(&PMSM.iabc,&PMSM.ialphabeta);
            MC_TransformPark_InlineC(&PMSM.ialphabeta, &PMSM.sincosTheta, &PMSM.idqFdb);
            
            /* Determine d & q current reference values based */
            RB_SetCurrentReference(boardUI.potState, &PMSM.idqRef);
                        
            /* PI control for D-axis - sets Vd command*/
            MC_ControllerPIUpdate_Assembly( PMSM.idqRef.d, 
                                            PMSM.idqFdb.d, 
                                            &PMSM.idCtrl, 
                                            &PMSM.vdqCmd.d);
            
            /* Dynamic d-q adjustment with d component priority vq=sqrt (vs^2 - vd^2) 
                limit vq maximum to the one resulting from the calculation above 
                MAX_VOLTAGE_VECTOR = 0.98 */
            int16_t temp_q15 = (int16_t)(__builtin_mulss(PMSM.vdqCmd.d,
                                                 PMSM.vdqCmd.d) >> 15);
            temp_q15 = Q15(0.98) - temp_q15; 
            PMSM.iqCtrl.outMax = Q15SQRT(temp_q15); // set max Iq
            PMSM.iqCtrl.outMin = - PMSM.iqCtrl.outMax; // set min Iq
            
            /* PI control for Q-axis - sets Vq command */
            MC_ControllerPIUpdate_Assembly(PMSM.idqRef.q,
                                           PMSM.idqFdb.q,
                                           &PMSM.iqCtrl,
                                           &PMSM.vdqCmd.q);
            
            // estimate electrical angle into hall.theta
            RB_HALL_Estimate(&hall);
            
            MC_CalculateSineCosine_Assembly_Ram(hall.theta,&PMSM.sincosTheta);
            MC_TransformParkInverse_Assembly(&PMSM.vdqCmd,&PMSM.sincosTheta,&PMSM.valphabetaCmd);
            MC_TransformClarkeInverseSwappedInput_Assembly(&PMSM.valphabetaCmd,&PMSM.vabcCmd);
            MC_CalculateSpaceVectorPhaseShifted_Assembly(&PMSM.vabcCmd, PWM_PERIOD, &PMSM.pwmDutycycle);
            
            /* Lastly, Set duties */
            
        /*********************************************************************
        *       END OF FOC ALGORITHM                                         *
        *********************************************************************/
            
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
    
    /* Low-priority tasks at the end */
    X2CScope_Update();
    RB_BoardUIService(&boardUI); // update the button states and the POT value
}


/**
 * TMR1 timeout routine
 */
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; // reset interrupt flag
    RB_HALL_Reset(&hall);
}


/**
 * Hall sensor interrupt
 * 
 */
void RB_HALL_ISR(void)
{
    RB_HALL_StateChange(&hall);
}

/**
 * Start state machine in initialization state
 */
void RB_ISR_StateInit(void){
    state = RBFSM_INIT;  
}

#ifdef	__cplusplus
}
#endif

#endif	/* RB_ISR_H */

