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
#include "timer/sccp4.h"
#include "library/mc-library/motor_control.h"

 
typedef enum 
{   
    RBFSM_INIT = 0,
    RBFSM_BOARD_INIT,
    RBFSM_MANUAL_STARTUP,     
    RBFSM_RUN_OPENLOOP,
    RBFSM_RUN_FOC,
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
            //RB_FocInit(&PMSM);
            RB_ADCCalibrationInit(&PMSM.currentCalib); 
            RB_FixedFrequencySinePWMInit(); //for testing
            RB_BoardUIInit(&boardUI);
            
            state = RBFSM_BOARD_INIT;
            break;
            
        case RBFSM_BOARD_INIT:
            
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
                state = RBFSM_MANUAL_STARTUP;
                MCAF_LED1_SetHigh();
            }
            
            break;
        
        case RBFSM_MANUAL_STARTUP:
            RB_ADCReadStepISR(&PMSM.currentCalib, &PMSM.iabc, &PMSM.vDC);
            RB_HALL_Estimate(&hall);
            
            // (hall.speed > 25) 
            if((hall.speed > 22))// speed calculation is messed up. fix it
            {   
                // get ready to output PWM 
                HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MIN_DUTY_COUNTS);
                HAL_PWM_UpperTransistorsOverride_Disable();
                
                
                RB_InitControlLoopState(&PMSM);
                state = RBFSM_RUN_FOC;
            }
            
            break;
            
        case RBFSM_RUN_OPENLOOP:
            /* Runs fixed frequency SinePWM based on potentiometer input */
            RB_ADCReadStepISR(&PMSM.currentCalib, &PMSM.iabc, &PMSM.vDC);
            RB_HALL_Estimate(&hall);
            RB_FixedFrequencySinePWM(boardUI.potState);
            
            /* TESTING */
            RB_SetCurrentReference(boardUI.potState, &PMSM.idqRef);
            
            // if we hit speed 27RPM, move to FOC
            if (hall.speed >= 27){ // 35 is the top speed from OL now
                
                RB_InitControlLoopState(&PMSM);
                
                state = RBFSM_RUN_FOC;
            }
            
            break;
            
        case RBFSM_RUN_FOC:         
            
        /*********************************************************************
        *       START OF FOC ALGORITHM                                       *
        *********************************************************************/ 
            
            RB_ADCReadStepISR(&PMSM.currentCalib, &PMSM.iabc, &PMSM.vDC);
           
            /* Calculate qId,qIq from qSin,qCos,qIa,qIb */
            MC_TransformClarke_Assembly(&PMSM.iabc,&PMSM.ialphabeta);
            MC_TransformPark_Assembly(&PMSM.ialphabeta, &PMSM.sincosTheta, &PMSM.idqFdb);
            
            /* Determine d & q current reference values based */
            RB_SetCurrentReference(boardUI.potState, &PMSM.idqRef);
                        
            /* PI control for D-axis - sets Vd command*/
            MC_ControllerPIUpdate_Assembly( PMSM.idqRef.d, 
                                            PMSM.idqFdb.d, 
                                            &PMSM.idCtrl, 
                                            &PMSM.vdqCmd.d);
            
            /* Dynamic d-q adjustment with d component priority vq=sqrt(vs^2 - vd^2) 
                limit vq maximum to the one resulting from the calculation above 
                MAX_VOLTAGE_VECTOR^2 = 0.98 */
            int16_t temp_q15 = (int16_t)(__builtin_mulss(PMSM.vdqCmd.d,
                                                 PMSM.vdqCmd.d) >> 15);
            temp_q15 = Q15(0.98) - temp_q15; 
            PMSM.iqCtrl.outMax = Q15SQRT(temp_q15); // set VqCmd for max Iq
            PMSM.iqCtrl.outMin = - PMSM.iqCtrl.outMax; // set VqCmd for min Iq
            
            /* PI control for Q-axis - sets Vq command */
            MC_ControllerPIUpdate_Assembly(PMSM.idqRef.q,
                                           PMSM.idqFdb.q,
                                           &PMSM.iqCtrl,
                                           &PMSM.vdqCmd.q);
            
            /* estimate electrical angle into hall.theta */
            RB_HALL_Estimate(&hall);
            
            /* forward path calculations */
            MC_CalculateSineCosine_Assembly_Ram(hall.theta,&PMSM.sincosTheta);
            MC_TransformParkInverse_Assembly(&PMSM.vdqCmd,&PMSM.sincosTheta,&PMSM.valphabetaCmd);
            MC_TransformClarkeInverseSwappedInput_Assembly(&PMSM.valphabetaCmd,&PMSM.vabcCmd);
            MC_CalculateSpaceVectorPhaseShifted_Assembly(&PMSM.vabcCmd, 4630, &PMSM.pwmDutyCycle); //4630=4775-145 
            
            /* Adjust Duties to be in between 145 and 4775 */
            RB_PWMDutyCycleAdjust(&PMSM.pwmDutyCycle, 145, 4775);  
            
            /* Lastly, Set duties */
            RB_PWMDutyCycleSet(&PMSM.pwmDutyCycle);
            
            // if we button pressed, stop
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
                
                state = RBFSM_MANUAL_STARTUP;
            }
            break;
            
        case RBFSM_FAULTED:
            break;      
        
    }
    
    HAL_ADC_InterruptFlag_Clear(); // interrupt flag must be cleared after data is read from buffer
    /* Low-priority tasks at the end */
    X2CScope_Update();
    RB_BoardUIService(&boardUI); // update the button states and the POT value
}


// /**
//  * TMR1 timeout routine (currently unused)
//  */
// void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
//     IFS0bits.T1IF = 0; // reset interrupt flag
// //    RB_HALL_Reset(&hall);
// }

/**
 * Hall timer (SCCP4) expiry routine
 */
void RB_HALL_TIMEOUT_ISR(void)
{
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

