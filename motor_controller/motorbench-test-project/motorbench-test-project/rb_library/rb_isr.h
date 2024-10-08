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
#include "motorBench/math_asm.h"
#include "timer/sccp5.h"
#include "rb_logging.h"
#include "spi_host/spi1.h"
#include "rb_mcp.h"
#include "rb_can.h"
    
#define THROTTLE_MULTIPLIER 327
    
// comment out for CAN controlled throttle
#define POT_THROTTLE

typedef enum 
{   
    RBFSM_INIT = 0,
    RBFSM_BOARD_INIT,
    RBFSM_MANUAL_STARTUP,
    RBFSM_RUN_FOC,
    RBFSM_FAULTED
        
} RB_FSM_STATE; 

/** ISR Variables - These are passed into library functions as needed*/
RB_MOTOR_DATA PMSM;
RB_HALL_DATA hall;
RB_FSM_STATE state;
bool stateChanged = false;
RB_BOOTSTRAP bootstrap;
RB_BOARD_UI boardUI;
RB_FAULT_DATA faultState;
int16_t throttleCmd_Q15 = 0;
uint16_t ADCISRExecutionTime; // monitor this value as code increases

// Logging objects
RB_LOGGING_SUMS logSums;
RB_LOGGING_AVERAGES logAverages;

// random can testing vars
RB_CAN_CONTROL CANControl;
int8_t tempThrottle = 0; // raw throttle value from teensy 
CAN_FRAME canFrame0;
extern uint8_t canTestArr[20]; // DO WE NEED THIS CHRIS?

uint8_t  SPI_received;
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
    /* Start timer to measure ADC ISR execution time. 
     * Period set to 0x1387 = 4999 = 50us
     */
    SCCP5_Timer_Stop();
    CCP5TMRL = 0;
    SCCP5_Timer_Start();
    
    switch(state){
                
        case RBFSM_INIT:
    
            RB_InitControlParameters(&PMSM);
            RB_PWMCapBootstrapInit(&bootstrap);
            RB_ADCCalibrationInit(&PMSM.currentCalib); 
            //RB_FixedFrequencySinePWMInit(); // only for testing
            RB_BoardUIInit(&boardUI);
            RB_FaultInit(&faultState);

            // logging init
            RB_Logging_SumReset(&logSums);
                        
            //CAN testing inits
            CANControl.timestamp = 0;
            CANControl.state = RBCAN_MESSAGE1;
            CANControl.counter = 0;
                  
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
            
            } else if((bootstrap.done) && (PMSM.currentCalib.done))
            {
                // Configure Hall ISRs and data
                RB_HALL_Init(&hall);       
                state = RBFSM_MANUAL_STARTUP;
                stateChanged = true;
            }
            
            break;
        
        case RBFSM_MANUAL_STARTUP:
            
            if(stateChanged)
            {
                //Maintains the low-side transistors at low dc and high-side OFF.
                HAL_PWM_UpperTransistorsOverride_Low();
                HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MAX_DUTY_COUNTS);
                MCAF_LED1_SetLow();
                stateChanged = false;
            }
            
            RB_ADCReadStepISR(&PMSM.currentCalib, &PMSM.iabc, &PMSM.vDC, 
                    &PMSM.iDC, &PMSM.vabc, &PMSM.bridgeTemp);
            RB_HALL_Estimate(&hall);
       
            if((throttleCmd_Q15 != 0) && (hall.minSpeedReached))
            {   
                state = RBFSM_RUN_FOC;
                stateChanged = true;
            }
            
            break;
            
        case RBFSM_RUN_FOC:         
            
            if(stateChanged)
            {
                RB_InitControlLoopState(&PMSM);
                
                // get ready to output PWM 
                HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MIN_DUTY_COUNTS);
                HAL_PWM_UpperTransistorsOverride_Disable();
                stateChanged = false;
            }
            
            RB_ADCReadStepISR(&PMSM.currentCalib, &PMSM.iabc, &PMSM.vDC, 
                    &PMSM.iDC, &PMSM.vabc, &PMSM.bridgeTemp);
           
            /* Calculate Id,Iq from Sin(theta), Cos(theta), Ia, Ib */
            MC_TransformClarke_Assembly(&PMSM.iabc,&PMSM.ialphabeta);
            MC_TransformPark_Assembly(&PMSM.ialphabeta, &PMSM.sincosTheta, &PMSM.idqFdb);
            
            /* LPF did not work well. try moving average filter forIq */
//            PMSM.idqFdb.q = RB_LPF(PMSM.idqFdb.q, prevIqOutput, Q15(0.628)); // around 1000Hz Fc
//            prevIqOutput = PMSM.idqFdb.q;
            
            /* Determine d & q current reference values based */ 
            RB_SetCurrentReference(throttleCmd_Q15, &PMSM.idqRef, &PMSM.iqRateLim, 
                    !hall.minSpeedReached);
            
            /* PI control for D-axis - sets Vd command */
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
            

            /* Estimate electrical angle into hall.theta */
            RB_HALL_Estimate(&hall);
            
            /* Forward path calculations */
            MC_CalculateSineCosine_Assembly_Ram(hall.theta,&PMSM.sincosTheta);
            MC_TransformParkInverse_Assembly(&PMSM.vdqCmd,&PMSM.sincosTheta,&PMSM.valphabetaCmd);
            MC_TransformClarkeInverseSwappedInput_Assembly(&PMSM.valphabetaCmd,&PMSM.vabcCmd);
            MC_CalculateSpaceVectorPhaseShifted_Assembly(&PMSM.vabcCmd, 4630, &PMSM.pwmDutyCycle); //4630=4775-145 
            
            /* Adjust Duties to be in between 145 and 4775 */
            RB_PWMDutyCycleAdjust(&PMSM.pwmDutyCycle, 145, 4775);  
            
            /* Lastly, Set duties */
            RB_PWMDutyCycleSet(&PMSM.pwmDutyCycle);
            
            /* For logging */
            RB_CalculateMotorOutput(&PMSM.power, &PMSM.torque, &PMSM.omega, 
                    PMSM.idqFdb.q, hall.speed);
            
            // TODO: if stopped and throttleCmd_Q15 is positive or zero, move to startup state
            if (!hall.minSpeedReached)
            {   
                state = RBFSM_MANUAL_STARTUP;
                stateChanged = true;
            }
            
            break;           
            
        case RBFSM_FAULTED:
            
            if(stateChanged)
            {
                //Maintains the low-side transistors at low dc and high-side OFF.
                HAL_PWM_UpperTransistorsOverride_Low();
                HAL_PWM_LowerTransistorsOverride_Low();
                stateChanged = false;
            }
            
            // do nothing
            break;        
    }
    
    RB_FaultCheck(&faultState, &PMSM.iabc, PMSM.bridgeTemp);
    if(faultState.isFault)
    {
        state = RBFSM_FAULTED;
        stateChanged = true;
    }

    // logging summations
    RB_Logging_SumStepISR(&logSums,
                          PMSM.vDC, PMSM.iDC, PMSM.idqRef.q, PMSM.idqFdb.q,
                          PMSM.power, hall.speed, PMSM.bridgeTemp, PMSM.iabc.a, 
                          PMSM.iabc.b, PMSM.vabc.a, PMSM.vabc.b);
    
    // calculate logging averages and rms
    if (CANControl.counter == RB_CAN_CYCLE_COUNT_MINUS1)
    {
        RB_Logging_Average(&logAverages, &logSums);
        RB_Logging_SumReset(&logSums);
    }
    
    //logAverages.speed = hall.speed; // set variables here before sending over CAN
    CANControl.counter = (CANControl.counter + 1) % RB_CAN_CYCLE_COUNT;
    RB_CAN_Service(&canFrame0, &tempThrottle, &CANControl, throttleCmd_Q15, 0, logAverages); // 0  = errorWarning
    
    
    if (boardUI.potThrottle.state){
        throttleCmd_Q15 = (boardUI.potState >= -3000 && boardUI.potState <= 3000) ? 0 
            : boardUI.potState;
        MCAF_LED2_SetHigh();

    }
    else {
        RB_ClampInput8Bit(&tempThrottle, MAX_THROTTLE_INPUT, MIN_THROTTLE_INPUT);
        throttleCmd_Q15 = (int16_t)(tempThrottle) * THROTTLE_MULTIPLIER;
        MCAF_LED2_SetLow();

    }
        
   
    /* interrupt flag must be cleared after data is read from buffer */
    HAL_ADC_InterruptFlag_Clear(); 
    
    /* Low-priority tasks at the end */
    X2CScope_Update();
    RB_BoardUIService(&boardUI); // update the button states and the POT value
    
    /* Lastly, record timer period to measure ADC ISR execution time */
    SCCP5_Timer_Stop();
    ADCISRExecutionTime = SCCP5_Timer_CounterGet();
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

