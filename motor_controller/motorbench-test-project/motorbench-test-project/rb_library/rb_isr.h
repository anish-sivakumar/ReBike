/* 
 * File:   rb_isr.h
 * Author: siani
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
volatile uint16_t phaseAIndex  = 0;
volatile uint16_t phaseBIndex  = 99;
volatile uint16_t phaseCIndex  = 198;

volatile uint16_t sineA  = 0;
volatile uint16_t sineB  = 0;
volatile uint16_t sineC  = 0;



/** Global Variables */
RB_MOTOR_DATA PMSM;
RB_FSM_STATE current_state;
RB_FSM_STATE next_state;

RB_BOOTSTRAP bootstrap;

bool bootstrapDone;


/** system data, accessed directly */
extern MCAF_SYSTEM_DATA systemData;

/** watchdog state, accessed directly */
//extern volatile MCAF_WATCHDOG_T watchdog;

extern volatile uint16_t ISR_testing; //temp

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
    
    
    
    const bool state_changed = (next_state != current_state);
    current_state = next_state;
    
    switch(next_state){
        
        case RBFSM_INIT:
            
            MCAF_FaultDetectInit(&PMSM.faultDetect);
            RB_InitControlParameters(&PMSM);
            //MCAF_MotorControllerOnRestartInit(pmotor); not sure if IMPORTANT?
            MCAF_FaultDetectInit(&PMSM.faultDetect);
            RB_PWMCapBootstrapInit(&bootstrap);
            
            // might need this: HAL_PWM_FaultClearBegin();
            RB_FocInit(&PMSM);
            
            next_state = RBFSM_STARTUP;
            break;
            
        case RBFSM_STARTUP:
            
            bootstrapDone = RB_PWMCapBootstrapISRStep(&bootstrap);
            if (bootstrapDone) {
                // calibrate ADC offsets before RUnning
                next_state = RBFSM_RUNNING;
            }
            
            break;
            
        case RBFSM_RUNNING:
            if (state_changed)
            {
                HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MIN_DUTY_COUNTS);
                HAL_PWM_UpperTransistorsOverride_Disable();
                MCAF_LED1_SetHigh();
            }
            
            RunningStateCounter++;
            
            if (RunningStateCounter >= 10)
            {
                RunningStateCounter = 0;
                
                sineA = SineDutyCycle[phaseAIndex];
                sineB = SineDutyCycle[phaseBIndex];
                sineC = SineDutyCycle[phaseCIndex];
                
                //MCC_PWM_DutyCycleSet(MOTOR1_PHASE_A,SineDutyCycle[phaseAIndex]);
                //MCC_PWM_DutyCycleSet(MOTOR1_PHASE_B,SineDutyCycle[phaseBIndex]);
                //MCC_PWM_DutyCycleSet(MOTOR1_PHASE_C,SineDutyCycle[phaseBIndex]);
                
                phaseAIndex++;
                phaseBIndex++;
                phaseCIndex++;
                
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
    current_state = RBFSM_INIT;
    next_state = RBFSM_INIT;
}

#ifdef	__cplusplus
}
#endif

#endif	/* RB_ISR_H */

