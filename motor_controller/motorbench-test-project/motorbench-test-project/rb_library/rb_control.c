/*
 * File:   rb_control.c
 * Author: siani
 *
 * Created on June 5, 2024, 4:10 PM
 */

#include "rb_control.h"
#include "util.h"

void RB_InitControlParameters(RB_MOTOR_DATA *pPMSM)
{
    /* ============= Motor Parameters =============== */
    pPMSM->motorParams.rs = RB_MOTOR_RS;
    pPMSM->motorParams.ke = RB_MOTOR_KV;
    
    /* ============= PI D Term =============== */
    pPMSM->idCtrl.kp = RB_DCURRENT_KP;
    pPMSM->idCtrl.ki = RB_DCURRENT_KI;
    pPMSM->idCtrl.kc = RB_DCURRENT_KC;
    pPMSM->idCtrl.outMax = RB_DVOLTAGE_OUTMAX;
    pPMSM->idCtrl.outMin = RB_DVOLTAGE_OUTMIN;

    /* ============= PI Q Term =============== */
    pPMSM->iqCtrl.kp = RB_QCURRENT_KP;
    pPMSM->iqCtrl.ki = RB_QCURRENT_KI;
    pPMSM->iqCtrl.kc = RB_QCURRENT_KC;
    pPMSM->iqCtrl.outMax = RB_QVOLTAGE_OUTMAX;
    pPMSM->iqCtrl.outMin = RB_QVOLTAGE_OUTMIN;
    
    /* ============= Q Ramp Rate terms =============== */
    pPMSM->iqRateLim.inc = RB_QRAMP_INCREMENT;
    pPMSM->iqRateLim.rampCount = 0;
    
    /* ============= Bridge Temperature Terms - Do Later=============== */
}

bool RB_FocInit(RB_MOTOR_DATA *pPMSM)
{
    RB_InitControlLoopState(pPMSM);
      
    /* initialize the mux'd channel (doesn't matter which setting is first) */
    //pPMSM->adcSelect = HADC_POTENTIOMETER; Anish removed this variable from PMSM
    
    /* initialize ADC compensation parameters  */
    RB_ADCCalibrationInit(&pPMSM->currentCalib); 
    
    //pPMSM->rVdc = MCAF_ComputeReciprocalDCLinkVoltage(INT16_MAX);
    
    //MCAF_SatInit(&pPMSM->sat);
    
    return true;
}


void RB_SetCurrentReference(int16_t throttleCmd, MC_DQ_T *pidqRef, 
        RB_RATELIMIT *iqRateLim, bool stopped)
{    
    /* D-axis Current Reference for controlling Flux
     *  - controlled at zero for positive throttle
     *  - Do later: if stopped and still braking, set Id nonzero (negative?)
     */
   
    // regular operation: motoring and regeneration
    pidqRef->d = 0;
    

     
    /* Q-axis Current Reference for controlling Torque
     * - target Iq is set to the potVal scaled from 0 to -RB_QCURRENT_MAX
     *      because positive bike direction corresponds to negative Iq current
     * - reference is rate limited by RB_QRAMP_INCREMENT
     */
    
    // regular operation: motoring and regeneration
    iqRateLim->target = __builtin_mulss(throttleCmd, -RB_QCURRENT_MAX)>>15; 
    
    
    // Set & limit Iq reference every RB_QRAMP_COUNT ISRs
    if (iqRateLim->rampCount < RB_QRAMP_COUNT)
    {
        iqRateLim->rampCount++;
    } else
    {
        iqRateLim->diff = pidqRef->q - iqRateLim->target;
        
        // increase (since Iq is negative, this is slowing down)
        if (iqRateLim->diff < 0)
        {
            /* Set this cycle reference as the sum of
            previously calculated one plus the reference ramp value */
            pidqRef->q = pidqRef->q + iqRateLim->inc;
        } else
        {   // decrease (since Iq is negative, this is speeding up)
            pidqRef->q = pidqRef->q - iqRateLim->inc;
        }
        
        /* If difference less than half of increment, set reference
            directly from the pot */
        if (UTIL_Abs16(iqRateLim->diff) < (iqRateLim->inc << 1))
        {
            pidqRef->q = iqRateLim->target;
        }
        
        iqRateLim->rampCount = 0;
        
    }
}


bool RB_PISaturationDetect(int16_t *piqRef, int16_t iqFdb, int16_t vqCmd, 
        int16_t speed)
{
    bool isSaturated = false;
    
    /* Check if the voltage command is at the limit 
     * and if the current is not reaching the reference value
     */ 
//    if(vqCmd <= RB_VOLTAGE_CMD_MIN && iqFdb > *piqRef)
//    {
//
//        *piqRef = -1400; // adjust reference
//        isSaturated = true;
//    }
//    
//    if(vqCmd >= RB_VOLTAGE_CMD_MAX && iqFdb < *piqRef)
//    {
//        *piqRef = 1400; // adjust reference
//        isSaturated = true;
//    }
    
    return isSaturated;
}