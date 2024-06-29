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
    pPMSM->motorParams.ke = RB_MOTOR_KE;
    
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


void RB_SetCurrentReference(int16_t throttleCmd, MC_DQ_T *pidqRef, RB_RATELIMIT *iqRateLim)
{    
    // d-axis current controlled at zero
    pidqRef->d = 0;
    
    // if pot is below mid point, target Iq = 0. 
    // pot mid point is around 1600 ADC reading
    if (throttleCmd <= 2000)
    {
        iqRateLim->target = 0;
    } else
    {
        // target Iq = potVal scaled from 0->-RB_QCURRENT_MAX
        iqRateLim->target = __builtin_mulss(throttleCmd, -RB_QCURRENT_MAX)>>15; 
    }  
   
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
