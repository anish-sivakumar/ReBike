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
    pPMSM->motorParams.rs = RB_RS;
    pPMSM->motorParams.ke = RB_KE;
    
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
    
    /* ============= PI Speed Terms - Do Later =============== */
    
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


void RB_SetCurrentReference(int16_t potVal, MC_DQ_T *pidqRef)
{
       
    // d-axis current controlled at zero
    pidqRef->d = 0;
    
    // if pot is below mid point, iq ref = 0. pot mid point is around 1600 ADC reading
    if (potVal <= 2000)
    {
        pidqRef->q = 0;
    } else
    {
        // iq ref = potVal scaled from 0->2000
        // (6000/2^15) * 21.83A = 4A 
        pidqRef->q = __builtin_mulss(potVal, -6000)>>15; 
    }    

}


void RB_ControlDStepISR(MC_DQ_T idqFdb, MC_DQ_T idqRef, 
        MC_PISTATE_T *pidCtrl, MC_DQ_T *pvdqCmd)
{
    uint16_t temp;
    
}
