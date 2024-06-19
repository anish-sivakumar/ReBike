/*
 * File:   rb_control.c
 * Author: siani
 *
 * Created on June 5, 2024, 4:10 PM
 */

#include "rb_control.h"

void RB_InitControlParameters(RB_MOTOR_DATA *pPMSM)
{
    /* ============= Motor Parameters =============== */
    pPMSM->motorParams.rs = RB_RS;
    pPMSM->motorParams.ke = RB_KE;
    
    /* ============= PI D Term =============== */
    pPMSM->idCtrl.kp = RB_DKP;
    pPMSM->idCtrl.ki = RB_DKI;
    pPMSM->idCtrl.kc = RB_DKC;
    pPMSM->idCtrl.outMax = RB_DOUTMAX;
    pPMSM->idCtrl.outMin = RB_DOUTMIN;

    /* ============= PI Q Term =============== */
    pPMSM->iqCtrl.kp = RB_QKP;
    pPMSM->iqCtrl.ki = RB_QKI;
    pPMSM->iqCtrl.kc = RB_QKC;
    pPMSM->iqCtrl.outMax = RB_QOUTMAX;
    pPMSM->iqCtrl.outMin = RB_QOUTMIN;
    
    /* ============= PI Speed Terms - Do Later =============== */
    
    /* ============= Bridge Temperature Terms =============== */
    pPMSM->bridgeTemperature.gain   = RB_BRIDGE_TEMPERATURE_GAIN;
    pPMSM->bridgeTemperature.offset = RB_BRIDGE_TEMPERATURE_OFFSET;
    pPMSM->bridgeTemperature.filter.gain = RB_BRIDGE_TEMPERATURE_FILTER_GAIN;
    pPMSM->bridgeTemperature.filter.state.x32 = 0;
    pPMSM->bridgeTemperature.filter.output = 0;
    pPMSM->bridgeTemperature.filter.slewRate = RB_BRIDGE_TEMPERATURE_SLEW_RATE;

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


void RB_SetCurrentReference(uint16_t potVal, MC_DQ_T *pidqRef)
{
    // d-axis current controlled at zero
    pidqRef->d = 0;
    
    pidqRef->q = potVal; 

}


void RB_ControlDStepISR(MC_DQ_T idqFdb, MC_DQ_T idqRef, 
        MC_PISTATE_T *pidCtrl, MC_DQ_T *pvdqCmd)
{
    uint16_t temp;
    
}
