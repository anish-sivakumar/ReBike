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

    MCAF_FilterLowPassS16Init(&pPMSM->vqFiltered, RB_FILTER_COEFF_VQ);
}

bool RB_FocInit(RB_MOTOR_DATA *pPMSM)
{
    RB_InitControlLoopState(pPMSM);
    pPMSM->controlFlags = 0; 
    
    /* initialize the mux'd channel (doesn't matter which setting is first) */
    pPMSM->adcSelect = HADC_POTENTIOMETER;
    
    /* initialize ADC compensation parameters  */
    RB_ADCCompensationInit(&pPMSM->currentCalib); 
    
    //pPMSM->rVdc = MCAF_ComputeReciprocalDCLinkVoltage(INT16_MAX);
    
    //MCAF_SatInit(&pPMSM->sat);
    
    return true;
}


