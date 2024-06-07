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
    MCAF_ADCCompensationInit(&pPMSM->initialization,
                             &pPMSM->currentCalibration); 
    
    pPMSM->rVdc = MCAF_ComputeReciprocalDCLinkVoltage(INT16_MAX);
    
    MCAF_SatInit(&pPMSM->sat);
    
    return true;
}

void RB_ADCRead(const MCAF_CURRENT_COMPENSATION_PARAMETERS *pcal, MC_ABC_T *piabc, int16_t *pvDC)
{
    //1. read phase A and B current
    piabc->a = MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEA_CURRENT);
    piabc->b = MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEB_CURRENT); 
    
    //2. apply current offset compensation. pcal is initialized in RB_FocInit())
    /**
    * Compensation for current measurements.
    * 
    * These are applied as follows:
    * If Ia[0] and Ib[0] are the original ADC measurements, we calculate:
    * (note: sign is correct since our current sensing gain into the ADC is negative)
    * 
    * Ia[1] = -(Ia[0] - offseta)
    * Ib[1] = -(Ib[0] - offsetb)
    * 
    * Ia[final] = qKaa * Ia[1] + qKab * Ib[1]
    * Ib[final] = qKba * Ia[1] + qKbb * Ib[1]
    * 
    * The cross-terms are important to correct cross-coupling of the ADC measurements.
    */
    const int16_t a1 = RB_applyOffset(piabc->a, pcal->offseta, true);
    const int16_t b1 = RB_applyOffset(piabc->b, pcal->offsetb, true);
    piabc->a =  (__builtin_mulss(a1, pcal->qKaa)
                +__builtin_mulss(b1, pcal->qKab)) >> 15;
    piabc->b =  (__builtin_mulss(a1, pcal->qKba)
                +__builtin_mulss(b1, pcal->qKbb)) >> 15;
    
    //3. read DC link voltage
    uint16_t unsignedVdc = HAL_ADC_UnsignedFromSignedInput(MCC_ADC_ConversionResultGet(MCAF_ADC_DCLINK_VOLTAGE));
    *pvDC = unsignedVdc >> 1; //vDC is signed - I don't understand
      
}

