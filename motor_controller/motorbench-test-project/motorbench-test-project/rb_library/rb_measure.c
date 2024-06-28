#include "rb_measure.h"
#include "hal/hardware_access_functions.h"
#include "adc/adc1.h"
#include "motorBench/util.h"
#include "rb_foc_params.h"


void RB_ADCCalibrationInit(RB_MEASURE_CURRENT_T *pcalib)
{
    /* Scaling constants: Determined by calibration or hardware design. */
    pcalib->qKaa = C_KAA;
    pcalib->qKab = C_KAB;         /* cross-axis gain compensation terms */
    pcalib->sumIa = 0;
    pcalib->offsetIa = 0;
    
    pcalib->qKba = C_KBA;         /* cross-axis gain compensation terms */
    pcalib->qKbb = C_KBB;
    pcalib->sumIb = 0;
    pcalib->offsetIb = 0;
    
    pcalib->qKidc = 0; // not sure for now, deal with Idc later
    pcalib->sumIdc = 0;
    pcalib->offsetIdc = 0;
    
    pcalib->calibCounter = 0;
    pcalib->done = false;
           
}


void RB_ADCCalibrationStepISR(RB_MEASURE_CURRENT_T *pcalib)
{
    
    /* Read phase A and B current into current compensation structure
     * ADC buffer is unsigned and inverted (higher value means negative current)
     * We convert the unsigned buffer to a signed value, that is still inverted 
     * The invert will be handled discretely when offset is applied.
     */ 
    pcalib->rawIa = MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEA_CURRENT);
    pcalib->rawIb = MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEB_CURRENT);
    
    // sum current values
    pcalib->sumIa += pcalib->rawIa;
    pcalib->sumIb += pcalib->rawIb;
    
    pcalib->calibCounter++;
    
    // if we have summed enough samples, calculate offset average
    if (pcalib->calibCounter >= CURRENT_OFFSET_COUNT_MAX)
    {
        pcalib->offsetIa = (int16_t)(pcalib->sumIa >> CURRENT_OFFSET_COUNT_BITS);
        pcalib->offsetIb = (int16_t)(pcalib->sumIb >> CURRENT_OFFSET_COUNT_BITS);

        pcalib->calibCounter = 0;
        pcalib->sumIa = 0;
        pcalib->sumIb = 0;
        pcalib->done = true;
    }
}


void RB_ADCReadStepISR(RB_MEASURE_CURRENT_T *pcalib, MC_ABC_T *piabc, int16_t *pvDC)
{
    //1. read phase A and B current into current compensation structure
    /* ADC buffer is unsigned and inverted (higher value means negative current)
     * We convert the unsigned buffer to a signed value, that is still inverted 
     * The invert will be handled discretely when offset is applied.
     */ 
    pcalib->rawIa = MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEA_CURRENT);
    pcalib->rawIb = MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEB_CURRENT); 
    
    //2. apply current offset compensation
    piabc->a = RB_ADCCompensate(pcalib->rawIa, pcalib->offsetIa, pcalib->qKaa);
    piabc->b = RB_ADCCompensate(pcalib->rawIb, pcalib->offsetIb, pcalib->qKbb);
    
    //3. read DC link voltage
    uint16_t unsignedVdc = HAL_ADC_UnsignedFromSignedInput(MCC_ADC_ConversionResultGet(MCAF_ADC_DCLINK_VOLTAGE));
    *pvDC = unsignedVdc >> 1; //vDC is signed - I don't understand   
}


void RB_FaultInit(RB_FAULT_DATA *state)
{
    state->isFault = 0;
    state->faultType = 0;
}


bool phaseCurrentFault(MC_ABC_T *piabc)
{
    bool tempFault = true; //assume faulted
    
    if (UTIL_Abs16(piabc->a) <= RB_PHASECURRENT_MAX) 
    {
        tempFault = false;
    } else if (UTIL_Abs16(piabc->b) <= RB_PHASECURRENT_MAX)
    {
        tempFault = false;
    }
    
    return tempFault;
}

bool bridgeTempFault(void)
{
    return false;
}

void RB_FaultCheck(RB_FAULT_DATA *pstate, MC_ABC_T *piabc)
{
    bool tempFault = true; //assume there is a fault and check to deny that
    
    if (phaseCurrentFault(piabc))
    {
        pstate->faultType = RBFAULT_PHASE_OVERCURRENT;
    } else if (bridgeTempFault())
    {
        pstate->faultType = RBFAULT_BRIDGE_OVERTEMP;
    } else
    {
        tempFault = false;
    }

    pstate->isFault = tempFault;
}
