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


void RB_ADCReadStepISR(RB_MEASURE_CURRENT_T *pcalib, MC_ABC_T *piabc, 
        int16_t *pvDC, int16_t *piDC, MC_ABC_T *pvabc, uint16_t *pbridgeTemp)
{
    //1. read phase A and B current into current compensation structure
    /* ADC buffer is unsigned and inverted (higher value means negative current)
     * We convert the unsigned buffer to a signed value, that is still inverted 
     * The invert will be handled discretely when offset is applied.
     */ 
    pcalib->rawIa = MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEA_CURRENT);
    pcalib->rawIb = MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEB_CURRENT);
    
    //2. apply current offset compensation
    piabc->a = RB_ADCCompensate(pcalib->rawIa, pcalib->offsetIa);
    piabc->b = RB_ADCCompensate(pcalib->rawIb, pcalib->offsetIb);
    *piDC =  RB_ADCCompensate((MCC_ADC_ConversionResultGet(MCAF_ADC_DCLINK_CURRENT)), 0);
    
    //3. read DC link voltage
    uint16_t unsignedVdc = HAL_ADC_UnsignedFromSignedInput(MCC_ADC_ConversionResultGet(MCAF_ADC_DCLINK_VOLTAGE));
    *pvDC = unsignedVdc >> 1;  // seems like we need to divider by two for the unsigned values    
    
    //4. read bridge temp - apply offset and gain to get Celsius
    int16_t rawTemp = (int16_t)((MCC_ADC_ConversionResultGet(MCAF_ADC_BRIDGE_TEMPERATURE))>>1);
    *pbridgeTemp = (int16_t)(__builtin_mulss((rawTemp - 4964), Q15(0.010071108)) >> 15); //3.3V/(32767*0.01V)
    
    
    //5. read phase Voltages
    pvabc->a = (int16_t)MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEA_VOLTAGE);
    pvabc->b = (int16_t)MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEB_VOLTAGE);
    pvabc->c = (int16_t)MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEC_VOLTAGE);
    
}


void RB_FaultInit(RB_FAULT_DATA *state)
{
    state->isFault = 0;
    state->faultType = 0;
}


bool RB_PhaseCurrentFault(MC_ABC_T *piabc)
{
    bool tempFault = true; //assume faulted
    
    // check phase current
    if (UTIL_Abs16(piabc->a) <= RB_PHASECURRENT_MAX) 
    {
        tempFault = false;
    } else if (UTIL_Abs16(piabc->b) <= RB_PHASECURRENT_MAX)
    {
        tempFault = false;
    }
    
    return tempFault;
}

bool RB_BridgeTempFault(uint16_t temp)
{
    bool tempFault = true; //assume faulted
    
    // check temp
    if (temp < RB_BRIDGETEMP_MAX)
    {
        tempFault = false; 
    } else
    {
        tempFault = false;
    }
    return tempFault;
}

void RB_FaultCheck(RB_FAULT_DATA *pstate, MC_ABC_T *piabc, uint16_t bridgeTemp)
{
    bool tempFault = true; //assume there is a fault and check to deny that
    
    if (RB_PhaseCurrentFault(piabc))
    {
        pstate->faultType = RBFAULT_PHASE_OVERCURRENT;
    } else if (RB_BridgeTempFault(bridgeTemp))
    {
        
        pstate->faultType = RBFAULT_BRIDGE_OVERTEMP;
    } else
    {
        tempFault = false;
    }

    pstate->isFault = tempFault;
}
