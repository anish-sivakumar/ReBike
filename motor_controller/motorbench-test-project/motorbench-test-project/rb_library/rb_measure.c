#include "rb_measure.h"
#include "../motorBench/hal/hardware_access_functions.h"
#include "../mcc_generated_files/adc/adc1.h"


void RB_ADCCompensationInit(RB_MEASURE_CURRENT_T *pcal)
{
    /* Scaling constants: Determined by calibration or hardware design. */
    pcal->qKaa = C_KAA;
    pcal->qKab = C_KAB;         /* cross-axis gain compensation terms */
    pcal->sumIa = 0;
    pcal->offsetIa = 0;
    
    pcal->qKba = C_KBA;         /* cross-axis gain compensation terms */
    pcal->qKbb = C_KBB;
    pcal->sumIb = 0;
    pcal->offsetIb = 0;
    
    pcal->qKidc = 0; // not sure for now, deal with Idc later
    pcal->sumIdc = 0;
    pcal->offsetIdc = 0;
    
    pcal->calibrationCounter = 0;
    pcal->calibrationComplete = 0;
            

}

/**
* <B> Function: MCAPP_MeasureCurrentOffset(MCAPP_MEASURE_CURRENT_T *)  </B>
*
* @brief Function to compute current offset after measuring specified number of
*        current samples and averaging them.
*        .
* @param Pointer to the data structure containing measured current.
* @return none.
* @example
* <CODE> MCAPP_MeasureCurrentOffset(&current); </CODE>
*
*/
void RB_MeasureCurrentOffset(RB_MEASURE_CURRENT_T *pcal)
{
    
    pcal->sumIa += pcal->rawIa;
    pcal->sumIb += pcal->rawIb;
    pcal->calibrationCounter++;
    
    // if we have summed enough samples, calculate offset
    if (pcal->calibrationCounter >= CURRENT_OFFSET_COUNT_MAX)
    {
        pcal->offsetIa = (int16_t)(pcal->sumIa >> CURRENT_OFFSET_COUNT_BITS);
        pcal->offsetIb = (int16_t)(pcal->sumIb >> CURRENT_OFFSET_COUNT_BITS);

        pcal->calibrationCounter = 0;
        pcal->sumIa = 0;
        pcal->sumIb = 0;
        pcal->calibrationComplete = 1;
    }
}

void RB_ADCRead(RB_MEASURE_CURRENT_T *pcal, MC_ABC_T *piabc, int16_t *pvDC)
{
    //1. read phase A and B current into current compensation structure
    pcal->rawIa = MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEA_CURRENT);
    pcal->rawIb = MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEB_CURRENT); 
    
    //2. apply current offset compensation if already complete
    int16_t tempIa = pcal->rawIa;
    int16_t tempIb = pcal->rawIb;
    if(pcal->calibrationComplete == 1)
    {
        tempIa = RB_ADCProcess(pcal->rawIa, pcal->offsetIa);
        tempIb = RB_ADCProcess(pcal->rawIb, pcal->offsetIb);
    }
    
    piabc->a =  (__builtin_mulss(tempIa, pcal->qKaa)
                +__builtin_mulss(tempIb, pcal->qKab)) >> 15;
    piabc->b =  (__builtin_mulss(tempIa, pcal->qKba)
                +__builtin_mulss(tempIb, pcal->qKbb)) >> 15;
    
    //3. read DC link voltage
    uint16_t unsignedVdc = HAL_ADC_UnsignedFromSignedInput(MCC_ADC_ConversionResultGet(MCAF_ADC_DCLINK_VOLTAGE));
    *pvDC = unsignedVdc >> 1; //vDC is signed - I don't understand
      
}