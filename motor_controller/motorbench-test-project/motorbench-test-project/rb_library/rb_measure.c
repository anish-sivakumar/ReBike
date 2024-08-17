#include "rb_measure.h"
#include "hal/hardware_access_functions.h"
#include "adc/adc1.h"
#include "motorBench/util.h"
#include "rb_foc_params.h"


void RB_ADCCalibrationInit(RB_MEASURE_CURRENT_T *pcalib)
{
    pcalib->sumIa = 0;
    pcalib->offsetIa = 0;
    
    pcalib->sumIb = 0;
    pcalib->offsetIb = 0;
    
    pcalib->sumIdc = 0;
    pcalib->offsetIdc = 0;
    
    pcalib->sumVa = 0;
    pcalib->offsetVa = 0;
    
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
    pcalib->rawIdc = MCC_ADC_ConversionResultGet(MCAF_ADC_DCLINK_CURRENT);
    pcalib->rawVa = MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEA_VOLTAGE);
    
    // sum current values
    pcalib->sumIa += pcalib->rawIa;
    pcalib->sumIb += pcalib->rawIb;
    pcalib->sumIdc += pcalib->rawIdc;
    pcalib->sumVa += pcalib->rawVa;
    
    pcalib->calibCounter++;
    
    // if we have summed enough samples, calculate offset average
    if (pcalib->calibCounter >= CURRENT_OFFSET_COUNT_MAX)
    {
        pcalib->offsetIa = (int16_t)(pcalib->sumIa >> CURRENT_OFFSET_COUNT_BITS);
        pcalib->offsetIb = (int16_t)(pcalib->sumIb >> CURRENT_OFFSET_COUNT_BITS);
        pcalib->offsetIdc = (int16_t)(pcalib->sumIdc >> CURRENT_OFFSET_COUNT_BITS);
        pcalib->offsetVa = (int16_t)(pcalib->sumVa >> CURRENT_OFFSET_COUNT_BITS);

        pcalib->calibCounter = 0;
        pcalib->sumIa = 0;
        pcalib->sumIb = 0;
        pcalib->sumIdc = 0;
        pcalib->sumVa = 0;
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
    
    //3. read DC bus voltage and current
    uint16_t unsignedVdc = HAL_ADC_UnsignedFromSignedInput(MCC_ADC_ConversionResultGet(MCAF_ADC_DCLINK_VOLTAGE));
    *pvDC = unsignedVdc >> 1;  // Need to divide by two for some reason
    
    /** 4. read DC bus current and filter
     *      Board at standstill draws 0.1A & ADC reads 360 
     *      offset of 435 for inverting op amp gives processed
     *      ADC reading = 435 - 360 at standstill = 75 = 0.1A
     */
    int16_t rawIdc = (int16_t)(MCC_ADC_ConversionResultGet(MCAF_ADC_DCLINK_CURRENT));
    *piDC = RB_ADCCompensate(rawIdc, 435);
    
    //5. read bridge temp - apply offset and gain to get Celsius
    int16_t rawTemp = (int16_t)((MCC_ADC_ConversionResultGet(MCAF_ADC_BRIDGE_TEMPERATURE))>>1);
    *pbridgeTemp = (int16_t)(__builtin_mulss((rawTemp - 4964), Q15(0.010071108)) >> 15); //3.3V/(32767*0.01V)
    
    //6. read phase voltages, and apply offset just for Phase A 
    pvabc->a = ((int16_t)MCC_ADC_ConversionResultGet(MCAF_ADC_PHASEA_VOLTAGE)) - pcalib->offsetVa;
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
    
    // check phase currents
    if ((UTIL_Abs16(piabc->a) <= RB_PHASECURRENT_MAX) || (UTIL_Abs16(piabc->b) <= RB_PHASECURRENT_MAX))
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


void RB_CalculateMotorOutput(int16_t *ppower, int16_t *ptorque, uint16_t *pomega, 
        int16_t iqFdb, uint16_t speed)
{
        
    /**
     * Torque = (3/2) * Ke(V/rad/s) * Iq_Q15
     *              --> Ke = 1/(8.5 RPM/V * 0.10472), 
     *                  using 8.5 instead of 8.2 for lower power estimation
     *        = (3/2) * 1.123444 * Iq_Q15
     *        = 1.685166 * Iq_Q15 
     *        = 1.685166 * (Iq_Q15/750 scaling factor) [Nm]
     *        = 0.002246888 * Iq_Q15
     * https://www.mathworks.com/help/sps/ref/pmsm.html
     */
    *ptorque = (__builtin_mulss(Q15(0.002246888), -iqFdb))>>10; // in 2^5Nm
    
    /**
     * Angular speed omega calculated from hall speed
     *      1RPM = 0.104rad/s
     */
    *pomega = (__builtin_mulss(speed, Q15(0.10472)))>>15; 
            
    /**
     * Motor Power = torque (Nm) x speed (rad/s)
     */         
    *ppower = __builtin_mulus(*pomega, *ptorque)>>5; // in Watts
}