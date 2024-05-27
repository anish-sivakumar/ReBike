/**
 * current_measure.h
 *
 * Current measurement routines: multi-channel
 * 
 * Component: ADC compensation
 */

/* *********************************************************************
 *
 * Motor Control Application Framework
 * R7/RC37 (commit 116330, build on 2023 Feb 09)
 *
 * (c) 2017 - 2023 Microchip Technology Inc. and its subsidiaries. You may use
 * this software and any derivatives exclusively with Microchip products.
 *
 * This software and any accompanying information is for suggestion only.
 * It does not modify Microchip's standard warranty for its products.
 * You agree that you are solely responsible for testing the software and
 * determining its suitability.  Microchip has no obligation to modify,
 * test, certify, or support the software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH
 * MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY
 * APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL,
 * PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF
 * ANY KIND WHATSOEVER RELATED TO THE USE OF THIS SOFTWARE, THE
 * motorBench(R) DEVELOPMENT SUITE TOOL, PARAMETERS AND GENERATED CODE,
 * HOWEVER CAUSED, BY END USERS, WHETHER MICROCHIP'S CUSTOMERS OR
 * CUSTOMER'S CUSTOMERS, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
 * CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
 * OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
 * SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
 * THESE TERMS.
 *
 * *****************************************************************************/

#ifndef __CURRENT_MEASURE_H
#define __CURRENT_MEASURE_H

#include <stdint.h>
#include "units.h"
#include "current_measure_types.h"
#include "system_state.h"
#include "parameters/adc_params.h"

#ifdef __cplusplus
extern "C" {
#endif

inline static int16_t applyOffset(int16_t measurement, int16_t offset, bool invert)
{
    if (invert)
    {
        return offset - measurement;
    }
    else
    {
        return measurement - offset;
    }
}

inline static void MCAF_ADCApplyCurrentCompensation(const MCAF_CURRENT_COMPENSATION_PARAMETERS *pcal,
                                             MC_ABC_T *piabc)
{
    const int16_t a1 = applyOffset(piabc->a, pcal->offseta, MCAF_ADCIsPhaseACurrentInverted());
    const int16_t b1 = applyOffset(piabc->b, pcal->offsetb, MCAF_ADCIsPhaseBCurrentInverted());
    
    piabc->a =  (UTIL_mulss(a1, pcal->qKaa)
                +UTIL_mulss(b1, pcal->qKab)) >> 15;
    piabc->b =  (UTIL_mulss(a1, pcal->qKba)
                +UTIL_mulss(b1, pcal->qKbb)) >> 15;

    if (HAL_ADC_IsAvailableIphaseC())
    {
        const int16_t c1 = applyOffset(piabc->c, pcal->offsetc, MCAF_ADCIsPhaseCCurrentInverted());
        piabc->c = (UTIL_mulss(c1, pcal->qKcc)) >> 15;
    }
}

/**
 * Obtains motor phase current information from appropriate sources.
 * @param currentMeasure MCAF current measurement
 * @param iabc abc current vector
 */
inline static void MCAF_ADCCurrentRead(const MCAF_CURRENT_MEASUREMENT *currentMeasure, MCAF_U_CURRENT_ABC *iabc)
{
    iabc->a = HAL_ADC_ValueIphaseA();
    iabc->b = HAL_ADC_ValueIphaseB(); 
    
    if (HAL_ADC_IsAvailableIphaseC())
    {
        iabc->c = HAL_ADC_ValueIphaseC();
    }
}

/**
 * Executes one step of current offset calibration.
 * During the calibration interval, integrates filtered offset
 * based on the compensated measurement value, but only if it is within
 * range.
 * 
 * @param pLPF pointer to low-pass-filter integrator
 * @param measurement compensated measurement
 * @param k integrator gain
 */
inline static void MCAF_ADCCalibrateCurrentOffset(sx1632_t *pLPF, int16_t measurement, int16_t k, bool invert)
{
    if ((measurement > -MCAF_CAL_RANGE) && (measurement < MCAF_CAL_RANGE))
    {
        asm volatile ("; BEGIN MCAF_ADCCalibrateCurrentOffset" ::);
        const int32_t delta = UTIL_mulss(measurement << MCAF_CAL_SHIFT, k);
        if (invert)
        {
            pLPF->x32 -= delta;
        }
        else
        {
            pLPF->x32 += delta;
        }
        asm volatile ("; END MCAF_ADCCalibrateCurrentOffset" ::);
    }
}

/**
 * Executes one step of ADC current compensation
 * 
 * @param pinit motor initialization state data
 * @param pcal current compensation gains
 * @param piabc measured currents
 */
inline static void MCAF_ADCCalibrateCurrentOffsets(MCAF_MOTOR_INITIALIZATION *pinit, 
                                     MCAF_CURRENT_COMPENSATION_PARAMETERS *pcal,
                                     const MCAF_U_CURRENT_ABC *piabc,
                                     const MCAF_U_CURRENT pidc)
{
    if (pinit->sampleCount < pinit->sampleCountLimit)
    {
        MCAF_ADCCalibrateCurrentOffset(&pinit->offsetLPF[0],
                                        piabc->a,
                                        pinit->kfilter,
                                        MCAF_ADCIsPhaseACurrentInverted());
        MCAF_ADCCalibrateCurrentOffset(&pinit->offsetLPF[1],
                                        piabc->b,
                                        pinit->kfilter,
                                        MCAF_ADCIsPhaseBCurrentInverted());
        if (HAL_ADC_IsAvailableIphaseC())
        {
            MCAF_ADCCalibrateCurrentOffset(&pinit->offsetLPF[2],
                                            piabc->c,
                                            pinit->kfilter,
                                            MCAF_ADCIsPhaseCCurrentInverted());
        }
    
        pcal->offseta = pinit->offsetLPF[0].x16.hi >> MCAF_CAL_SHIFT;
        pcal->offsetb = pinit->offsetLPF[1].x16.hi >> MCAF_CAL_SHIFT;
        if (HAL_ADC_IsAvailableIphaseC())
        {
            pcal->offsetc = pinit->offsetLPF[2].x16.hi >> MCAF_CAL_SHIFT;
        }
        ++pinit->sampleCount;
    }
    else
    {
        pinit->ready = true;
    }
}
/**
 * Initializes state variables for ADC compensation
 * 
 * @param pinit motor initialization state data
 * @param pcal current compensation gains
 */
void MCAF_ADCCompensationInit(MCAF_MOTOR_INITIALIZATION *pinit, 
                              MCAF_CURRENT_COMPENSATION_PARAMETERS *pcal);

/**
 * Calculate scaled PWM duty cycles from Va,Vb,Vc and the PWM period
 * @param pmotor motor data
 */
void MCAF_ComputeDutyCycleOutputs(MCAF_MOTOR_DATA *pmotor);

/**
  * Initialize multi channel state
  * @param pmultiChannel multi channel state
  */        
 inline static void MCAF_CurrentMeasureInit(MCAF_CURRENT_MEASUREMENT *pmultiChannel) {}
 
 /**
  * Reinitialize multi channel state on restart
  * @param pmultiChannel single channel state
  */        
 inline static void MCAF_CurrentMeasureRestart(MCAF_CURRENT_MEASUREMENT *pmultiChannel) {}
 
#ifdef __cplusplus
}
#endif

#endif /* __CURRENT_MEASURE_H */
