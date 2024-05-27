/**
 * adc_compensation.c
 * 
 * ADC compensation
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

#include <stdint.h>
#include "util.h"
#include "adc_compensation.h"
#include "system_state.h"
#include "parameters/adc_params.h"
#include "hal.h"
#include "current_measure.h"

inline static uint16_t adcScaleVdc(uint16_t raw)
{
    const uint32_t vdcscaled = UTIL_muluu(raw, MCAF_VDC_SCALING_FACTOR);
    // Saturate if we can't shift right into a uint16_t
    return UTIL_SatShrU16(vdcscaled, MCAF_VDC_SCALING_FACTOR_Q);
}

/**
 * Reads ADC samples for current phases, auxiliary analog inputs,
 * and routes results to appropriate area in motor data structure.
 *
 * @param pmotor motor data
*/
void MCAF_ADCRead(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_ADCCurrentRead(&pmotor->currentMeasure, &pmotor->iabc);
    
    MCAF_ADCApplyCurrentCompensation(&pmotor->currentCalibration, &pmotor->iabc);

    uint16_t unipolarADCResult = HAL_ADC_UnsignedFromSignedInput(HAL_ADC_ValueDclink());
    if (MCAF_ADCIsVdcScaled())
    {
        unipolarADCResult = adcScaleVdc(unipolarADCResult);
    }
    pmotor->psys->vDC = unipolarADCResult >> 1;    
    pmotor->vDC = pmotor->psys->vDC;

    if (!MCAF_OverrideVelocityCommand(&pmotor->testing))
    {
        pmotor->velocityControl.velocityCmd = pmotor->velocityControl.velocityCmdApi;
    }
}

/**
 * Reads ADC samples that are not time critical
 * and routes results to appropriate area in motor data structure.
 *
 * @param pmotor motor data
*/
void MCAF_ADCReadNonCritical(MCAF_MOTOR_DATA *pmotor)
{    
    pmotor->potInput = HAL_ADC_UnsignedFromSignedInput(HAL_ADC_ValuePotentiometer());
    /* The default ADC result is bipolar with 0 counts =
     * the middle of the input voltage range.
     * VDC sensing is an exception to this rule.
     */
 
    if (HAL_ADC_IsAvailableBridgeTemperature())
    {
        const uint16_t raw = HAL_ADC_ValueBridgeTemperature();
        
        MCAF_BRIDGE_TEMPERATURE *pbtemp = &pmotor->bridgeTemperature;
        pbtemp->raw = raw;
        pbtemp->processed = UTIL_MulUUQ16(raw, pbtemp->gain) - pbtemp->offset;
        const int32_t dT = UTIL_mulus(pbtemp->filter.gain, pbtemp->processed)
                         - UTIL_mulus(pbtemp->filter.gain, pbtemp->filter.state.x16.hi);
        const int16_t dT_limited = UTIL_LimitS32ToS16(dT, pbtemp->filter.slewRate);
        pbtemp->filter.state.x32 += dT_limited;
        pbtemp->filter.output = pbtemp->filter.state.x16.hi;                
    }
    if (HAL_ADC_IsAvailableAbsoluteReferenceVoltage())
    {
        pmotor->vAbsRef = HAL_ADC_ValueAbsoluteReferenceVoltage();
    }
}

