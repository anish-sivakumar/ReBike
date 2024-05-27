/**
 * current_measure.c
 * 
 * Current measurement: multi-channel
 *
 * Enables FOC with currents read from phase current sensors
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
#include "current_measure.h"
#include "system_state.h"
#include "hal.h"
#include "motor_control.h"
#include "motor_control_function_mapping.h"
#include "parameters/hal_params.h"
#include "parameters/adc_params.h"
#include "parameters/options.h"

void MCAF_ADCCompensationInit(MCAF_MOTOR_INITIALIZATION *pinit, 
                              MCAF_CURRENT_COMPENSATION_PARAMETERS *pcal)
{
    /* Scaling constants: Determined by calibration or hardware design. */
                   
    pcal->qKaa = CURRENT_KAA;
    pcal->qKab = CURRENT_KAB;         /* cross-axis gain compensation terms */
    pcal->qKba = CURRENT_KBA;         /* cross-axis gain compensation terms */
    pcal->qKbb = CURRENT_KBB;
    pcal->qKcc = CURRENT_KCC;

    pinit->sampleCount = 0;
    pinit->sampleCountLimit = MCAF_CAL_COUNT;
    pinit->offsetLPF[0].x32 = 0;
    pinit->offsetLPF[1].x32 = 0;
    pinit->offsetLPF[2].x32 = 0;
    
    pinit->kfilter = MCAF_CAL_FILTER_GAIN;
}

inline static void MCAF_ScaleQ15(const MC_ABC_T *pabc_in, MC_ABC_T *pabc_out, int16_t k)
{
    pabc_out->a = UTIL_MulQ15(pabc_in->a, k);
    pabc_out->b = UTIL_MulQ15(pabc_in->b, k);
    pabc_out->c = UTIL_MulQ15(pabc_in->c, k);
}

inline static void calculateManualZeroSequenceModulation(
    const MC_ABC_T *pabc_in, MC_ABC_T *pabc_out, int16_t delta, 
    int16_t min, int16_t max)
{
    pabc_out->a = MC_adjust_zero_sequence(pabc_in->a, delta, min, max);
    pabc_out->b = MC_adjust_zero_sequence(pabc_in->b, delta, min, max);
    pabc_out->c = MC_adjust_zero_sequence(pabc_in->c, delta, min, max);    
}

void MCAF_ComputeDutyCycleOutputs(MCAF_MOTOR_DATA *pmotor)
{
    /* Calculate scaled PWM duty cycles from Va,Vb,Vc and PWM period */
    if (!MCAF_OverrideZeroSequenceModulation(&pmotor->testing))
    {
        MC_CalculateZeroSequenceModulation(&pmotor->dabcUnshifted,
            &pmotor->dabc,
            HAL_PARAM_MIN_DUTY_Q15, HAL_PARAM_MAX_DUTY_Q15);
    }
    else
    {
        calculateManualZeroSequenceModulation(&pmotor->dabcUnshifted, &pmotor->dabc,
                MCAF_GetOverrideZeroSequenceOffset(&pmotor->testing),
                HAL_PARAM_MIN_DUTY_Q15, HAL_PARAM_MAX_DUTY_Q15);
    }
    MCAF_ScaleQ15(&pmotor->dabc, &pmotor->pwmDutycycle, 
        HAL_PARAM_PWM_PERIOD_COUNTS);
}

