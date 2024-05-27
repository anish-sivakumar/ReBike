/**
 * filter.h
 *
 * Module to perform first order low-pass and high-pass filters
 *
 * Component: miscellaneous
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

#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>
#include "filter_types.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Initialize state variables of LPF
 *
 * @param piirfilter LPF state
 */
inline static void MCAF_LpfFilterInitx16(MCAF_LPF_FILTER_X16_T *pfilterx16)
{
    pfilterx16->stateVar = 0;
    pfilterx16->output = 0;
}

/**
 * Initialize state variables of 16-bit low-pass filter
 *
 * @param pfilter LPF state
 * @param pfilter filter coefficient
 */ 
inline static void MCAF_FilterLowPassS16Init(MCAF_FILTER_LOW_PASS_S16_T *pfilter, uint16_t coeff)
{
    pfilter->state.x32 = 0;
    pfilter->coeff     = coeff;
}

 /**
 * Initialize state variables of HPF
 *
 * @param piirfilter HPF state
 */
inline static void MCAF_HpfFilterInitx16(MCAF_HPF_FILTER_X16_T *pfilterx16)
{
    pfilterx16->stateVar = 0;
    pfilterx16->output = 0;
}
    
/**
 * 16 bit implementation of low pass filter
 * y[n] += (x[n]-y[n-1])*coeff
 *
 * @param pfilterx16 LPF state
 * @param input 1.15 fixed point
 * @return Output of low pass filter
 */
inline static int16_t MCAF_LpfFilterx16(MCAF_LPF_FILTER_X16_T *pfilterx16, int16_t input)
{
    pfilterx16->stateVar += __builtin_mulss(input, pfilterx16->coeff);
    pfilterx16->stateVar -= __builtin_mulss(pfilterx16->output, pfilterx16->coeff);
    const int16_t output = (int16_t) (pfilterx16->stateVar >> 15);
    pfilterx16->output = output;
    return output;
}

/**
 * 16 bit implementation of low pass filter
 * y[n] += (x[n]-y[n-1])*coeff where coeff is represented as Q0.16 unsigned
 *
 * @param pfilter LPF state
 * @param input any 16-bit fixed-point representation
 * @return Output of low pass filter, same representation as input
 */
inline static int16_t MCAF_FilterLowPassS16Update(MCAF_FILTER_LOW_PASS_S16_T *pfilter, int16_t input)
{
    /* This is an efficient implementation of a low-pass filter that is not
     * subject to overflow. It uses an unsigned Q16 coefficient for execution
     * speed, to avoid the shift required for Q15, etc.
     * 
     * The state update is computed with two separate multiplies rather than
     * the easier y[k] = y[k-1] + coeff * (x[k]-y[k-1]) with a single multiply,
     * so that it supports the case where x[k] and y[k-1] are different by more
     * than 32767 counts (example: x[k] = +20000 and y[k-1] = -20000).
     */
    const int16_t previousOutput = pfilter->state.x16.hi;
    pfilter->state.x32 -= __builtin_mulus(pfilter->coeff, previousOutput);
    pfilter->state.x32 += __builtin_mulus(pfilter->coeff, input);
    const int16_t newOutput = pfilter->state.x16.hi;
    return newOutput;
}

/**
 * Get low-pass filter output
 *
 * @return Output of low pass filter
 */
inline static int16_t MCAF_FilterLowPassS16Output(const MCAF_FILTER_LOW_PASS_S16_T *pfilter)
{
    return pfilter->state.x16.hi;
}

/**
 * 16 bit implementation of high pass filter
 * L[n] += (x[n]-L[n-1])*coeff
 * y[n] = x[n]-L[n]
 *
 * @param pfilterx16 HPF state
 * @param input 1.15 fixed point
 * @return Output of high pass filter
 */
inline static int16_t MCAF_HpfFilterx16(MCAF_HPF_FILTER_X16_T *pfilterx16, int16_t input)
{
    // Perform a low pass filter and subtract it from the input.
    const int16_t lpf_shift_prev = (int16_t) (pfilterx16->stateVar >> 16);
    pfilterx16->stateVar -= __builtin_mulss(lpf_shift_prev, pfilterx16->coeff);
    pfilterx16->stateVar += __builtin_mulss(input, pfilterx16->coeff);
    const int16_t lpf_shift = (int16_t) (pfilterx16->stateVar >> 16);
    const int16_t output = UTIL_SatSubS16(input, lpf_shift);
    pfilterx16->output = output;
    return output;
}

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H */
