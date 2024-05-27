/**
 * sat_PI.h
 * 
 * Module to detect current and voltage saturation. It also contains modified PI controller   
 * 
 * Component: FOC
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
#include "sat_PI_types.h"
#include "motor_control_types.h"

#ifndef SAT_PI_H
#define SAT_PI_H

#ifdef __cplusplus
extern "C" {
#endif



/**
 * Initializes parameters related to saturation detect
 * @param psat saturation detect state
 */
void MCAF_SatInit(MCAF_SAT_DETECT_T *psat);

/**
 * Updates status of saturation state flag
 * 
 * Summary : Saturation Detect
 * 
 * @param psat saturation detect state
 * @param pidq current vector Idq  
 * @param pvdq voltage vector Vdq
 * @param vDC DC link voltage
 *        (needed because voltage saturation limits are expressed 
 *         as a fraction of DC link voltage)
 */
void MCAF_SatDetect(MCAF_SAT_DETECT_T *psat, const MC_DQ_T *pidq, const MC_DQ_T *pvdq, int16_t vDC);

/**
 * Type-safe masking of saturation flags
 *
 * @param value input saturation status
 * @param mask allowable saturation status
 * @return masked saturation status
 */
inline static MCAF_SAT_STATE_T MCAF_SatMask(MCAF_SAT_STATE_T status, MCAF_SAT_STATE_T mask)
{
    return (MCAF_SAT_STATE_T)((uint16_t)status & (uint16_t)mask);
}

/**
 * Computes PI correction. 
 * This function computes the output of a modified PI controller. The
 * proportional term is calculated and saturated, and the integrator term
 * with anti-windup is conditionally updated under conditions determined
 * by the sat_State, inReference, inMeasure and direction inputs.
 * 
 * The integrator term is conditionally updated when either of two 
 * conditions are true:
 * - sat_State is MCAF_SAT_NONE
 * - the error term and the direction are opposite polarities, that is:
 *      inReference <= inMeasure when direction >= 0
 *   or inReference >= inMeasure when direction < 0.
 * 
 * Summary : Modified PI controller
 * 
 * @param inReference Reference Input
 * @param inMeasure Measured Input
 * @param state PI controller state variables
 * @param sat_State status of saturation state to support anti-windup 
 * @param out output of PI controller where result is stored
 * @param direction sign of command, >=0 for positive, <0 for negative
 */
void MCAF_ControllerPIUpdate(int16_t inReference, int16_t inMeasure, 
        MCAF_PISTATE_T *state, MCAF_SAT_STATE_T sat_State, int16_t *out,
        int16_t direction);

/**
 * Invert the integrator value.
 * This is required in some estimators.
 * 
 * @param state PI controller state
 */
inline static void MCAF_ControllerPIIntegratorInvert(MCAF_PISTATE_T *state)
{
    state->integrator = -state->integrator;
}

#ifdef __cplusplus
}
#endif

#endif /* SAT_PI_H */