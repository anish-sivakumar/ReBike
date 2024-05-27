/**
 * sat_PI.c
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
 
#include "sat_PI.h"
#include "parameters/sat_PI_params.h"
#include "util.h"
#include "hal.h"
#include "motor_control_types.h"

volatile register int a_Reg asm("A");
volatile register int b_Reg asm("B");

/**
 * Initializes parameters related to voltage saturation
 * 
 * @param pvoltsat voltage saturation state
 */
void MCAF_VoltSatInit(MCAF_SAT_T *pvoltsat)
{
    /* Threshold for coming out of voltage saturation */
    pvoltsat->thresholdDownLimit = UTIL_SignedSqr(VOLTAGE_SATURATION_THRESHOLD_DOWN_CONSTANT);
    /* Threshold for entering voltage saturation */
    pvoltsat->thresholdUpLimit = UTIL_SignedSqr(VOLTAGE_SATURATION_THRESHOLD_UP_CONSTANT);
    pvoltsat->satFlag = 0;
}

/**
 * Initializes parameters related to current saturation 
 * 
 * @param pcurrentsat current saturation state
 */
void MCAF_CurrentSatInit(MCAF_SAT_T *pcurrentsat)
{
    /* Threshold for coming out of current saturation */
    pcurrentsat->thresholdDownLimit = CURRENT_SATURATION_THRESHOLD_DOWN;
    pcurrentsat->thresholdUpLimit = CURRENT_SATURATION_THRESHOLD_UP;
    pcurrentsat->satFlag = 0;
} 

void MCAF_SatInit(MCAF_SAT_DETECT_T *psat)
{
    /* Initialize voltage and current saturation parameters */
    MCAF_VoltSatInit(&psat->voltSat); 
    MCAF_CurrentSatInit(&psat->currentSat);
}

/**
 * Updates latched flag:
 *   INTO_SATURATION sets the latch,
 *   OUT_OF_SATURATION clears the latch,
 *   otherwise we just keep the previous state.
 * 
 * @param latch previous latch value
 * @param satFlag saturation flag
 * @return next latch value
 */
inline static bool updateLatchedFlag(bool latch, uint16_t satFlag)
{
    if (satFlag & MCAF_INTO_SATURATION)
    {
        latch = true;
    }
    else if (satFlag & MCAF_OUT_OF_SATURATION)
    {
        latch = false;
    }
    return latch;
}

/**
 * Monitors Vs (Vd^2 + Vq^2) and indicates voltage saturation status
 * 
 * Summary : Voltage Saturation Detect
 * 
 * @param pvoltsat voltage saturation state
 * @param pvdq voltage vector Vdq
 * @param vDC DC link voltage
 */
inline static void MCAF_VoltSatDetect(MCAF_SAT_T *pvoltsat, const MC_DQ_T *pvdq, int16_t vDC)
{
    /* 
     * uint16_t is used in the following calculations to avoid overflow;
     * since UTIL_SignedSqr returns a number from 0-32767 we can store
     * the sum of two such values in a uint16_t. 
     * 
     * Subsequent comparison operators are valid as long as
     * both operands are uint16_t; we can safely cast an int16_t to uint16_t as long
     * as it is a positive integer (for example, vDCSquared is always positive
     * and pvoltsat->thresholdUpLimit should always be positive, so their
     * product is positive.)
     */
    uint16_t vs = (uint16_t)(UTIL_SignedSqr(pvdq->d))
                + (uint16_t)(UTIL_SignedSqr(pvdq->q));
    const int16_t vDCSquared = UTIL_SignedSqrNoOverflow(vDC);
    if (vs >= (uint16_t)UTIL_MulQ15(pvoltsat->thresholdUpLimit, vDCSquared))
    {
       pvoltsat->satFlag = UTIL_SetBits(pvoltsat->satFlag, MCAF_INTO_SATURATION); 
    }
    else
    {
        pvoltsat->satFlag = UTIL_ClearBits(pvoltsat->satFlag, MCAF_INTO_SATURATION);
    }   
    if (vs < (uint16_t)UTIL_MulQ15(pvoltsat->thresholdDownLimit, vDCSquared))
    {
        pvoltsat->satFlag = UTIL_SetBits(pvoltsat->satFlag, MCAF_OUT_OF_SATURATION); 
    }
    else
    {
        pvoltsat->satFlag = UTIL_ClearBits(pvoltsat->satFlag, MCAF_OUT_OF_SATURATION);
    }   
}

/**
 * Monitors Iq and indicates current saturation status
 * 
 * Summary : Current Saturation Detect
 * @param pcurrentsat current saturation state
 * @param input current Iq 
 */
inline static void MCAF_CurrentSatDetect(MCAF_SAT_T *pcurrentsat, const int16_t input)
{
    if (UTIL_Abs16(input) >= pcurrentsat->thresholdUpLimit)
    {
       pcurrentsat->satFlag = UTIL_SetBits(pcurrentsat->satFlag, MCAF_INTO_SATURATION); 
    }
    else
    {
        pcurrentsat->satFlag = UTIL_ClearBits(pcurrentsat->satFlag, MCAF_INTO_SATURATION);
    }
    if (UTIL_Abs16(input) < pcurrentsat->thresholdDownLimit)
    {
        pcurrentsat->satFlag = UTIL_SetBits(pcurrentsat->satFlag, MCAF_OUT_OF_SATURATION); 
    }
    else
    {
        pcurrentsat->satFlag = UTIL_ClearBits(pcurrentsat->satFlag, MCAF_OUT_OF_SATURATION);
    }
}

void MCAF_SatDetect(MCAF_SAT_DETECT_T *psat, const MC_DQ_T *pidq, const MC_DQ_T *pvdq, int16_t vDC)
{
    MCAF_CurrentSatDetect(&psat->currentSat, pidq->q);
    MCAF_VoltSatDetect(&psat->voltSat, pvdq, vDC);
    
    /*
     * Use the INTO_SATURATION to set, and OUT_OF_SATURATION to reset,
     * each of the voltage and current saturation flags, independently.
     * We take advantage of bitwise operations to do this, which depends on
     * the fact that:
     *     MCAF_SAT_NONE == 0
     * and MCAF_SAT_VOLT | MCAF_SAT_CURRENT == MCAF_SAT_VOLT_AND_CURRENT
     */
    bool satVoltage = updateLatchedFlag(psat->state & MCAF_SAT_VOLT, 
                                        psat->voltSat.satFlag);
    bool satCurrent = updateLatchedFlag(psat->state & MCAF_SAT_CURRENT, 
                                        psat->currentSat.satFlag);
    
    /* Now merge them together into an updated state. */
    if (satVoltage)
    {
        psat->state = (satCurrent) ? MCAF_SAT_VOLT_AND_CURRENT : MCAF_SAT_VOLT;
    }
    else
    {
        psat->state = (satCurrent) ? MCAF_SAT_CURRENT : MCAF_SAT_NONE;
    }
}

/** * subtracts two 16-bit numbers but saturates the results * (requires saturation mode to be set) */
inline static int16_t saturatedSubtract(int16_t x1, int16_t x2)
{
    a_Reg = __builtin_lac(x1, 0);
    b_Reg = __builtin_lac(x2, 0);
    a_Reg = __builtin_subab(a_Reg, b_Reg);
    return __builtin_sacr(a_Reg, 0);
}

/** * Read accumulator A */
inline static int32_t readAccA32()
{
#if __XC16_VERSION__ >= 1026
    const int32_t tmp = __builtin_sacd(a_Reg, 0);
    /* Prevent optimization from re-ordering/ignoring this sequence of operations */
    asm volatile ("");
    return tmp;
#else
    int32_t result;
    /* Prevent optimization from re-ordering/ignoring this sequence of operations */
    asm volatile ("" : "+w"(a_Reg):); 
    result = ACCAH;
    result <<= 16;
    result |= ACCAL; return result;
#endif
}

/** * Write accumulator B */
inline static void writeAccB32(int32_t input)
{
#if __XC16_VERSION__ >= 1026
    const int32_t tmp = input;
    asm volatile ("" :: "r"(tmp)); 
    b_Reg = __builtin_lacd(tmp, 0);
#else
    uint32_t temp_dword;
    uint16_t temp_word;
    temp_dword = 0xFFFF0000 & input;
    temp_dword = temp_dword >> 16;
    temp_word = (uint16_t)temp_dword;
    b_Reg = __builtin_lac(temp_word, 0);
    /* Prevent optimization from re-ordering/ignoring this sequence of operations */
    asm volatile ("" : "+w"(b_Reg):); 
    temp_word = (uint16_t)(0xFFFF & input);
    ACCBL = temp_word;
#endif
}

void MCAF_ControllerPIUpdate(int16_t in_Ref, int16_t in_Meas, 
        MCAF_PISTATE_T *state, MCAF_SAT_STATE_T sat_State, int16_t *out,
        int16_t direction)
{
    int16_t error;
    
    /* non saturated output */
    int16_t out_nonsat;
    /* saturated output */
    int16_t out_sat;
    uint16_t saveCorcon = HAL_CORCON_RegisterValue_Get();
    
    /* Init CORCON register */
    HAL_CORCON_Initialize();
    
    /* Calculate error */
    error = saturatedSubtract(in_Ref, in_Meas);

    /* Read state->integrator into B */
    writeAccB32(state->integrator);

    /* Calculate (Kp * error * 2^Nkp), store in A and out_Buffer */
    a_Reg = __builtin_mpy(error, state->kp, 0, 0, 0, 0, 0, 0);
    a_Reg = __builtin_sftac(a_Reg, -state->nkp);
    a_Reg = __builtin_addab(a_Reg, b_Reg);
    out_nonsat = __builtin_sacr(a_Reg, 0);

    /* Limit the output */
    out_sat = UTIL_LimitS16(out_nonsat, state->outMin, state->outMax);
    
    *out = out_sat;
    /* Calculate integrator term and add it to previous value if not in saturation state */
    if ((sat_State == MCAF_SAT_NONE)
         || (UTIL_DirectedLessThanEqual(in_Ref, in_Meas, direction)))
    {    
        /* Calculate (error * Ki) and store in A */
        a_Reg = __builtin_mpy(error, state->ki, 0, 0, 0, 0, 0, 0);
        a_Reg = __builtin_sftac(a_Reg, -state->nki);
        
        /* Calculate (excess * Kc), subtract from (error * Ki) and store in A */
        error = out_nonsat - out_sat;
        a_Reg = __builtin_msc(a_Reg, error, state->kc,0,0,0,0,0,0,0,0);
        
        /* Add (error * Ki)-(excess * Kc) to the integrator value in B */
        a_Reg = __builtin_addab(a_Reg,b_Reg);
        
        state->integrator = readAccA32();
    }

    HAL_CORCON_RegisterValue_Set(saveCorcon);
    
}