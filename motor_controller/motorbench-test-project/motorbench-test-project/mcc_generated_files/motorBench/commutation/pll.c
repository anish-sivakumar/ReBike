/**
 * pll.c
 * 
 * Hosts components of the PLL estimator
 * 
 * Component: commutation
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
#include "parameters/pll_params.h"
#include "parameters/operating_params.h"
#include "parameters/motor_params.h"
#include "motor_control.h"
#include "motor_control_function_mapping.h"
#include "pll.h"
#include "system_state.h"
#include "math_asm.h"
#include "commutation/common.h"

inline static bool useNegatedEsd(const MCAF_ESTIMATOR_PLL_T *pll)
{
    const MCAF_U_VELOCITY_ELEC omega = pll->output.omegaElectrical;
    if (UTIL_Abs16(omega) > DECIMATE_BASE_SPEED)
    {
        /* At speed greater than decimation speed, calculate the estimated
         * velocity based on:
         *  OmegaMr = (1/ke) * (Eqfiltered - sgn(Eqfiltered) * Edfiltered)
         */
        return pll->esdqFiltered.q > 0;
    }
    else
    {
        /* At speed lower than or equal to decimation speed, calculate the estimated
         * velocity based on:
         *  OmegaMr = (1/ke) * (Eqfiltered - sgn(omega) * Edfiltered)
         * to improve stability.
         */        
        return omega > 0;
    }
}

inline static int16_t computeDeltaEs(const MCAF_ESTIMATOR_PLL_T *pll)
{
    /* Calculate the estimated velocity based on the filtered value of BEMF
     * voltage. Up-scale the velocity by a factor of two in order to compensate for
     * the down-scaled BEMF voltage. This scale-down + scale-up arrangement is required to
     * avoid overflow/saturation of BEMF calculation in a few corner cases. 
     * 
     * Compute Eqfiltered - s*Edfilterend,
     * where s is +/- 1 and is determined either from Eqfiltered or omega.
     */
    if (useNegatedEsd(pll))
    {
        return pll->esdqFiltered.q - pll->esdqFiltered.d;
    }
    else
    {
        return pll->esdqFiltered.q + pll->esdqFiltered.d;
    }
}

inline static int16_t computeOmegaMr(const MCAF_ESTIMATOR_PLL_T *pll,
                                     const MCAF_MOTOR_PARAMETERS_T *pmotor)
{
    const int16_t deltaEs = computeDeltaEs(pll);
    
    /* Adjusting the shift count by 1 to compensate for Q14 scaling of the estimated back EMF */
    return UTIL_SatShrS16(UTIL_mulss(pll->keInverse, deltaEs), MCAF_MOTOR_KE_INVERSE_Q - 1);
}

void MCAF_EstimatorPllStep(MCAF_ESTIMATOR_PLL_T *pll, 
                           const MCAF_STANDARD_INPUT_SIGNALS_T *pinput,
                           const MCAF_MOTOR_PARAMETERS_T *pmotor)
{
    MC_ALPHABETA_T deltaI;
    const MCAF_U_CURRENT_ALPHABETA *ialphabeta = &pinput->ialphabeta;
    const MCAF_U_VOLTAGE_ALPHABETA *valphabeta = &pinput->valphabeta;
        
    /* 
     * Calculate voltage drop in stationary (alpha-beta) frame
     * due to inductance and change in current.
     *
     *   vInductance = Ls * (dI/dt)
     *
     * The discrete-time calculation of this voltage is
     *
     *   delta_I = (I[k] - I[k-1])
     *   vInductance = Ls * delta_I / delta_t
     *
     * When evaluating this expression in fixed-point,
     * we use an implicit factor of delta_t = 1 (per-unit)
     * to eliminate the need for an additional divide by delta_t
     * (or multiplication by 1/delta_t). This adds constraints
     * on the fixed-point representations of delta_I, vInductance,
     * and Ls, and as a result, there are several equivalent ways
     * of interpreting the calculation:
     *   
     * Without implicit delta_t = 1 (per-unit):
     *
     *   vInductance = (Ls/delta_t) * delta_I
     *   vInductance = (Ls * delta_I) / delta_t
     *   vInductance = Ls * (delta_I / delta_t)
     *
     * With implicit delta_t = 1 (per-unit):
     *
     *   vInductance = Ls*delta_I
     *
     * For example, suppose the normalization factors are
     *
     *   current (I):      8.8  A
     *   time:            50.0  us    (= timestep)
     *   voltage (V):     52.8  V
     *   dI/dt:          176.0  A/ms   = 8.8A / 50us
     *   inductance (L): 300.0  uH     = (52.8V * 50us)/8.8A
     *   L/t:              6.0  ohm    = 52.8V / 8.8A
     *   flux (V*t):       2.64 mVs    = 52.8V * 50us
     *
     * If Ls = 450uH and delta_I = 0.55A, then the inductive drop is
     * Ls*delta_I/delta_t = 4.95V.
     *   
     * Normalized (or "per-unit") calculations are:
     *
     * Ls              = 1.5
     *                 = 450uH (= 1.5 * 300uH)
     * Ls/delta_t      = 1.5
     *                 = 9 ohm (= 1.5 * 6 ohm = 450uH / 50us)
     * delta_I         = 0.0625
     *                 = 0.55A (= 0.0625*8.8A)
     * delta_I/delta_t = 0.0625
     *                 = 11A/msec (= 0.0625*176A/ms = 0.55A/50us)
     * Ls*delta_I      = 0.09375
     *                 = 247.5uVs (= 0.09375 * 2.64mVs = 450uH*0.55A)
     * vInductance     = 0.09375
     *                 = 4.95V (= 0.09375 * 52.8V)
     *
     *                 = (Ls/delta_t) * delta_I = 1.5 * 0.0625
     *                 = (Ls*delta_I) / delta_t = 0.09375 / 1
     *                 = Ls*(delta_I/delta_t)   = 1.5 * 0.0625
     *                 = Ls*delta_I             = 1.5 * 0.0625
     *
     * Note that calculations that vary by a factor of delta_t
     * have identical per-unit values, even though
     * their actual units are different.
     * (For example Ls = 1.5 = Ls/delta_t,
     * delta_I = 0.0625 = delta_I/delta_t,
     * Ls*delta_I = 0.09375 = vInductance)
     */
    if (UTIL_Abs16(pll->output.omegaElectrical) < pll->omegaFilterThreshold)
    {
        /* At lower speed the granularity of change is higher. Hence, the difference
         * is calculated between two sampled values that are PLL_LOWSPEED_DIBYDT_PRESCALER 
         * number of PLL execution steps apart */
        const MC_ALPHABETA_T *lastIalphabeta = 
                &pll->lastIalphabeta[(pll->diCounter-(PLL_LOWSPEED_DIBYDT_PRESCALER-1))&
                (PLL_LOWSPEED_DIBYDT_PRESCALER-1)];
        deltaI.alpha = (ialphabeta->alpha - lastIalphabeta->alpha);
        deltaI.beta = (ialphabeta->beta - lastIalphabeta->beta);
        
        /* Limit the change in current to dIlimitLS in order to
         * reduce the effect of noisy current measurements */
        deltaI.alpha = UTIL_LimitS16(deltaI.alpha, -pll->dIlimitLS, pll->dIlimitLS);
        deltaI.beta = UTIL_LimitS16(deltaI.beta, -pll->dIlimitLS, pll->dIlimitLS);
        
        /* 
         * vInductance = Ls * (dI/dt) with implicit dt=1 (per-unit)
         */
        pll->vInductance.alpha = UTIL_SatShrS16(__builtin_mulss(pmotor->l0BaseDt, deltaI.alpha),
                                    MCAF_MOTOR_L0_BASE_DT_Q+PLL_LOWSPEED_DIBYDT_PRESCALER_SHIFTCOUNT);
        pll->vInductance.beta = UTIL_SatShrS16(__builtin_mulss(pmotor->l0BaseDt, deltaI.beta),
                                    MCAF_MOTOR_L0_BASE_DT_Q+PLL_LOWSPEED_DIBYDT_PRESCALER_SHIFTCOUNT);
    }
    else
    {
        /* At higher speed the granularity of change is insignificant. Hence, the
         * difference can be calculated between two sampled values that are one PLL
         * execution steps apart */
        const MC_ALPHABETA_T *lastIalphabeta = &pll->lastIalphabeta[pll->diCounter];
        deltaI.alpha = (ialphabeta->alpha - lastIalphabeta->alpha);
        deltaI.beta = (ialphabeta->beta - lastIalphabeta->beta);  

        /* Limit the change in current to dIlimitHS in order to
         * reduce the effect of noisy current measurements */        
        deltaI.alpha = UTIL_LimitS16(deltaI.alpha, -pll->dIlimitHS, pll->dIlimitHS);
        deltaI.beta = UTIL_LimitS16(deltaI.beta, -pll->dIlimitHS, pll->dIlimitHS);        
                
        /* 
         * vInductance = Ls * (dI/dt) with implicit dt=1 (per-unit)
         */
        pll->vInductance.alpha = UTIL_SatShrS16(__builtin_mulss(pmotor->l0BaseDt, deltaI.alpha),
                                            MCAF_MOTOR_L0_BASE_DT_Q);
        pll->vInductance.beta = UTIL_SatShrS16(__builtin_mulss(pmotor->l0BaseDt, deltaI.beta),
                                            MCAF_MOTOR_L0_BASE_DT_Q);
    }

    /* Update the sample history of Ialpha and Ibeta */
    pll->diCounter = (pll->diCounter+1) & (PLL_LOWSPEED_DIBYDT_PRESCALER-1);
    MC_ALPHABETA_T *pialphabetaHistory = &pll->lastIalphabeta[pll->diCounter];
    pialphabetaHistory->alpha = ialphabeta->alpha;
    pialphabetaHistory->beta  = ialphabeta->beta;

    /*
     * If delay matching is desired, add a one-cycle delay for Valphabeta
     * to match the delay in the phase current samples.
     */
    const MCAF_U_VOLTAGE_ALPHABETA *valphabetaCompensated =
         (MCAF_PLL_DELAY_MATCH)
       ? &pll->lastValphabeta
       : valphabeta;
    
    /* Calculate the BEMF voltage:
     *  Ealphabeta = Valphabeta - Rs*Ialphabeta - Ls*(dIalphabeta/dt)
     * and scale it down by a factor of two. This scaling is required 
     * to prevent overflow/saturation of BEMF calculation in a few corner cases.
     */
    pll->irDropAlpha = UTIL_SatShrS16(__builtin_mulss(pmotor->rs, ialphabeta->alpha),
                                      MCAF_MOTOR_RS_Q);
    pll->esalphabeta.alpha = (((int32_t)valphabetaCompensated->alpha) - pll->irDropAlpha
                                                 - pll->vInductance.alpha) >> 1;
    pll->irDropBeta = UTIL_SatShrS16(__builtin_mulss(pmotor->rs, ialphabeta->beta ),
                                     MCAF_MOTOR_RS_Q);
    pll->esalphabeta.beta = (((int32_t)valphabetaCompensated->beta) - pll->irDropBeta
                                                  - pll->vInductance.beta) >> 1;

    if (MCAF_PLL_DELAY_MATCH)
    {
        /* Update previous values of Valphabeta to be used in the next control step */
        pll->lastValphabeta.alpha = valphabeta->alpha;
        pll->lastValphabeta.beta = valphabeta->beta;
    }

    /* Calculate sine and cosine components of the rotor angle */
    MC_CalculateSineCosine(pll->output.thetaElectrical, &pll->sincos);

    /* Transform the calculated BEMF voltage into rotor reference frame using:
     *  Esd =  Ealpha*cos(Angle) + Ebeta*sin(Angle)
     *  Esq = -Ealpha*sin(Angle) + Ebeta*cos(Angle)
     * i.e. equivalent to Park transform
     */
    pll->esdq.d = UTIL_Shr15(__builtin_mulss(pll->esalphabeta.alpha, pll->sincos.cos) + 
                        __builtin_mulss(pll->esalphabeta.beta, pll->sincos.sin));
    pll->esdq.q = UTIL_Shr15(__builtin_mulss(pll->esalphabeta.beta, pll->sincos.cos) - 
                        __builtin_mulss(pll->esalphabeta.alpha, pll->sincos.sin));    
    
    /* Filter the BEMF voltage using a first order low pass filter:
     *  Edqfiltered = 1/TFilterd * Integral{ (Esd-EsdFilter).dt }
     */
    const int16_t ddiff = pll->esdq.d - pll->esdqFiltered.d;
    pll->esdqStateVar.d += __builtin_mulss(ddiff, pll->kEsdqFilter);
    pll->esdqFiltered.d = UTIL_Shr15(pll->esdqStateVar.d);
    const int16_t qdiff = pll->esdq.q - pll->esdqFiltered.q;
    pll->esdqStateVar.q += __builtin_mulss(qdiff, pll->kEsdqFilter);
    pll->esdqFiltered.q = UTIL_Shr15(pll->esdqStateVar.q);

    pll->omegaMr = computeOmegaMr(pll, pmotor);
    
    /* Integrate the estimated velocity to get estimated rotor angle */
    pll->rhoStateVar += __builtin_mulss(pll->omegaMr, pll->dtAngular);
    pll->rho = UTIL_Shr15(pll->rhoStateVar);
    /* Compensate the estimated rotor angle with predetermined offset value */
    pll->output.thetaElectrical = pll->rho + pll->rhoOffset;
    /* Filter the estimated velocity using a first order low-pass filter */
    const int16_t omegadiff = pll->omegaMr - pll->output.omegaElectrical;
    pll->velEstimStateVar += __builtin_mulss(omegadiff, pll->kVelEstimFilter);
    pll->output.omegaElectrical = UTIL_Shr15(pll->velEstimStateVar);

}

void MCAF_EstimatorPllInit(MCAF_ESTIMATOR_PLL_T *pll, 
        const MCAF_MOTOR_PARAMETERS_T *pmotor)
{
    pll->rhoStateVar = 0;
    pll->omegaMr = 0;
    pll->diCounter = 0;
    pll->esdqStateVar.d = 0;
    pll->esdqStateVar.q = 0;
    pll->esdq.d = 0;
    pll->esdq.q = 0;
    pll->dIlimitHS = D_ILIMIT_HS;
    pll->dIlimitLS = D_ILIMIT_LS;
    pll->kEsdqFilter = KFILTER_ESDQ;
    pll->kVelEstimFilter = KFILTER_VELESTIM;
    pll->omegaFilterThreshold = MCAF_PLL_VELOCITY_FILTER_THRESHOLD;

    pll->dtAngular = MCAF_PLL_DT_ANGULAR;
    pll->rhoOffset = INITOFFSET_TRANS_OPEN_CLSD;
    pll->output.thetaElectrical = 0;
    pll->output.omegaElectrical = 0;
    pll->rho = 0;
    
    pll->keInverse = UTIL_MulQ14(MCAF_MOTOR_KE_INVERSE, MCAF_PLL_KE_INVERSE_SCALING);
}

void MCAF_EstimatorPllStartupInit(MCAF_ESTIMATOR_PLL_T *pll)
{
    // do nothing, at least not right now
}
