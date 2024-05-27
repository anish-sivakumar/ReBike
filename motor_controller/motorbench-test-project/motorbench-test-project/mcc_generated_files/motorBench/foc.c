/**
 * foc.c
 * 
 * main field-oriented-control code
 * 
 * Component: FOC
 */

#include "filter.h"


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
#include <stddef.h>
#include "util.h"
#include "adc_compensation.h"
#include "parameters/sat_PI_params.h"
#include "parameters/foc_params.h"
#include "parameters/hal_params.h"
#include "parameters/motor_params.h"
#include "parameters/timing_params.h"
#include "parameters/outerloop_params.h"
#include "parameters/options.h"
#include "motor_control.h"
#include "motor_control_function_mapping.h"
#include "system_state.h"
#include "foc.h"
#include "commutation.h"
#include "startup.h"
#include "math_asm.h"
#include "sat_PI.h"
#include "hal.h"
#include "deadtimecomp.h"
#include "flux_control.h"
#include "dyn_current.h"
#include "test_harness.h"
#include "mcapi_internal.h"
#include "commutation_excitation.h"
#include "current_measure.h"

/* ------------------------- Initialization ------------------------- */

int16_t MCAF_ComputeReciprocalDCLinkVoltage(int16_t vdc);

/** 
 * Initializes required parameters 
 * 
 * @param pmotor motor data
 */
inline static void initStateParameters(MCAF_MOTOR_DATA *pmotor)
{

    /* ============= Open Loop Startup ====================== */
    MCAF_StartupTransitioningInit(&pmotor->startup);

    MCAF_ADCCompensationInit(&pmotor->initialization,
                             &pmotor->currentCalibration); 
    MCAF_FluxControlInit(&pmotor->fluxControl);        

    pmotor->rVdc = MCAF_ComputeReciprocalDCLinkVoltage(INT16_MAX);
    
    MCAF_InitializeCurrentFilter(pmotor);
#if MCAF_TRIGGERED_AVERAGE_EXAMPLE == 1
    const uint16_t sampleCount = 256;  // Take 256 samples per averaging routine
    const uint16_t shiftCount = 8;     // Right-shift by 8 (equivalent to dividing by 256)
    MCAF_TriggeredAverage_Init(&pmotor->iqAverage, sampleCount, shiftCount);
#endif
}

/**
 * Initialize controller state
 * 
 * @param pmotor motor data
 */
inline static void initControlLoopState(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->idCtrl.integrator = 0;
    pmotor->iqCtrl.integrator = 0;
    pmotor->omegaCtrl.integrator = 0;
    pmotor->vdqCmd.d = 0;
    pmotor->vdqCmd.q = 0;
    pmotor->idqCmdRaw.d = 0;
    pmotor->idqCmdRaw.q = 0;
}


/**
 * Initialize motor parameters
 * 
 * @param pparam motor parameters
 */
inline static void initMotorParameters(MCAF_MOTOR_PARAMETERS_T *pparam)
{
    pparam->rs = MCAF_MOTOR_RS;
    
    pparam->ldBaseDt = MCAF_MOTOR_LD_BASE_DT;
    pparam->lqBaseDt = MCAF_MOTOR_LQ_BASE_DT;
    pparam->l0BaseDt = MCAF_MOTOR_L0_BASE_DT;
    pparam->l1BaseDt = MCAF_MOTOR_L1_BASE_DT;
    
    pparam->ldBaseOmegaE = MCAF_MOTOR_LD_BASE_OMEGA_E;
    pparam->lqBaseOmegaE = MCAF_MOTOR_LQ_BASE_OMEGA_E;
    pparam->l0BaseOmegaE = MCAF_MOTOR_L0_BASE_OMEGA_E;
    pparam->l1BaseOmegaE = MCAF_MOTOR_L1_BASE_OMEGA_E;
    
    pparam->ke = MCAF_MOTOR_KE;
    pparam->keInverse = MCAF_MOTOR_KE_INVERSE;
}

bool MCAF_FocInit(MCAF_MOTOR_DATA *pmotor)
{
    initMotorParameters(&pmotor->motorParameters);
        
    pmotor->config.deadTimeCompensationVoltageDelay = MCAF_DEAD_TIME_COMPENSATION_VOLTAGE_DELAY;
    // delay for matching current and voltage timeskew
    
    MCAF_DeadTimeCompensationInit(&pmotor->deadTimeCompensation);
    MCAF_DynamicCurrentLimitInit(&pmotor->dynLimit);
    
    int i;
    for (i = 0; i < 3; ++i)
    {
        pmotor->dalphabetaOut[i].alpha = 0;
        pmotor->dalphabetaOut[i].beta  = 0;
    }

    initControlLoopState(pmotor);

    pmotor->controlFlags = 0;    
    pmotor->standardInputs.stateFlags = 0;
    
    /* initialize the mux'd channel (doesn't matter which setting is first) */
    pmotor->adcSelect = HADC_POTENTIOMETER;
    
    initStateParameters(pmotor);
    
    MCAF_SatInit(&pmotor->sat);
    return true;
}

/* ------------------------- Runtime ------------------------- */

/*
 * 1. Feedback path
 */

inline static void estimateBackEMF(MCAF_MOTOR_DATA *pmotor)
{
    MC_ALPHABETA_T  fluxAlphaBeta;     /* Flux in alpha-beta */
    MCAF_STANDARD_INPUT_SIGNALS_T  *pinput     = &pmotor->standardInputs;
    MCAF_BACKEMF_CALCULATION_T     *pemf       = &pmotor->backEMF;
    const MCAF_U_CURRENT_ALPHABETA *ialphabeta = &pinput->ialphabeta;
    const MCAF_U_VOLTAGE_ALPHABETA *valphabeta = &pinput->valphabeta;

    const MCAF_U_STATOR_RESISTANCE rs = pmotor->motorParameters.rs;
    const MCAF_U_STATOR_RESISTANCE l0 = pmotor->motorParameters.l0BaseDt;
    const MCAF_U_STATOR_RESISTANCE l1 = pmotor->motorParameters.l1BaseDt;
    
    if (MCAF_IsMotorSaliencySignificant())
    {
        const int16_t twoTheta = pmotor->thetaElectrical << 1;
        MC_CalculateSineCosine_InlineC_Ram(twoTheta, &pemf->sincos2Theta);
        pemf->l1Cos2Theta = UTIL_MulQ15(l1, pemf->sincos2Theta.cos);
        pemf->l1Sin2Theta = UTIL_MulQ15(l1, pemf->sincos2Theta.sin);

        pemf->laa = l0 + pemf->l1Cos2Theta;
        pemf->lab = pemf->l1Sin2Theta;
        pemf->lba = pemf->l1Sin2Theta;
        pemf->lbb = l0 - pemf->l1Cos2Theta;
    }

    /*
     * Stator voltage equations:
     * Valpha = Rs * Ialpha + d/dt(Ls * Ialpha) + BEMFalpha
     * Vbeta = Rs * Ibeta + d/dt(Ls * Ibeta) + BEMFbeta
     *
     * For motors with saliency, this is a matrix equation:
     *
     *   Vs = Rs*Is + d/dt(Ls*Is) + BEMFs
     * 
     * where Vs, Is, and BEMFs are vectors with alpha and beta components;
     * Ls is a 2x2 matrix.
     */
    /* The stator voltages are scaled down with a factor of two to avoid 
     * overflow/saturation of BEMF calculations in a few corner cases.
     * Accordingly, the shift count for flux and voltage calculations is
     * increased by 1 to match the number system */

    pemf->lastValphabeta.alpha = valphabeta->alpha >> 1;
    pemf->lastValphabeta.beta = valphabeta->beta >> 1;

    /*
     * Compute flux linkage L*I in stationary (alpha-beta) frame
     * due to inductance and the current vector.
     *
     * For motors with significant saliency, L is a 2x2 matrix.
     * 
     * Otherwise just assume isotropic inductance
     * and multiply the current vector by the scalar L0.
     * (This saves CPU cycles.)
     *
     * The check for significant saliency is performed at compile-time
     * and the compiler can optimize out the branch not taken, 
     * to reduce CPU and program memory usage.
     */
    if (MCAF_IsMotorSaliencySignificant())
    {
        fluxAlphaBeta.alpha = UTIL_SatShrS16((__builtin_mulss(pemf->laa, ialphabeta->alpha) +
                        __builtin_mulss(pemf->lab, ialphabeta->beta)), (MCAF_MOTOR_LMAX_BASE_DT_Q + 1));

        fluxAlphaBeta.beta = UTIL_SatShrS16((__builtin_mulss(pemf->lba, ialphabeta->alpha) +
                        __builtin_mulss(pemf->lbb, ialphabeta->beta)), (MCAF_MOTOR_LMAX_BASE_DT_Q + 1));
    }
    else
    {
        fluxAlphaBeta.alpha =
        UTIL_SatShrS16(__builtin_mulss(l0, ialphabeta->alpha), (MCAF_MOTOR_LMAX_BASE_DT_Q + 1));

        fluxAlphaBeta.beta =
        UTIL_SatShrS16(__builtin_mulss(l0, ialphabeta->beta), (MCAF_MOTOR_LMAX_BASE_DT_Q + 1));
    }
    
    /* 
     * Calculate voltage drop in stationary (alpha-beta) frame
     * due to inductance and change in current.
     *
     *   vInductance = d/dt(Ls*Is)
     *
     * The discrete-time calculation of this voltage is
     *
     *   vInductance = (fluxAlphaBeta[k] - fluxAlphaBeta[k-1]) / delta_t
     *
     * where fluxAlphaBeta = Ls*Is.
     *
     * When evaluating this expression in fixed-point,
     * we use an implicit factor of delta_t = 1 (per-unit)
     * to eliminate the need for an additional divide by delta_t.
     * This adds constraints on the fixed-point representations 
     * current, flux, voltage, and inductance.
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
     * and the motor has Ls = 450uH with no saliency.
     * Furthermore, suppose Ialpha changes from 0.44A to 0.99A in one timestep.
     *   
     * Normalized (or "per-unit") calculations are:
     *
     * Ls              = 1.5
     *                 = 450uH (= 1.5 * 300uH)
     * Ialpha[k-1]     = 0.05
     *                 = 0.44A (= 0.05 * 8.8A)
     * Ialpha[k]       = 0.1125
     *                 = 0.99A (= 0.1125 * 8.8A)
     * flux[k-1]       = 0.075
     *                 = 0.198mVs (= 0.075*2.64mVs = 450uH*0.44A)
     * flux[k]         = 0.16875
     *                 = 0.4455mVs (= 0.16875*2.64mVs = 450uH*0.99A)
     * delta_flux      = 0.09375  (= 0.16875-0.075)
     *                 = 247.5uVs (= 0.09375 * 2.64mVs)
     * vInductance     = 0.09375
     *                 = 4.95V (= 0.09375 * 52.8V = 247.5uVs / 50us)
     *
     *                 = (flux[k] - flux[k-1]) / delta_t
     *                 = (flux[k] - flux[k-1])
     *
     * (flux and vInductance in this contrived example
     *  are the alpha components)
     *
     * Note that calculations that vary by a factor of delta_t
     * have identical per-unit values, even though
     * their actual units are different.
     * (For example vInductance = 0.09375 = delta_flux)
     */ 
    pemf->vInductance.alpha = fluxAlphaBeta.alpha - 
                                        pemf->lastFluxalphabeta.alpha;

    pemf->vInductance.beta = fluxAlphaBeta.beta - 
                                        pemf->lastFluxalphabeta.beta;

    pemf->irDrop.alpha = UTIL_SatShrS16(__builtin_mulss(rs, ialphabeta->alpha),
                                                                       (MCAF_MOTOR_RS_Q + 1));

    pemf->irDrop.beta = UTIL_SatShrS16(__builtin_mulss(rs, ialphabeta->beta),
                                                                       (MCAF_MOTOR_RS_Q + 1));

    pinput->ealphabeta.alpha = pemf->lastValphabeta.alpha -
                               pemf->vInductance.alpha -
                               pemf->irDrop.alpha;

    pinput->ealphabeta.beta = pemf->lastValphabeta.beta -
                              pemf->vInductance.beta -
                              pemf->irDrop.beta;

    /* update  LastIalpha and LastIbeta */

    pemf->lastFluxalphabeta.alpha = fluxAlphaBeta.alpha;
    pemf->lastFluxalphabeta.beta  = fluxAlphaBeta.beta;    
}

/*
 * ----- Calculation of standard inputs -----
 * 
 * To make a more uniform interface among motor control algorithms,
 * we calculate some standard input signals and make them available
 * to most algorithms in MCAF.
 * 
 * Because there are feedback loops, there are cyclical dependencies
 * between certain quantities, and in some special cases it is important
 * to minimize time delays and avoid using stale data.
 * 
 * Standard inputs are therefore calculated in several stages.
 * (See doc comments in the following functions.)
 */

/**
 * Calculate standard inputs just prior to a commutation update
 * 
 * This updates input signals that are needed by commutation tasks
 * (angle/velocity estimators and startup state machines, for example)
 * so they contain fresh data.
 * 
 * @param pmotor motor control state variables
 */
inline static void calculateStandardInputsPreCommutation(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_STANDARD_INPUT_SIGNALS_T *pstdinputs = &pmotor->standardInputs;
    
    pstdinputs->valphabeta = pmotor->valphabetaOut;
    pstdinputs->ialphabeta = pmotor->ialphabeta;
    pstdinputs->idq = pmotor->idq;
    pstdinputs->omegaElectricalCommand = pmotor->omegaCmd;    
    pstdinputs->vDC = pmotor->psys->vDC;
    if (MCAF_BackEmfAlphaBetaCalculationNeeded())
    {
        estimateBackEMF(pmotor);
    }
}

/**
 * Calculate standard inputs just after a commutation update
 * 
 * This updates input signals that are produced by commutation tasks
 * (angle/velocity estimators and startup state machines, for example)
 * so they contain fresh data.
 * 
 * @param pmotor motor control state variables
 */
inline static void calculateStandardInputsPostCommutation(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_STANDARD_INPUT_SIGNALS_T *pstdinputs = &pmotor->standardInputs;

    pstdinputs->omegaElectricalEstimated = pmotor->omegaElectrical;
    pstdinputs->startupStatus = MCAF_StartupGetStatus(&pmotor->startup);
    pstdinputs->thetaForcedCommutation = MCAF_StartupGetIdqCmdAngle(&pmotor->startup);
    pstdinputs->iqCmd = pmotor->idqCmdRaw.q;
    const MCAF_U_CURRENT iCmdLimit = MCAF_DynamicCurrentLimitGet(&pmotor->dynLimit);
    pstdinputs->iCmdLimit = iCmdLimit;
    pstdinputs->rICmdLimit = UTIL_DivQ15SatPos(MCAF_RECIPROCAL_CURRENT_NUMERATOR,
                                               iCmdLimit);
    const int16_t motorDirection = UTIL_SignFromHighBit(pmotor->velocityControl.velocityCmd);
    pstdinputs->direction = motorDirection;
}

/**
 * Calculate standard inputs just after all velocity/current control loops
 * have executed.
 * 
 * This updates input signals that are produced by the control loops
 * and may be needed later.
 * 
 * @param pmotor motor control state variables
 */
inline static void calculateStandardInputsPostControlLoops(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_STANDARD_INPUT_SIGNALS_T *pstdinputs = &pmotor->standardInputs;

    pstdinputs->vdq = pmotor->vdq;
}

inline static int16_t calculateZeroSequence(MC_ABC_T *pabc)
{
    asm volatile (";calculateZeroSequence\n");
    const int16_t one_thirdQ16  = 21845U;  // 1/3 Q16
    const int16_t result = 
           MC_UTIL_mulus16(one_thirdQ16, pabc->a)
         + MC_UTIL_mulus16(one_thirdQ16, pabc->b)
         + MC_UTIL_mulus16(one_thirdQ16, pabc->c);
    return result;
}

void MCAF_FocStepIsrFeedbackPath(MCAF_MOTOR_DATA *pmotor)
{
    /* Clarke transform */
    if (MCAF_TripleChannelEnabled() && HAL_ADC_IsAvailableIphaseC())
    {
        MC_TransformClarkeABC(&pmotor->iabc, &pmotor->ialphabeta);
        pmotor->i0 = calculateZeroSequence(&pmotor->iabc);
    }
    else
    {
        MC_TransformClarke(&pmotor->iabc, &pmotor->ialphabeta);
    }

    /* Park transform */
    MC_TransformPark(&pmotor->ialphabeta, &pmotor->sincos, &pmotor->idq);
    
    calculateStandardInputsPreCommutation(pmotor);
    
    /* Calculate commutation angle using estimator */
    MCAF_CommutationStep(pmotor);

    calculateStandardInputsPostCommutation(pmotor);
    
    /* Calculate Sine and Cosine from pmotor->theta_e */
    MC_CalculateSineCosine(pmotor->thetaElectrical, &pmotor->sincos);
}

/*
 * 2. Controllers for current/velocity/etc.
 */

/**
 * Limits the slew rate of current commands.
 * (For now, does nothing.)
 * 
 * @param pidqCmdRaw raw dq-axis current command
 * @param pidqCmd slew-rate limited dq-axis current command
 */
inline void MCAF_RateLimitCurrentCommand(const MC_DQ_T *pidqCmdRaw, MC_DQ_T *pidqCmd)
{
    /* Placeholder method. For now, just copy. */
    pidqCmd->d = pidqCmdRaw->d;
    pidqCmd->q = pidqCmdRaw->q;
}

/**
 * Compute alpha-axis voltage perturbation
 * 
 * @param pmotor motor state data
 * @return voltage including perturbation
 */
inline static MCAF_U_VOLTAGE MCAF_ComputeValphaPerturbation(const MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->valphabeta.alpha 
            + MCAF_CommutationExcitationValpha(pmotor);
}

/**
 * Compute beta-axis voltage perturbation
 * 
 * @param pmotor motor state data
 * @return voltage including perturbation
 */
inline static MCAF_U_VOLTAGE MCAF_ComputeVbetaPerturbation(const MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->valphabeta.beta
            + MCAF_CommutationExcitationVbeta(pmotor);
}

/**
 * Compute q-axis voltage perturbation
 * 
 * @param pmotor motor state data
 * @return voltage including perturbation
 */
inline static MCAF_U_VOLTAGE MCAF_ComputeVqPerturbation(const MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->vdqCmd.q 
            + MCAF_TestPerturbationVq(&pmotor->testing)
            + MCAF_CommutationExcitationVq(pmotor);
}

/**
 * Compute d-axis voltage perturbation
 * 
 * @param pmotor motor state data
 * @return voltage including perturbation
 */
inline static MCAF_U_VOLTAGE MCAF_ComputeVdPerturbation(const MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->vdqCmd.d
            + MCAF_TestPerturbationVd(&pmotor->testing)
            + MCAF_CommutationExcitationVd(pmotor);
}


/**
 * Executes one PI iteration for the current and velocity loops
 * 
 * @param pmotor motor state data
 */
void MCAF_VelocityAndCurrentControllerStep(MCAF_MOTOR_DATA *pmotor)
{
    /*
     * Check to see if new velocity information is available by comparing
     * the number of interrupts per velocity calculation against the
     * number of velocity count samples taken.  If new velocity info
     * is available, calculate the new velocity value and execute
     * the speed control loop. "Velocity" in this context may either be 
     * the estimated velocity OR a substitute for it such as voltage.
     */

    if (pmotor->subsampleCounter == 0)
    {
        /*
         * Execute the velocity control loop
         */
        pmotor->omegaElectrical = pmotor->estimator.omega;
        
        const bool executeVelocityControlLoop =
            MCAF_OperatingModeNormal(&pmotor->testing) && MCAF_IsClosedLoopVelocity(pmotor);

        if (executeVelocityControlLoop)
        {
            const int16_t velocityCmdPerturbed = 
                  pmotor->velocityControl.velocityCmd
                + MCAF_TestPerturbationVelocity(&pmotor->testing);

            pmotor->velocityControl.velocityCmdRateLimited =
                UTIL_LimitSlewRateSymmetrical(
                    velocityCmdPerturbed,                /* input */
                    pmotor->omegaElectrical,             /* reference used as previousOutput */
                    pmotor->velocityControl.slewRateLimit1); /* slew rate limit */

            int16_t limitPos, limitNeg;
            if (pmotor->omegaCmd > 0)
            {
                limitPos = pmotor->velocityControl.slewRateLimitAccel;
                limitNeg = pmotor->velocityControl.slewRateLimitDecel;
            }
            else
            {
                limitPos = pmotor->velocityControl.slewRateLimitDecel;
                limitNeg = pmotor->velocityControl.slewRateLimitAccel;
            }
            pmotor->omegaCmd =
                UTIL_LimitSlewRate(
                    pmotor->velocityControl.velocityCmdRateLimited, /* input */
                    pmotor->omegaCmd,                             /* previousOutput */
                    limitPos,   /* positive limit */
                    limitNeg);  /* negative limit */
      
            /* Antiwindup feedback for the velocity controller should include
             * - both current and voltage saturation, if flux-weakening is not enabled,
             * - only current saturation,             if flux-weakening is enabled.
             *
             * The reason for disallowing voltage saturation as antiwindup feedback
             * when FW is enabled, is that the entire FW region is roughly equal in
             * output voltage requirement. This makes the upper speed end of FW 
             * effectively indistinguishable from the lower speed end of FW
             * from looking at the voltage required from the current loop.
             */
            const MCAF_SAT_STATE_T satState = pmotor->sat.state;
            const MCAF_SAT_STATE_T satStateMasked =
                MCAF_FluxWeakEnabled()
                ? MCAF_SatMask(satState, MCAF_SAT_CURRENT)
                : satState;
                
            const int16_t velocityReference =
                (MCAF_OuterLoopType() == MCAF_OLT_VOLTAGE)
                ? UTIL_MulQ15(pmotor->omegaCmd, pmotor->velocityControl.velocityCmdGain)
                : pmotor->omegaCmd;
            const int16_t velocityFeedback =
                (MCAF_OuterLoopType() == MCAF_OLT_VOLTAGE)
              ? MCAF_FilterLowPassS16Output(&pmotor->vqFiltered)
              : pmotor->omegaElectrical;
            // only the sign of omegaCmd matters
            const int16_t direction = pmotor->omegaCmd;
            MCAF_ControllerPIUpdate(
                velocityReference,
                velocityFeedback,
                &pmotor->omegaCtrl,
                satStateMasked,
                &pmotor->idqCmdRaw.q,
                direction);
        
            if (!MCAF_OverrideFluxControl(&pmotor->testing))
            {
                // Apply d-axis current command and q-axis current limit
                
                pmotor->idqCmdRaw.d = MCAF_FluxControlGetIdCommand(&pmotor->fluxControl);
                const MCAF_U_CURRENT iCmdLimit = MCAF_DynamicCurrentLimitGet(&pmotor->dynLimit);
                const MCAF_U_CURRENT iqLimit = 
                    MCAF_FluxControlGetIqLimit(&pmotor->fluxControl, iCmdLimit);
                pmotor->iqCmdLimit = iqLimit;
                pmotor->omegaCtrl.outMax = iqLimit;
                pmotor->omegaCtrl.outMin = -iqLimit;
            }
        }
    }

    if (MCAF_OperatingModeCurrentLoopActive(&pmotor->testing))
    {
        /* 
         * In certain cases, the electrical angle is adjusted
         * and the PI loop integrator values are inverted.
         * 
         * Some startup methods do not use this, and the following
         * if-statement is optimized out at compile-time.
         */
        if (MCAF_StartupReferenceFrameFlip(&pmotor->startup))
        {
            MCAF_ControllerPIIntegratorInvert(&pmotor->idCtrl);
            MCAF_ControllerPIIntegratorInvert(&pmotor->iqCtrl);
        }
        
        pmotor->idqCmdPerturbed.q = 
                  pmotor->idqCmdRaw.q
                + MCAF_TestPerturbationIq(&pmotor->testing);
        pmotor->idqCmdPerturbed.d = 
                  pmotor->idqCmdRaw.d
                + MCAF_TestPerturbationId(&pmotor->testing);
        
        MCAF_RateLimitCurrentCommand(&pmotor->idqCmdPerturbed, &pmotor->idqCmd);

        const int16_t vlim_d = UTIL_MulQ15(pmotor->idqCtrlOutLimit.d,
                                           pmotor->psys->vDC);
        pmotor->idCtrl.outMax =  vlim_d;
        pmotor->idCtrl.outMin = -vlim_d;
        /* PI control for D-axis */
        MCAF_ControllerPIUpdate(
                pmotor->idqCmd.d, 
                pmotor->idq.d, 
                &pmotor->idCtrl,
                MCAF_SAT_NONE, 
                &pmotor->vdqCmd.d,
                0);
        
        pmotor->vdq.d = MCAF_ComputeVdPerturbation(pmotor);

        if (MCAF_OverrideDAxisVoltagePriority(&pmotor->testing))
        {
            /*
             * Vector limitation
             * Vd is not limited
             * Vq is limited so the vector Vs is less than a specified maximum.
             * Vs = SQRT(Vd^2 + Vq^2) < VDQ_SQUARED_LIMIT
             * Vq = SQRT(VDQ_SQUARED_LIMIT - Vd^2)
             * 
             * here VDQ_SQUARED_LIMIT is calculated as
             * Vdc * MCAF_CURRENT_CTRL_DQ_MAGNITUDE_LIMIT
             */
            const int16_t vdSquared = UTIL_SignedSqr(pmotor->vdq.d);
            const int16_t vdqSquaredLimit = UTIL_SignedSqr(UTIL_MulQ15(pmotor->psys->vDC, MCAF_CURRENT_CTRL_DQ_MAGNITUDE_LIMIT));
            pmotor->iqCtrl.outMax = Q15SQRT(vdqSquaredLimit - vdSquared);
            pmotor->iqCtrl.outMin = -pmotor->iqCtrl.outMax;
        }
        else
        {
            const int16_t vlim_q = UTIL_MulQ15(pmotor->idqCtrlOutLimit.q,
                                               pmotor->psys->vDC);            
            pmotor->iqCtrl.outMax =  vlim_q;
            pmotor->iqCtrl.outMin = -vlim_q;
        }

        /* PI control for Q-axis */
        MCAF_ControllerPIUpdate(
                pmotor->idqCmd.q,
                pmotor->idq.q,
                &pmotor->iqCtrl,
                MCAF_SAT_NONE, 
                &pmotor->vdqCmd.q,
                0);

        pmotor->vdq.q = MCAF_ComputeVqPerturbation(pmotor);
    } /* end of OM_FORCE_CURRENT section */
    else  /* in raw Vd, Vq mode */
    {
        pmotor->vdq.q = MCAF_ComputeVqPerturbation(pmotor);
        pmotor->vdq.d = MCAF_ComputeVdPerturbation(pmotor);
    }

    MCAF_SatDetect(&pmotor->sat, &pmotor->idq, &pmotor->vdq, pmotor->psys->vDC);
    calculateStandardInputsPostControlLoops(pmotor);
}

/*
 * 3. Forward modulation path
 */

inline static void MCAF_ApplyDCLinkVoltageCompensation(const MC_ABC_T *pvabc,
                                                        MC_ABC_T *pdabc,
                                                        int16_t rVdc,
                                                        int16_t rVdc_q)
{
    pdabc->a = (int16_t) (UTIL_mulss(pvabc->a, rVdc) >> rVdc_q);
    pdabc->b = (int16_t) (UTIL_mulss(pvabc->b, rVdc) >> rVdc_q);
    pdabc->c = (int16_t) (UTIL_mulss(pvabc->c, rVdc) >> rVdc_q);
}

inline int16_t MCAF_ComputeReciprocalDCLinkVoltage(int16_t vdc)
{
    if (vdc <= MCAF_RVDC_MIN_VDC)
    {
        return MCAF_RVDC_MIN;
    }
    else
    {
        return __builtin_divf(MCAF_RVDC_MIN_VDC, vdc);
    }
}

inline static void MCAF_ScaleQ15(const MC_ABC_T *pabc_in, MC_ABC_T *pabc_out, int16_t k)
{
    pabc_out->a = UTIL_MulQ15(pabc_in->a, k);
    pabc_out->b = UTIL_MulQ15(pabc_in->b, k);
    pabc_out->c = UTIL_MulQ15(pabc_in->c, k);
}

/**
 * Add dead-time compensation duty cycles to output duty cycles, if appropriate
 *
 * @param pdabcIn input duty cycles
 * @param pdabcOut output duty cycles
 * @param pdtc_abc pointer to dead-time compensation duty cycles,
 *        or NULL if no dead-time compensation
 */
inline static void MCAF_ApplyDeadTimeCompensationForwardPath(
    const MCAF_U_DUTYCYCLE_ABC *pdabcIn,
    MCAF_U_DUTYCYCLE_ABC *pdabcOut,
    const MCAF_U_DUTYCYCLE_ABC *pdtc_abc)
{
    if (pdtc_abc == NULL)
    {
        // No dead-time compensation
        pdabcOut->a = pdabcIn->a;
        pdabcOut->b = pdabcIn->b;
        pdabcOut->c = pdabcIn->c;            
    }
    else
    {
        pdabcOut->a = pdabcIn->a + pdtc_abc->a;
        pdabcOut->b = pdabcIn->b + pdtc_abc->b;
        pdabcOut->c = pdabcIn->c + pdtc_abc->c;
    }
}

inline static void calculateManualZeroSequenceModulation(
    const MC_ABC_T *pabc_in, MC_ABC_T *pabc_out, int16_t delta,
    int16_t min, int16_t max)
{
    pabc_out->a = MC_adjust_zero_sequence(pabc_in->a, delta, min, max);
    pabc_out->b = MC_adjust_zero_sequence(pabc_in->b, delta, min, max);
    pabc_out->c = MC_adjust_zero_sequence(pabc_in->c, delta, min, max);    
}

void MCAF_FocStepIsrForwardPath(MCAF_MOTOR_DATA *pmotor)
{
    /* Calculate control values */
    MCAF_VelocityAndCurrentControllerStep(pmotor);

    /* Calculate vAlpha, Vbeta from Sine, Cosine, Vd and Vq */
    MC_TransformParkInverse(&pmotor->vdq, &pmotor->sincos, &pmotor->valphabeta);
    
    pmotor->valphabetaPerturbed.alpha = MCAF_ComputeValphaPerturbation(pmotor);
    pmotor->valphabetaPerturbed.beta  = MCAF_ComputeVbetaPerturbation(pmotor);

    /* Calculate Va,Vb,Vc from vAlpha and Vbeta */
    MC_TransformClarkeInverse(&pmotor->valphabetaPerturbed, &pmotor->vabc);
    
    MCAF_ApplyDCLinkVoltageCompensation(&pmotor->vabc, &pmotor->dabcRaw, pmotor->rVdc, MCAF_RVDC_Q);
    
    /* Dead-time compensation, forward path */
    const MCAF_U_DUTYCYCLE_ABC *pdtc_abc = 
        MCAF_DeadTimeCompensationForwardPathCompute(&pmotor->deadTimeCompensation,
                                                    &pmotor->iabc);
    MCAF_ApplyDeadTimeCompensationForwardPath(&pmotor->dabcRaw,
                                              &pmotor->dabcUnshifted,
                                              pdtc_abc);
        
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

#if MCAF_SINGLE_CHANNEL_SUPPORT 
    /* Calculate the adjusted duty cycles and the necessary triggers for current measurement */
    MCAF_ComputeSingleChannelVectors(&pmotor->currentMeasure, &pmotor->vabc, 
            &pmotor->dabc, HAL_PARAM_PWM_PERIOD_COUNTS);
    
    MCAF_ScaleQ15(&pmotor->currentMeasure.pwmDutyCycle[0], 
            &pmotor->currentMeasure.pwmDutyCycleOut.rising, HAL_PARAM_PWM_PERIOD_COUNTS);
    MCAF_ScaleQ15(&pmotor->currentMeasure.pwmDutyCycle[1], 
            &pmotor->currentMeasure.pwmDutyCycleOut.falling, HAL_PARAM_PWM_PERIOD_COUNTS); 
#else    
    MCAF_ScaleQ15(&pmotor->dabc, &pmotor->pwmDutycycle, 
        HAL_PARAM_PWM_PERIOD_COUNTS);
#endif
    
     /* Feedback path to estimators */
    {
        MCAF_U_DUTYCYCLE_ALPHABETA dalphabeta_compensated;

        // Update history of voltage provided as output
        pmotor->dalphabetaOut[2].alpha = pmotor->dalphabetaOut[1].alpha;
        pmotor->dalphabetaOut[2].beta  = pmotor->dalphabetaOut[1].beta;
        pmotor->dalphabetaOut[1].alpha = pmotor->dalphabetaOut[0].alpha;
        pmotor->dalphabetaOut[1].beta  = pmotor->dalphabetaOut[0].beta;
       
        const MCAF_U_DUTYCYCLE_ABC *pDabc =
        MCAF_DutyCycleFeedbackIncludesClipping() ? &pmotor->dabc 
                                                 : &pmotor->dabcUnshifted;
        MC_TransformClarkeABC(pDabc, &pmotor->dalphabetaOut[0]);
        
        // We use a delay=2 to match current and duty cycle signals properly
        uint16_t kdelay = pmotor->config.deadTimeCompensationVoltageDelay;
        if (kdelay > 2)
            kdelay = 2;

        const MCAF_U_DUTYCYCLE_ALPHABETA *pdtc_alphabeta = MCAF_DeadTimeCompensationFeedbackPathCompute(&pmotor->deadTimeCompensation);
        
        if (pdtc_alphabeta == NULL)
        {
            dalphabeta_compensated.alpha = pmotor->dalphabetaOut[kdelay].alpha;
            dalphabeta_compensated.beta  = pmotor->dalphabetaOut[kdelay].beta;            
        }
        else
        {
            dalphabeta_compensated.alpha = pmotor->dalphabetaOut[kdelay].alpha - pdtc_alphabeta->alpha;
            dalphabeta_compensated.beta  = pmotor->dalphabetaOut[kdelay].beta  - pdtc_alphabeta->beta;
        }
        
        const MCAF_U_VOLTAGE vDC = pmotor->psys->vDC;
        pmotor->valphabetaOut.alpha = UTIL_MulQ15(dalphabeta_compensated.alpha, vDC);
        pmotor->valphabetaOut.beta  = UTIL_MulQ15(dalphabeta_compensated.beta, vDC);
    }
}

/*
 * 4. Non-critical tasks
 */

void MCAF_FocStepIsrNonCriticalTask(MCAF_MOTOR_DATA* pmotor)
{
    MCAF_ADCReadNonCritical(pmotor);
    if (!MCAF_OverrideDCLinkCompensation(&pmotor->testing))
    {
        pmotor->rVdc = MCAF_ComputeReciprocalDCLinkVoltage(pmotor->psys->vDC);
    }
    MCAF_DynamicCurrentLimitUpdate(&pmotor->dynLimit, &pmotor->idq);
    
    MCAF_CaptureTimestamp(&pmotor->testing, MCTIMESTAMP_FLUX_CONTROL_START);
    MCAF_FluxControlStep(&pmotor->fluxControl,
                         &pmotor->standardInputs,
                         &pmotor->motorParameters);
    MCAF_CaptureTimestamp(&pmotor->testing, MCTIMESTAMP_FLUX_CONTROL_END);

    if (++pmotor->subsampleCounter >= MCAF_ISR_SUBSAMPLE_DIVIDER)
    {
        pmotor->subsampleCounter = 0;
    }
    
    if (MCAF_OuterLoopType() == MCAF_OLT_VOLTAGE)
    {
        MCAF_FilterLowPassS16Update(&pmotor->vqFiltered, pmotor->vdqCmd.q);
    }
#if MCAF_TRIGGERED_AVERAGE_EXAMPLE == 1
    MCAF_TriggeredAverage_Step(&pmotor->iqAverage, pmotor->idq.q);
#endif
}

/*
 * 5. Miscellaneous
 */

void MCAF_FocStepMain(MCAF_MOTOR_DATA *pmotor)
{
    /* do nothing for now */
}

