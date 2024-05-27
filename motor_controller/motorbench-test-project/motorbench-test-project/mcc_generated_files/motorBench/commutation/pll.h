/**
 * pll.h
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

#ifndef __PLL_H
#define __PLL_H

#include <stdint.h>
#include "motor_control_types.h"
#include "foc_types.h"
#include "units.h"
#include "commutation/common.h"
#include "parameters/pll_params.h"
#include "startup_types.h"

#ifdef __cplusplus
extern "C" {
#endif
    
/**
 * State variables for PLL estimator
 */
typedef struct tagMCAF_ESTIMATOR_PLL
{
    /*
     * Runtime-adjustable parameters
     *
     * These values are typically set once, at startup,
     * but may be adjusted using real-time diagnostic tools.
     */
    
    int16_t dtAngular;          /** Integration scaling factor dt from electrical frequency to electrical angle */
    int16_t rhoOffset;          /** Offset angle used to compensate the estimated rotor angle */
    int16_t dIlimitLS;          /** Delta current limit used for low velocity range */
    int16_t dIlimitHS;          /** Delta current limit used for high velocity range */
    int16_t kEsdqFilter;        /** Filter constant used for filtering BEMF voltage */
    int16_t kVelEstimFilter;    /** Filter constant used for filtering estimated velocity */
    MCAF_U_VELOCITY_ELEC omegaFilterThreshold;    /** Velocity below which we use slow filtering */
    MCAF_U_BACKEMF_INVERSE keInverse;             /** Inverse of back-EMF constant */

    /*
     * State variables
     *
     * The state of the estimator at any given instant 
     * is completely determined by its state variables.
     *
     * At each new step of the estimator, the state variables can theoretically
     * be expressed as a deterministic function of the following values:
     *
     *   - state variables at the previous step
     *   - runtime-adjustable parameters
     *   - inputs to the estimator step function
     */
    int32_t rhoStateVar;        /** Internal state variable for rotor angle */
    int16_t rho;                /** Estimated rotor angle prior to offset compensation */
    int16_t diCounter;          /** Counter index used to track current sample history */
    MCAF_U_VOLTAGE_DQ_Q14 esdqFiltered;       /** Filtered value of BEMF in d-q reference frame */
    struct
    {
        int32_t d;
        int32_t q;
    } esdqStateVar;             /** State variable for BEMF in d-q reference frame */
    int32_t velEstimStateVar;   /** State Variable for estimated velocity */
    MCAF_U_VOLTAGE_ALPHABETA lastValphabeta;     /** Value of Valphabeta from previous control step */
    MCAF_U_CURRENT_ALPHABETA lastIalphabeta[PLL_LOWSPEED_DIBYDT_PRESCALER];  /** Array storing a history of
                                                                     * previous Ialphabeta values */
    
    /*
     * Auxiliary variables
     *
     * These values can be derived from the state variables
     * using memory-less calculations. 
     *
     * They are usually output or intermediate variables,
     * and are typically retained in RAM to support data logging 
     * with real-time diagnostic tools.
     */
    int16_t omegaMr;            /** Estimated velocity that is unscaled and unfiltered */
    MCAF_U_VOLTAGE_ALPHABETA_Q14 esalphabeta; /** Calculated BEMF in alpha-beta reference frame */
    MCAF_U_VOLTAGE_DQ_Q14 esdq;               /** Calculated BEMF in d-q reference frame */
    MCAF_U_DIMENSIONLESS_SINCOS sincos;   /** Sine and cosine component of calculated rotor angle */
    MCAF_U_VOLTAGE_ALPHABETA vInductance; /** Calculated value of voltage across stator inductance */
    MCAF_U_VOLTAGE irDropAlpha;           /** Calculated value of voltage across stator resistance */
    MCAF_U_VOLTAGE irDropBeta;            /** Calculated value of voltage across stator resistance */
    
    MCAF_ESTIMATOR_OUTPUTS_T output;      /** Output velocity and angle */
} MCAF_ESTIMATOR_PLL_T;

/**
 * Initializes PLL state variables on reset.
 * Summary: Initializes PLL state variables.
 * @param pll PLL state variable structure
 * @param pmotor Motor parameters
 */
void MCAF_EstimatorPllInit(MCAF_ESTIMATOR_PLL_T *pll, const MCAF_MOTOR_PARAMETERS_T *pmotor);

/**
 * Initializes PLL state variables prior to starting motor.
 * 
 * @param pll PLL state variable structure
 */
void MCAF_EstimatorPllStartupInit(MCAF_ESTIMATOR_PLL_T *pll);

/**
 * Executes one control step of the PLL estimator.
 * Summary: Executes one control step of the PLL estimator.
 * @param pll PLL state variable structure
 * @param pinput Standard algorithm inputs (e.g. stationary-frame voltage and current)
 * @param pmotor Motor parameters
 */
void MCAF_EstimatorPllStep(MCAF_ESTIMATOR_PLL_T *pll,
                           const MCAF_STANDARD_INPUT_SIGNALS_T *pinput,
                           const MCAF_MOTOR_PARAMETERS_T *pmotor);

/**
 * Returns commutation angle
 * 
 * @param pll state
 * @return commutation angle
 */
inline static MCAF_U_ANGLE_ELEC MCAF_EstimatorPllCommutationAngle(const MCAF_ESTIMATOR_PLL_T *pll)
{
    return pll->output.thetaElectrical;
}

/**
 * Sets commutation angle
 * 
 * This should be used sparingly and only to initialize the PLL,
 * when there is very high confidence in an angle obtained from another source,
 * and the PLL may be operating at a low speed and get stuck.
 * 
 * @param pll state
 * @param theta desired angle
 */
inline static void MCAF_EstimatorPllSetAngle(MCAF_ESTIMATOR_PLL_T *pll, MCAF_U_ANGLE_ELEC theta)
{
    MCAF_U_ANGLE_ELEC rho = theta - pll->rhoOffset;
    pll->rhoStateVar = (int32_t)rho << 15;
}

/**
 * Updates commutation angle and velocity
 * 
 * This should be used sparingly and only to initialize the PLL,
 * when there is very high confidence in an angle obtained from another source,
 * and the PLL may be operating at a low speed and get stuck.
 * 
 * This type signature (with typeless void *) is designed to be compatible with 
 * MCAF_ESTIMATOR_OUTPUT_UPDATE_FUNCTION, for interaction with other code
 * modules that should not have a code dependency on this estimator.
 *
 * @param obj PLL state
 * @param pout desired angle and velocity
 */
inline static void MCAF_EstimatorPllUpdateOutput(void *obj, const MCAF_ESTIMATOR_OUTPUTS_T *pout)
{
    MCAF_ESTIMATOR_PLL_T *pll = (MCAF_ESTIMATOR_PLL_T *)obj;
    MCAF_EstimatorPllSetAngle(pll, pout->thetaElectrical);
    /* MCAF R7 will not update the velocity at this time. */
}

/**
 * Returns electrical frequency
 * 
 * @param pll state
 * @return electrical frequency
 */
inline static MCAF_U_VELOCITY_ELEC MCAF_EstimatorPllElectricalFrequency(const MCAF_ESTIMATOR_PLL_T *pll)
{
    return pll->output.omegaElectrical;
}

/**
 * Determine whether startup delay is requested
 * 
 * @param pll state
 * @param startupStatus startup status
 * @return whether a startup delay is requested
 */
inline static bool MCAF_EstimatorPllStartupDelayRequested(const MCAF_ESTIMATOR_PLL_T *pll, MCAF_STARTUP_STATUS_T startupStatus)
{
    return false;
}



#ifdef __cplusplus
}
#endif

#endif /* __PLL_H */