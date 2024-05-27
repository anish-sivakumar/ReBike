/* 
 * options.h
 * 
 * Feature flags for options that can be enabled
 *
 * Component: miscellaneous
 */ /*
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
 *
 ******************************************************************************/

#ifndef __OPTIONS_H
#define __OPTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#define MCAF_INCLUDE_STALL_DETECT 1

inline static bool MCAF_OvercurrentDetectEnabled(void) { return true; }
inline static bool MCAF_NegativeEdDetectEnabled(void)  { return true; }
inline static bool MCAF_LowSpeedDetectEnabled(void)    { return true; }
inline static bool MCAF_TorqueAngleDetectEnabled(void) { return true; }
inline static bool MCAF_VarianceDetectEnabled(void)    { return true; }

/** Does E = V - IR - d(LI)/dt need to be calculated
 *  in the stationary frame?
 */
inline static bool MCAF_BackEmfAlphaBetaCalculationNeeded(void)    { return false; }

inline static bool MCAF_MTPAEnabled(void)    { return false; }

inline static bool MCAF_FluxWeakEnabled(void)    { return false; }

#define MCAF_SINGLE_CHANNEL_SUPPORT 0

inline static bool MCAF_SingleChannelEnabled(void)    { return false; }
inline static bool MCAF_TripleChannelEnabled(void)    { return false; }

typedef enum  {
    MCAF_CURRENT_LIMIT_RECTANGULAR = 0,
    MCAF_CURRENT_LIMIT_QUADRATIC = 1
} MCAF_CURRENT_LIMIT_REGION_TYPE;

/** What type of current limit region is used? */
inline static MCAF_CURRENT_LIMIT_REGION_TYPE MCAF_CurrentLimitRegionType()
{
    return MCAF_CURRENT_LIMIT_QUADRATIC;
}

/** Are internal opamps to be enabled? */
inline static bool MCAF_OpAmpsEnabled(void) { return true; }

/** Does the Clarke transform for taking duty cycle feedback take clipping into account? */
inline static bool MCAF_DutyCycleFeedbackIncludesClipping(void) { return true; }

/** Enables the test harness.
 */
//#define MCAF_TEST_HARNESS

/** Enables cpu profiling.
 */
//#define MCAF_TEST_PROFILING

/** Does the test harness use symmetric (square wave) or asymmetric perturbation? 
 *  0 = asymmetric, 1 = symmetric
 */
#define MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC 1

/** Include triggered average example implementation?
 *  Note: MCAF_TEST_HARNESS must also be defined to enable triggered averaging.
 */
#define MCAF_TRIGGERED_AVERAGE_EXAMPLE 0

/** Enable use of MCAPI_AdcIsrPrologUserFunction()?
 */
inline static bool MCAF_AdcIsrPrologEnabled(void) { return false; }

/** Enable use of MCAPI_AdcIsrEpilogUserFunction()?
 */
inline static bool MCAF_AdcIsrEpilogEnabled(void) { return false; }

/** Does the STOPPING state (transition towards zero-speed) use closed-loop current control?
 *  If so, current will be controlled and PWM outputs kept active.
 *  Otherwise:
 *    PWM outputs will be set as minimal-impact state. 
 *    Output current may not be measurable, but is expected to be zero.
 *    Sensorless position and velocity estimators may not work.
 */
inline static bool MCAF_StoppingClosedLoopCurrent(void) { return false; }

/** Does the STOPPING state (transition towards zero-speed) use closed-loop velocity control?
 *  If so, velocity will be controlled towards zero.
 *  Otherwise, current command will be set to zero.
 *  (NOTE: Requires closed-loop current in stopping state. Unpredictable results
 *  may occur if closed-loop velocity is set but not closed-loop current.)
 */
inline static bool MCAF_StoppingClosedLoopVelocity(void) { return false; }

#ifdef __cplusplus
}
#endif

#endif /* __OPTIONS_H */
