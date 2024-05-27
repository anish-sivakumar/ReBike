/**
 * startup.c
 *
 * Module for startup and open loop to closed loop transition
 * 
 * Component: state machine
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
#include <stdbool.h>
#include "startup.h"
#include "parameters/timing_params.h"
#include "parameters/startup_params.h"
#include "ui.h"
#include "error_codes.h"
#include "util.h"

void MCAF_StartupTransitioningInit(MCAF_MOTOR_STARTUP_DATA *pstartup)
{
    pstartup->velocityThreshold[0] = STARTUP_ACCEL0_VELOCITY_THRESHOLD;
    pstartup->velocityThreshold[1] = STARTUP_ACCEL1_VELOCITY_THRESHOLD;
    pstartup->acceleration[0] = STARTUP_ACCELERATION0;
    pstartup->acceleration[1] = STARTUP_ACCELERATION1;
    pstartup->alignTime = STARTUP_ALIGN_TIME;
    pstartup->holdTime = STARTUP_HOLD_TIME;
    pstartup->rampupAngle = STARTUP_RAMPUP_ANGLE;
    pstartup->alignAngleDelta = STARTUP_ALIGN_ANGLE_DELTA;
    pstartup->thetaDelta = Q15_THETADELTA;
    pstartup->iAmplitude = MCAF_STARTUP_CURRENT;
    pstartup->dt = STARTUP_DELTA_T_FACTOR;
    pstartup->dtAcceleration = STARTUP_DELTA_T_FACTOR_ACCELERATION;
    
    pstartup->activeDamping.iqmax = STARTUP_DAMPING_IQMAX;
    pstartup->activeDamping.k = STARTUP_DAMPING_GAIN;
    pstartup->activeDamping.velocityThreshold = STARTUP_DAMPING_THRESHOLD;
}

inline static int16_t limit32(int32_t x, int16_t limitLo, int16_t limitHi)
{
    if (x > limitHi)
    {
        return limitHi;
    }
    else if (x < limitLo)
    {
        return limitLo;
    }
    else
    {
        return x;
    }
}

/**
 * add damping to nominal current
 *
 * @param pstartup startup data
 * @param nominalCurrent nominal current without damping
 * @return modified current
 */
inline static int16_t MCAF_StartupAddDamping(const MCAF_MOTOR_STARTUP_DATA *pstartup,
        int16_t nominalCurrent)
{
    const int16_t omega_error = pstartup->omegaElectrical.x16.hi
                                - pstartup->omegaElectricalEstimated;
    const int32_t iqdamping_q32 = UTIL_mulss(omega_error, pstartup->activeDamping.k)
                                  >> STARTUP_DAMPING_SHIFT;
    const int16_t iqdamping = limit32(iqdamping_q32,
                                        -pstartup->activeDamping.iqmax,
                                        pstartup->activeDamping.iqmax);
    return nominalCurrent + iqdamping;
}

enum MCAF_STARTUP_DAMPING { MSD_ENABLED = 1, MSD_CONDITIONAL = 0 };

/**
 *
 * @param pstartup startup data
 * @param nominalCurrent nominal current
 * @param enableDampingAlways whether to enable damping always
 * @return current command
 */
inline static int16_t MCAF_StartupCalcIq(const MCAF_MOTOR_STARTUP_DATA *pstartup,
        int16_t nominalCurrent,
        enum MCAF_STARTUP_DAMPING dampingPreference)
{
    int16_t result = nominalCurrent;
    if ((dampingPreference == MSD_ENABLED) ||
            UTIL_Abs16Approx(pstartup->omegaElectrical.x16.hi)
              > pstartup->activeDamping.velocityThreshold)
    {
        result = MCAF_StartupAddDamping(pstartup, nominalCurrent);
    }
    return result;
}

/**
 * Accelerate velocity, check whether velocity threshold is reached
 *
 * @param pstartup startup data
 * @param directedAcceleration acceleration (+ or -)
 * @param velocityThreshold threshold for completion
 * @return true if threshold is reached
 */
inline static bool MCAF_StartupAccelVelocityIsComplete(MCAF_MOTOR_STARTUP_DATA *pstartup,
        int16_t directedAcceleration,
        int16_t velocityThreshold)
{
    const bool thresholdReached =
        UTIL_Abs16(pstartup->omegaElectrical.x16.hi) >= velocityThreshold;
    if (!thresholdReached)
    {
        pstartup->omegaElectrical.x32 +=
                UTIL_mulss(pstartup->dtAcceleration, directedAcceleration);
    }
    return thresholdReached;
}

#define MCAF_STARTUP_RESET_CYCLES 1

void MCAF_StartupTransitioningStep(MCAF_MOTOR_STARTUP_DATA *pstartup,
        MC_DQ_T *idqCommand, int16_t direction)

{
    const int16_t iqtorquecmd = idqCommand->q;
    int16_t iqtorquecmd_next = iqtorquecmd;
    
    switch (pstartup->state)
    {
        case SSM_START:
            /* startup in reset: stay in this state until re-enabled */
            if (pstartup->enable)
            {
                if (pstartup->counter < MCAF_STARTUP_RESET_CYCLES)
                {
                    ++pstartup->counter;
                }
                else
                {
                    pstartup->state = SSM_CURRENT_RAMPUP;
                }
            }
            pstartup->iNominal = pstartup->iAmplitude * direction;
            break;
        case SSM_CURRENT_RAMPUP:
        {
            iqtorquecmd_next = UTIL_LimitSlewRateSymmetrical(
                    pstartup->iNominal,                /* input */
                    iqtorquecmd,                        /* previousOutput */
                    pstartup->iRampupLimit);           /* limit */

            if (iqtorquecmd == iqtorquecmd_next)
            {
                pstartup->counter = 0;
                pstartup->state = SSM_ALIGN;
                pstartup->thetaElectrical.x16.hi += pstartup->alignAngleDelta;
            }
            break;
        }
        case SSM_ALIGN:
        {
            if (pstartup->counter < pstartup->alignTime)
            {
                ++pstartup->counter;
            }
            else if (!pstartup->delayRequest)
            {
                pstartup->state = SSM_ACCEL0;
            }
            break;
        }
        case SSM_ACCEL0:
        {
            int16_t directed_accel = pstartup->acceleration[0] * direction;
            if (MCAF_StartupAccelVelocityIsComplete(pstartup, directed_accel,
                    pstartup->velocityThreshold[0]))
            {
                pstartup->state = SSM_ACCEL1;
            }
            iqtorquecmd_next = MCAF_StartupCalcIq(pstartup, pstartup->iNominal, MSD_CONDITIONAL);
            break;
        }
        case SSM_ACCEL1:
        {
            int16_t directed_accel = pstartup->acceleration[1] * direction;
            if (MCAF_StartupAccelVelocityIsComplete(pstartup, directed_accel,
                    pstartup->velocityThreshold[1]))
            {
                pstartup->counter = 0;
                pstartup->state = SSM_HOLD;
            }
            iqtorquecmd_next = MCAF_StartupCalcIq(pstartup, pstartup->iNominal, MSD_CONDITIONAL);
            break;
        }
        case SSM_HOLD:
        {
            iqtorquecmd_next = MCAF_StartupCalcIq(pstartup, pstartup->iNominal, MSD_ENABLED);
            if (pstartup->counter < pstartup->holdTime)
            {
                ++pstartup->counter;
            }
            else
            {                
                if (!pstartup->delayRequest)
                {
                    pstartup->counter = 0;
                    pstartup->state = SSM_CURRENT_RAMPDOWN;
                    pstartup->torqueCmd32 = (int32_t)iqtorquecmd_next << 16;
                }
            }
            break;
        }
        case SSM_CURRENT_RAMPDOWN:
        {
            /* Decrease magnitude of Iq current torque command
             * to reduce startup theta error */
            pstartup->torqueCmd32 -=
                    UTIL_mulss(pstartup->torqueCmd32 >> 16,
                                MCAF_RAMPDOWN_DECAY_RATE) >> MCAF_RAMPDOWN_DECAY_SHIFT;
            const int16_t nominalCurrent = pstartup->torqueCmd32 >> 16;
            iqtorquecmd_next = MCAF_StartupCalcIq(pstartup, nominalCurrent, MSD_ENABLED);
            if (UTIL_Abs16Approx(nominalCurrent) <= MCAF_RAMPDOWN_END_CURRENT)
            {
                pstartup->complete = true;
                pstartup->state = SSM_TRANSITION;
            }
            if (UTIL_Abs16Approx(MCAF_StartupGetThetaError(pstartup)) < pstartup->thetaDelta)
            {
                pstartup->complete = true;
                pstartup->state = SSM_TRANSITION;
            }
            break;
        }
        case SSM_TRANSITION:
        {
            sx1632_t errorConvergeStep;
            errorConvergeStep.x32 = ((int32_t)STARTUP_THETA_ERROR_CONVERGE_RATE) << (32-STARTUP_THETA_ERROR_CONVERGE_RATE_Q);
            const MCAF_U_ANGLE_ELEC thetaError = MCAF_StartupGetThetaError(pstartup);
            if (UTIL_Abs16(thetaError) > errorConvergeStep.x16.hi)
            {
                if (thetaError < 0)
                {
                    pstartup->thetaError.x32 += errorConvergeStep.x32;
                }
                else
                {
                    pstartup->thetaError.x32 -= errorConvergeStep.x32;
                }
            }
            else
            {
                pstartup->state = SSM_COMPLETE;
                MCAF_StartupSetThetaError(pstartup, 0);
            }
            break;
        }
        case SSM_COMPLETE:
            /* startup complete: do nothing */
            break;
        case SSM_INACTIVE:
            /* do nothing in test modes */
            break;
        default:
        {
            MCAF_UiFlashErrorCodeForever(ERR_INVALID_STARTUP_FSM_STATE);
        }
    }
    idqCommand->q = iqtorquecmd_next;
    pstartup->delayRequest = false;
}
