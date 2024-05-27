/**
 * state_machine.c
 * 
 * Top-level motor controller state-machine
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
#include "system_state.h"
#include "foc.h"
#include "adc_compensation.h"
#include "commutation.h"
#include "parameters/hal_params.h"
#include "parameters/operating_params.h"
#include "parameters/timing_params.h"
#include "parameters/options.h"
#include "board_service.h"
#include "hal.h"
#include "recover.h"
#include "stall_detect.h"
#include "util.h"
#include "monitor.h"
#include "error_codes.h"
#include "ui.h"
#include "flux_control.h"
#include "fault_detect.h"
#include "test_harness.h"
#include "current_measure.h"

inline static void MCAF_SetPwmMinimalImpact(void)
{
    HAL_PWM_LowerTransistorsDutyCycle_Set(HAL_PARAM_PWM_PERIOD_COUNTS, 
            HAL_PARAM_MIN_LOWER_DUTY_COUNTS);
}

/**
 * Initializes PWM registers for normal operation and
 * set a default "safe" duty cycle
 */
inline static void EnablePwmMinDuty(void)
{
    HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MIN_DUTY_COUNTS);
    HAL_PWM_UpperTransistorsOverride_Disable();
}

/**
 * Resets stopping timer state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_StoppingTimerReset(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->stopping.timer.count = pmotor->stopping.timer.duration;
    pmotor->stopping.timer.rate  = 1;
}

/**
 * Updates stopping timer state.
 * 
 * @param pmotor motor state data
 * @return true if timer has expired
 */
inline static bool MCAF_StoppingTimerUpdate(MCAF_MOTOR_DATA *pmotor)
{
    int32_t newcount = pmotor->stopping.timer.count - pmotor->stopping.timer.rate;
    const bool stopNow = MCAF_TestHarnessCheckStopNowAndReset(&pmotor->testing);
    const bool expired = (newcount <= 0) || stopNow;
                          
    if (expired)
    {
        newcount = 0;
    }
    pmotor->stopping.timer.count = newcount;
    return expired;
}

/** 
 * Reset state variables for recovery:
 * if there is user intervention and runRequested is no longer true,
 * we allow retry efforts to make a fresh start.
 */
inline void MCAF_ResetRecoveryIfNotRunRequested(MCAF_MOTOR_DATA *pmotor)
{
    const bool runRequested = pmotor->ui.run;
    if (!runRequested)
    {
        MCAF_RecoveryReset(&pmotor->recovery);
    }    
}

/**
 * Executes actions on entry to the STOPPED state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnStoppedInit(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_SetPwmMinimalImpact();
    MCAF_ClearClosedLoopFlags(pmotor);
    MCAF_MonitorRecoveryAcknowledged(pmotor);
    MCAF_RecoverySetInputFlag(&pmotor->recovery, MCAF_RECOVERY_FSMI_STOP_COMPLETED);
#if MCAF_INCLUDE_STALL_DETECT      
    MCAF_StallDetectReset(&pmotor->stallDetect);
    MCAF_StallDetectDeactivate(&pmotor->stallDetect);
#endif
    MCAF_FaultDetectReset(&pmotor->faultDetect);
    /* faults/failures are reset by re-init */
}

/**
 * Executes actions in the STOPPED state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnStopped(MCAF_MOTOR_DATA *pmotor)
{    
    MCAF_ResetRecoveryIfNotRunRequested(pmotor);
}

/**
 * Executes actions on entry to STARTING state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnStartingInit(MCAF_MOTOR_DATA *pmotor)
{       
    MCAF_CommutationStartupInit(pmotor);
    MCAF_CurrentMeasureRestart(&pmotor->currentMeasure);
    MCAF_FocInitializeIntegrators(pmotor);
    MCAF_FluxControlStartupInit(&pmotor->fluxControl);
#if MCAF_INCLUDE_STALL_DETECT      
    MCAF_StallDetectActivate(&pmotor->stallDetect);
#endif
    EnablePwmMinDuty();
    MCAF_SetClosedLoopCurrent(pmotor);
}

/**
 * Limits an array of three duty cycle values to at least a specified 
 * minimum. 
 * 
 * @param output output array
 * @param pinput input duty cycle values
 * @param min minimum output value
 */
inline static void constrainDutyCycleAsArray(uint16_t *output, 
        const MCAF_U_DUTYCYCLE_ABC *pinput, uint16_t min)
{
    output[0] = UTIL_LimitMinimumU16(pinput->a, min);
    output[1] = UTIL_LimitMinimumU16(pinput->b, min);
    output[2] = UTIL_LimitMinimumU16(pinput->c, min);
}

/**
 * Executes actions common to the "active states":
 * - STARTING
 * - RUN
 * - TEST_ENABLE
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnActiveStates(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_FocStepIsrForwardPath(pmotor);
    
#if MCAF_SINGLE_CHANNEL_SUPPORT 
    {
        const MCAF_CURRENT_MEASUREMENT *pcurr = &pmotor->currentMeasure;
        uint16_t pwmPhase[3];
        uint16_t pwmDutyCycle[3];
        constrainDutyCycleAsArray(pwmPhase, &pcurr->pwmDutyCycleOut.rising, HAL_PARAM_MIN_DUTY_COUNTS);
        constrainDutyCycleAsArray(pwmDutyCycle, &pcurr->pwmDutyCycleOut.falling, HAL_PARAM_MIN_DUTY_COUNTS);
        HAL_PWM_DutyCycleDualEdge_Set(pwmPhase, pwmDutyCycle);
    }
#else
    {
        uint16_t pwmDutyCycle[3];
        constrainDutyCycleAsArray(pwmDutyCycle, &pmotor->pwmDutycycle, HAL_PARAM_MIN_DUTY_COUNTS);
        HAL_PWM_DutyCycleRegister_Set(pwmDutyCycle);
    }
#endif    
}

/**
 * Executes actions on the STARTING state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnStarting(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_MotorControllerOnActiveStates(pmotor);
}

/**
 * Executes actions on entry to the RUNNING state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnRunningInit(MCAF_MOTOR_DATA *pmotor)
{
    if (!MCAF_IsClosedLoopVelocity(pmotor))
    {
        MCAF_CommutationTransitionToClosedLoop(pmotor);
    }
}

/**
 * Executes actions in the RUNNING state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnRunning(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_MotorControllerOnActiveStates(pmotor);
}

/**
 * Executes actions on entry to the STOPPING state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnStoppingInit(MCAF_MOTOR_DATA *pmotor)
{
    if (MCAF_StoppingClosedLoopCurrent())
    {
        pmotor->velocityControl.velocityCmd = 0;
        if (!MCAF_StoppingClosedLoopVelocity())
        {
            pmotor->idqCmdRaw.d = 0;
            pmotor->idqCmdRaw.q = 0;
            MCAF_ClearClosedLoopVelocity(pmotor);
        }
    }
    else
    {
        pmotor->omegaCmd = 0;
        MCAF_SetPwmMinimalImpact();
        MCAF_ClearClosedLoopFlags(pmotor);
    }
    MCAF_IncrementStopCount(pmotor);
    MCAF_StoppingTimerReset(pmotor);
}

/**
 * Executes actions in the STOPPING state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnStopping(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_ResetRecoveryIfNotRunRequested(pmotor);
    if (MCAF_StoppingClosedLoopCurrent())
    {
        MCAF_MotorControllerOnActiveStates(pmotor);
    }
}

/**
 * Executes actions on entry to the FAULT state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnFaultInit(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->ui.run = false;
    MCAF_SetPwmMinimalImpact();
    MCAF_ClearClosedLoopFlags(pmotor);
}

/**
 * Executes actions in the FAULT state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnFault(MCAF_MOTOR_DATA *pmotor)
{
    /* do nothing */
}

/**
 * Executes actions on entry to the TEST_DISABLE state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnTestDisableInit(MCAF_MOTOR_DATA *pmotor)
{
    /* do nothing */
}

/**
 * Executes actions in the TEST_DISABLE state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnTestDisable(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_SetPwmMinimalImpact();
}

/**
 * Executes actions in the TEST_RESTART state.
 * 
 * @param pmotor motor state data
 * @param init whether this is entry to the TEST_RESTART state
 */
inline static void MCAF_MotorControllerOnTestRestart(MCAF_MOTOR_DATA *pmotor, bool init)
{
    /* First time here: we need to delay for at least one cycle.
     */
    if (init)
    {
        /* 
         * Put PWMs in a safe state to keep gate drive going.
         * Clear latching PWM fault and delay for at least one ISR cycle.
         */
        MCAF_BOARD_DATA *pboard = &pmotor->psys->board;
        MCAF_BootstrapChargeInit(pboard);
        HAL_PWM_FaultClearBegin();
        MCAF_UiRestart(&pmotor->ui);
        MCAF_FocRestart(pmotor);
        
        MCAF_FocInitializeIntegrators(pmotor);
    }
    /* Otherwise, delay some number of samples for fault-latch circuitry 
     * to stabilize, then re-enable PWM fault latching.
     * (PWM peripheral requires the interval between disable
     * and reenable to straddle the PWM boundary)
     */
    else if (!MCAF_OvercurrentHWFlagValid(pmotor))
    {
        MCAF_OvercurrentHWFlagAttemptClear(pmotor);
    }    
    else
    {
        MCAF_BOARD_DATA *pboard = &pmotor->psys->board;
        bool bootstrapComplete = MCAF_BootstrapChargeStepIsr(pboard);
        if (bootstrapComplete)
        {
            MCAF_TestHarness_ClearRestartRequired(&pmotor->testing);
            MCAF_ADCCalibrateCurrentOffsets(&pmotor->initialization,
                                        &pmotor->currentCalibration,
                                        &pmotor->iabc,
                                        pmotor->iDC);
        }
    }    
}

/**
 * Executes actions on entry to the TEST_ENABLE state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnTestEnableInit(MCAF_MOTOR_DATA *pmotor, MCAF_FSM_STATE previous_state)
{
    /* Special-case entering the enable state from the following circumstance:
     * - STARTUP_PAUSE override flag set
     * - left STARTING state 
     */
    if (   MCAF_OverrideStartupPause(&pmotor->testing)
        && previous_state == MCSM_STARTING)
    {
        ; 
        /*
         * do nothing: we don't want to mess with the PWM duty cycles
         * in a mode where switching is already occurring
         */
    }
    else
    {
        EnablePwmMinDuty();
    }
}

/**
 * Executes actions in the TEST_ENABLE state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnTestEnable(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_MotorControllerOnActiveStates(pmotor);
}

/**
 * Executes actions on entry to the RESTART state.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnRestartInit(MCAF_MOTOR_DATA *pmotor)
{
    /* 
     * Put PWMs in a safe state to keep gate drive going.
     * Clear latching PWM fault and delay for at least one ISR cycle.
     */
    MCAF_BOARD_DATA *pboard = &pmotor->psys->board;
    MCAF_BootstrapChargeInit(pboard);
    HAL_PWM_FaultClearBegin();
    
    MCAF_FocRestart(pmotor);
    
    MCAF_UiRestart(&pmotor->ui);
    MCAF_TestHarness_Restart(&pmotor->testing);
    MCAF_CommutationRestart(pmotor);
#if MCAF_INCLUDE_STALL_DETECT      
    MCAF_StallDetectReset(&pmotor->stallDetect);
    MCAF_StallDetectDeactivate(&pmotor->stallDetect);
#endif    
    MCAF_FaultDetectInit(&pmotor->faultDetect);
    MCAF_RecoveryInit(&pmotor->recovery);
    MCAF_CurrentMeasureRestart(&pmotor->currentMeasure);
}

inline static bool restartStateComplete(const MCAF_MOTOR_DATA *pmotor)
{
    /* ADC initialization is the last phase of the RESTART state; see 
     * MCAF_MotorControllerOnRestart
     */
    return MCAF_ADCInitializationReady(pmotor);
}

/**
 * Executes actions in the RESTART state.
 * 
 * @param pmotor motor state data
 * @param init true if this is the first time we enter this state
 */
inline static void MCAF_MotorControllerOnRestart(MCAF_MOTOR_DATA *pmotor, bool init)
{
    /* First time here: we need to delay for at least one cycle.
     */
    if (init)
    {
        /* do nothing */
    }
    /* Otherwise, delay some number of samples for fault-latch circuitry 
     * to stabilize, then re-enable PWM fault latching.
     * (PWM peripheral requires the interval between disable
     * and reenable to straddle the PWM boundary)
     */
    else if (!MCAF_OvercurrentHWFlagValid(pmotor))
    {
        MCAF_OvercurrentHWFlagAttemptClear(pmotor);
    }
    /* Calibrate current offsets when the bootstrap charging sequence
     * is complete.
     */
    else
    {
        MCAF_BOARD_DATA *pboard = &pmotor->psys->board;
        bool bootstrapComplete = MCAF_BootstrapChargeStepIsr(pboard);
        if (bootstrapComplete)
        {
            MCAF_ADCCalibrateCurrentOffsets(&pmotor->initialization,
                                        &pmotor->currentCalibration,
                                        &pmotor->iabc,
                                        pmotor->iDC); 
        }
    }
}

/**
 * Executes actions common to all states.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnAllStates(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_ADCRead(pmotor);
    MCAF_FocStepIsrFeedbackPath(pmotor);
    MCAF_CommutationPrepareStallDetectInputs(pmotor);
    MCAF_MonitorSysDiagnose(pmotor);
}

/**
 * Executes low-priority actions common to all states.
 * 
 * @param pmotor motor state data
 */
inline static void MCAF_MotorControllerOnAllStatesLowPriority(MCAF_MOTOR_DATA *pmotor)
{   
    MCAF_FocStepIsrNonCriticalTask(pmotor);
    MCAF_TestGuard(&pmotor->testing, &pmotor->psys->testing);
    MCAF_TestPerturbationUpdate(&pmotor->testing);
}

inline static bool stopping_complete(MCAF_MOTOR_DATA *pmotor)
{
    if (MCAF_StoppingClosedLoopCurrent())
    {
        // Reset timer if the speed is too high
        const MCAF_U_VELOCITY_ELEC speed = UTIL_Abs16Approx(pmotor->omegaElectrical);
        if (speed > pmotor->stopping.speedThreshold)
        {
            MCAF_StoppingTimerReset(pmotor);
            return false;
        }
    }

    const bool timerExpired = MCAF_StoppingTimerUpdate(pmotor);
    const bool acceptTimerExpiration = MCAF_StoppingClosedLoopCurrent() || MCAF_MonitorIsMotorStopped(pmotor);
    return acceptTimerExpiration && timerExpired;
}

/**
 * Determines transition in state machine in the primary modes 
 * (those modes aside from test modes)
 *
 * Side effect: report any new error to the UI.
 * 
 * @param pmotor motor data
 * @return next state
 */
inline static MCAF_FSM_STATE MCAF_FSM_DetermineNextState(MCAF_MOTOR_DATA *pmotor)
{
    /* By default, we stay in the same state. */
    const MCAF_FSM_STATE this_state = pmotor->state;
    MCAF_FSM_STATE next_state = this_state;    
    
    /* Check if a fault has been detected. 
     * Mask reporting of new faults during the FAULT state; this is okay,
     * because if they are still around when we re-enter the RESTART state,
     * we'll drop back into the FAULT state.
     */
    const bool operating_fault_detected = 
        (next_state != MCSM_FAULT) &&
        MCAF_DetectFault(pmotor, FDM_PRIMARY);

    if (operating_fault_detected)
    {
        next_state = MCSM_FAULT;
        const uint16_t error_code = MCAF_GetFaultCode(pmotor);
        MCAF_UiRecordNewError(&pmotor->ui.indicatorState, error_code);
    }
    else
    {
        const bool directionChanged = MCAF_TestAndClearDirectionChangeFlag(&pmotor->ui);
        const bool run_requested = pmotor->ui.run && !directionChanged;
        const bool run_permitted =
            MCAF_MonitorIsRunPermitted(pmotor)
         || MCAF_OverrideStallDetection(&pmotor->testing);
        const bool run = run_requested && run_permitted;
        switch (this_state)
        {
            case MCSM_RESTART:
                {
                    if (restartStateComplete(pmotor))
                    {
                        if (MCAF_StoppingClosedLoopCurrent() 
                                && !MCAF_StartupHasCompleted(&pmotor->startup))
                        {
                            next_state = MCSM_STOPPED;
                        }
                        else
                        {
                            next_state = MCSM_STOPPING;
                        }
                    }
                }
                break;
            case MCSM_STOPPED:
                if (run) 
                {
                    next_state = MCSM_STARTING;
                }
                break;
            case MCSM_STARTING:
                if (!run)
                {
                    next_state = MCSM_STOPPING;
                }
                else if (MCAF_StartupHasCompleted(&pmotor->startup))
                {
                    next_state = MCSM_RUNNING;
                }
                break;
            case MCSM_RUNNING:
                if (!run)
                {
                    next_state = MCSM_STOPPING;
                }
                break;
            case MCSM_STOPPING:
                if (MCAF_StoppingClosedLoopCurrent() 
                        && run
                        && MCAF_StartupHasCompleted(&pmotor->startup))
                {
                    next_state = MCSM_RUNNING;
                }
                else if (stopping_complete(pmotor))
                {
                    next_state = MCSM_STOPPED;
                }
                break;
            case MCSM_FAULT:
                {
                    const bool ok_to_restart = pmotor->ui.exitFaultState;
                    if (ok_to_restart)
                    {
                        next_state = MCSM_RESTART;
                    }
                }
                break;
            default:
                next_state = MCSM_RESTART;
        }
    }
    pmotor->ui.exitFaultState = false;   // clear any pending requests
    return next_state;
}

/**
 * Determines transition in state machine in the test modes
 * 
 * Side effect: report any new error to the UI.
 * 
 * @param pmotor motor data
 * @return next state
 */
inline static MCAF_FSM_STATE MCAF_FSM_DetermineNextStateTestMode(MCAF_MOTOR_DATA *pmotor)
{    
    const MCAF_FSM_STATE this_state = pmotor->state;
    MCAF_FSM_STATE next_state = this_state;    
    const bool test_fault_detected = MCAF_DetectFault(pmotor, FDM_TEST);
    const bool test_fault_newly_detected = test_fault_detected &&
                                           (this_state != MCSM_FAULT);

    if (test_fault_newly_detected)
    {
        next_state = MCSM_FAULT;
        const uint16_t error_code = MCAF_GetFaultCode(pmotor);
        MCAF_UiRecordNewError(&pmotor->ui.indicatorState, error_code);
    }
    else
    {
        switch (this_state)
        {
        case MCSM_FAULT:
            {
                const bool ok_to_restart = pmotor->ui.exitFaultState;
                if (ok_to_restart)
                {
                    next_state = MCSM_TEST_RESTART;
                }
            }
            break;
        case MCSM_TEST_RESTART:
            {
                if (restartStateComplete(pmotor))
                {
                    next_state = MCSM_TEST_DISABLE;
                }        
            }
            break;
        default:
            /* All other states: ignore which state we're in,
             * and instead determine next state based only on operating mode
             * and whether a clean restart is required.
             * 
             * Please note that this_state may be one of the non-test states
             * if operating mode is suddenly switched from OM_NORMAL
             * to one of the test modes.
             * 
             * This gives the test operator one of two ways to enter a test state:
             * 
             * - just change operating mode to OM_xyz: changes abruptly
             * - set motor.testing.testRestartRequired = true,
             *   and *then* change operating mode to OM_xyz:
             *   goes through MCSM_TEST_RESTART first,
             *   then switches to MCSM_TEST_ENABLE
             */
            if (MCAF_GetOperatingMode(&pmotor->testing) == OM_DISABLED)
            {
                next_state = MCSM_TEST_DISABLE;
            }
            else  // operating mode != disabled
            {
                if (MCAF_TestHarness_TestRestartRequired(&pmotor->testing))
                {
                    next_state = MCSM_TEST_RESTART;
                }
                else
                {
                    next_state = MCSM_TEST_ENABLE;
                }
            }
        }
    }
    pmotor->ui.exitFaultState = false;   // clear any pending requests
    return next_state;
}

/**
 * Performs actions appropriate in each state, including on-entry actions
 * 
 * @param pmotor motor state data
 * @param next_state the next state of the state machine
 */
inline static void MCAF_FSM_Dispatch(MCAF_MOTOR_DATA *pmotor, MCAF_FSM_STATE next_state)
{
    const MCAF_FSM_STATE this_state = pmotor->state;
    const bool state_changed = (next_state != this_state);
    pmotor->state = next_state;
    
    switch (next_state)
    {
        case MCSM_RESTART:
            if (state_changed)
            {
                MCAF_MotorControllerOnRestartInit(pmotor);
            }
            MCAF_MotorControllerOnRestart(pmotor, state_changed);
            break;
        case MCSM_STOPPED:
            if (state_changed)
            {
                MCAF_MotorControllerOnStoppedInit(pmotor);
            }
            MCAF_MotorControllerOnStopped(pmotor);
            break;
        case MCSM_STARTING:
            if (state_changed)
            {
               MCAF_MotorControllerOnStartingInit(pmotor);
            }
            MCAF_MotorControllerOnStarting(pmotor);
            break;
        case MCSM_RUNNING:
            if (state_changed)
            {
                MCAF_MotorControllerOnRunningInit(pmotor);
            }
            MCAF_MotorControllerOnRunning(pmotor);
            break;
        case MCSM_STOPPING:
            if (state_changed)
            {
                MCAF_MotorControllerOnStoppingInit(pmotor);
            }
            MCAF_MotorControllerOnStopping(pmotor);
            break;                    
        case MCSM_FAULT:
            if (state_changed)
            {
                MCAF_MotorControllerOnFaultInit(pmotor);
            }
            MCAF_MotorControllerOnFault(pmotor);
            break;                    
        case MCSM_TEST_DISABLE:
            if (state_changed)
            {
                MCAF_MotorControllerOnTestDisableInit(pmotor);
            }
            MCAF_MotorControllerOnTestDisable(pmotor);
            break;                    
        case MCSM_TEST_RESTART:
            MCAF_MotorControllerOnTestRestart(pmotor, state_changed);
            break;
        case MCSM_TEST_ENABLE:
            if (state_changed)
            {
                MCAF_MotorControllerOnTestEnableInit(pmotor, this_state);
            }
            MCAF_MotorControllerOnTestEnable(pmotor);
            break;
    }    
}


/* ----- implementations of functions used outside this module: ----- */

void MCAF_SystemStateMachine_StepIsr(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_CaptureTimestamp(&pmotor->testing, MCTIMESTAMP_STATEMACH_START);

    /* 1. Perform critical tasks that are independent of the state. */
    MCAF_MotorControllerOnAllStates(pmotor);
    
    MCAF_CaptureTimestamp(&pmotor->testing, MCTIMESTAMP_STATEMACH_ON_ALL_STATES);

    /* 2. Determine next state. */
    const MCAF_FSM_STATE next_state =
    #ifdef MCAF_TEST_HARNESS
        pmotor->testing.operatingMode == OM_NORMAL 
            ? MCAF_FSM_DetermineNextState(pmotor)
            : MCAF_FSM_DetermineNextStateTestMode(pmotor);
    #else
        MCAF_FSM_DetermineNextState(pmotor);
    #endif

    MCAF_CaptureTimestamp(&pmotor->testing, MCTIMESTAMP_STATEMACH_NEXT_STATE);
    
    /* 3. Update state and execute appropriate actions. */
    MCAF_FSM_Dispatch(pmotor, next_state);

    MCAF_CaptureTimestamp(&pmotor->testing, MCTIMESTAMP_STATEMACH_DISPATCH);

    /* 4. Perform noncritical tasks that are independent of the state. */
    MCAF_MotorControllerOnAllStatesLowPriority(pmotor);

    MCAF_CaptureTimestamp(&pmotor->testing, MCTIMESTAMP_STATEMACH_END);
}

void MCAF_SystemStateMachine_Init(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->state = MCSM_RESTART;
    pmotor->stateFlags = 0;
    pmotor->stopping.timer.duration = MCAF_StoppingClosedLoopCurrent()
                                 ? MCAF_CLOSED_LOOP_STOPPING_TIME
                                 : VELOCITY_COASTDOWN_TIME;        
    pmotor->stopping.speedThreshold = MCAF_CLOSED_LOOP_STOPPING_SPEED;
    MCAF_TestHarness_Init(&pmotor->testing);
    MCAF_MotorControllerOnRestartInit(pmotor);    
    MCAF_CommutationInit(pmotor);
    pmotor->initialization.faultLatchDelay = MCAF_POWERUP_FAULT_LATCH_DELAY;
    
    /* Start Timer-1, used for implementing time profiling
     * feature within the test harness */
    HAL_ProfilingCounter_Start();
}

void MCAF_SystemStateMachine_StepMain(volatile MCAF_MOTOR_DATA *pmotor)
{
    /* Fault display is stateful: we only set it up once,
     * and afterwards we leave things alone. 
     *
     * Regular status display is stateless: it's just a slow PWM
     * for the LEDs
     */
    if (pmotor->state != MCSM_FAULT)
    {
        uint8_t duty1;
        uint8_t duty2;
        /* Mapping between system state and duty cycle of LED blink */
        switch (pmotor->state)
        {
            case MCSM_RESTART:
            case MCSM_TEST_RESTART:
            default:
                duty1 = MCUILD_OFF;
                duty2 = MCUILD_OFF;
                break;
            case MCSM_STOPPED:
                duty1 = MCUILD_LOW;
                duty2 = MCUILD_OFF;
                break;
            case MCSM_STARTING:
                duty1 = MCUILD_LOW;
                duty2 = MCUILD_ON;
                break;
            case MCSM_STOPPING:
                duty1 = MCUILD_HIGH;
                duty2 = MCUILD_OFF;
                break;
            case MCSM_RUNNING:
                duty1 = MCUILD_HIGH;
                duty2 = MCUILD_ON;
                break;
        }

        if (pmotor->ui.flags & MCAF_UI_REVERSE)
        {
            pmotor->ui.indicatorState.duty1 = duty1;        
            pmotor->ui.indicatorState.duty2 = duty2;        
        }
        else
        {
            pmotor->ui.indicatorState.duty1 = duty2;        
            pmotor->ui.indicatorState.duty2 = duty1;        
        }
    }
    MCAF_TestHarnessHandleForceStateChange(&pmotor->testing, &pmotor->ui.run);
}
