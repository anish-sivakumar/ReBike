/**
 * test_harness.h
 * 
 * Test harness definitions
 * 
 * Component: test harness
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

#ifndef __TEST_HARNESS_H
#define __TEST_HARNESS_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_control_types.h"
#include "hal.h"
#include "units.h"
#include "util.h"
#include "parameters/options.h"

#ifdef __cplusplus
extern "C" {
#endif

/* define timestamps for profiling */
    
#define MCTH_TIMESTAMP(NAME,VAL) MCTIMESTAMP_##NAME=VAL,
    
enum MCAF_TIMESTAMP_NAMES {
#include "test_harness_timestamps.h"
};
    
#undef MCTH_TIMESTAMP
    
/**
 * Test operating mode
 */
typedef enum tagMCAF_OPERATING_MODE 
{
    OM_DISABLED = 0,          /** motor disabled */
    OM_FORCE_VOLTAGE_DQ = 1,  /** applying dq-frame voltage */
    OM_FORCE_CURRENT = 2,     /** current loop enabled */
    OM_NORMAL = 3             /** current and velocity loop enabled */
} MCAF_OPERATING_MODE;

enum MCAF_TEST_CONSTANTS
{
    TEST_OVERRIDE_VELOCITY_COMMAND     = 1,  /** Override velocity command */
    TEST_OVERRIDE_COMMUTATION          = 2,  /** Override commutation angle */
    TEST_OVERRIDE_DC_LINK_COMPENSATION = 4,  /** Override DC link compensation */
    TEST_OVERRIDE_STALL_DETECTION      = 8,  /** Override stall detection */
    TEST_OVERRIDE_D_AXIS_VOLTAGE_PRIORITY = 16, /** Override to give d-axis voltage priority */
    TEST_OVERRIDE_STARTUP_PAUSE        = 32, /** Override to pause startup upon reaching the HOLD state */
    TEST_OVERRIDE_FLUX_CONTROL         = 64, /** Override flux control */
    TEST_OVERRIDE_ZERO_SEQUENCE_MODULATION = 128, /** Override zero-sequence modulation */
    
    TEST_GUARD_VALID = 0xD1A6,    /** valid guard key */
    TEST_GUARD_RESET = 0x0000,    /** value set at reset */
    
    TEST_FLAGS_SEIZURE_MAINLOOP = 0x0001, /** causes main loop to seize */
    TEST_FLAGS_SEIZURE_ISR      = 0x0002, /** causes ISR to seize */
    TEST_FLAGS_STACK_OVERFLOW   = 0x0004, /** causes stack overflow */

    TEST_FORCE_STATE_INACTIVE = 0,          /** No action */
    TEST_FORCE_STATE_RUN      = 1,          /** Run (turns ui.run to true) */
    TEST_FORCE_STATE_STOP     = 2,          /** Stop (turns ui.run to false) */
    TEST_FORCE_STATE_STOP_NOW = 3,          /** Stop immediately */
};

/**
 * Test harness uneven commutation (on/off)
 */
typedef struct tagMCAF_TEST_COMMUTATION_ON_OFF
{
    uint16_t maxCount;        /** (period - 1) in ISR counts */
    uint16_t counter;         /** counter */
    uint16_t threshold;       /** threshold, below which we apply omegaElectrical */
} MCAF_TEST_COMMUTATION_ON_OFF_T;

/**
 * Perturbation on each phase of asymmetric perturbation
 * (not applicable for symmetric square-wave perturbation)
 */
typedef struct {
    uint32_t duration;            /** duration, in cycles, or zero to stop */
    
    /* values */
    MCAF_U_VELOCITY   velocity;   /** velocity command perturbation */
    MCAF_U_CURRENT_DQ idq;        /** dq-axis current perturbation */
    MCAF_U_VOLTAGE_DQ vdq;        /** dq-axis voltage perturbation */
} MCAF_TEST_PERTURB_PHASE;

/**
 * Bit flags for asymmetric perturbation
 * (not applicable for symmetric square-wave perturbation)
 */
typedef enum {
    MCAF_TPF_PHASE       =  1 /** which phase is active? */
} MCAF_TEST_PERTURB_FLAGS;


/**
 * Autostepping of current perturbation
 * (not applicable for symmetric square-wave perturbation)
 */
typedef struct {
    uint16_t count;               /** number of steps */

    /* values */
    MCAF_U_CURRENT_DQ idq;        /** dq-axis current step */
} MCAF_TEST_PERTURB_STEP;

/**
 * Test harness state variables, per-motor
 */
typedef struct tagMCAF_MOTOR_TEST_MANAGER
{
#ifdef MCAF_TEST_HARNESS
#if MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC
    /** Square-wave state variables */
    struct tagSqWave 
    {
        uint32_t halfperiod;      /** half-period of square-wave disturbance, in control cycles */

        /* amplitudes */
        MCAF_U_VELOCITY   velocity;        /** amplitude of velocity command perturbation */
        MCAF_U_CURRENT_DQ idq;             /** amplitudes of dq-axis current perturbation */
        MCAF_U_VOLTAGE_DQ vdq;             /** amplitudes of dq-axis voltage perturbation */

        /* mutable state */
        int16_t  value;           /** value of the square wave */
        uint32_t count;           /** square-wave counter for each half-period */
    } sqwave;                     /** square-wave state variables */
#else  // MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC == 0
    /** Pulse-wave state variables */
    struct tagPerturbation
    {
        MCAF_TEST_PERTURB_PHASE phase[2];     /** each phase */
        volatile MCAF_TEST_PERTURB_PHASE *activePhase; /** which phase is active */
        uint32_t count;                       /** time counter for each phase */        
        uint16_t flags;                       /** flags: see  MCAF_TEST_PERTURB_FLAGS */
        uint16_t autobalanceRatio;            /** automatic balancing of the second phase from the first */
        int16_t  enable;                      /** 1 or 0 to enable/disable perturbation */
        MCAF_TEST_PERTURB_STEP step;          /** automatic stepping for some number of steps */
    } perturb;
#endif // MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC
    
    uint16_t                overrides;       /** override bits */
    
    /** commutation frequency to use in case TEST_OVERRIDE_VELOCITY_COMMAND is set */
    MCAF_U_VELOCITY_DTHETA_ELEC_DT overrideOmegaElectrical; 
    MCAF_TEST_COMMUTATION_ON_OFF_T overrideCommutationOnOff; /** override commutation unevenly */
    MCAF_U_DUTYCYCLE               overrideZeroSequenceOffset; /** override offset for zero sequence modulation */
    MCAF_OPERATING_MODE    operatingMode;   /** operating mode */
    
    uint16_t    forceStateChange;
    bool        stopNow;
    bool        testRestartRequired;
#endif
   
#ifdef MCAF_TEST_PROFILING
#define MCAF_PROFILING_TIMESTAMP_CAPACITY 8
    uint16_t    timestampReference; /** reference for timestamps */
    uint16_t    timestamps[MCAF_PROFILING_TIMESTAMP_CAPACITY];   /** timestamps for profiling */
#endif
} MCAF_MOTOR_TEST_MANAGER;

/**
 * Test harness state variables, per-system
 */
typedef struct tagMCAF_SYSTEM_TEST_MANAGER
{
    uint16_t flags;              /** system-wide test flags */
                                  
    /** Guard state variables */
    struct tagGuard
    {
        uint16_t key;            /** guard key */
        uint16_t timeout;        /** timeout counter */
    } guard;                     /** guard state variables */
} MCAF_SYSTEM_TEST_MANAGER;

/**
 * State variables for manually-triggered average value calculation routine
 */
typedef struct tagMCAF_TRIGGERED_AVERAGE
{
    bool triggerActive;     /** Flag to begin routine */
    uint16_t sampleCount;   /** Number of samples to acquire */
    uint16_t shiftCount;    /** Number of right bit-shifts to calculate average */
    uint16_t count;         /** Samples remaining to acquire */
    int32_t sum;            /** Sum of inputs to average */
    int16_t average;        /** Result of average calculation */
} MCAF_TRIGGERED_AVERAGE_T;


/* accessor functions for checking override status */

/**
 * Returns <code>true</code> if velocity command override is set.
 * 
 * @param ptest testing state
 * @return <code>true</code> if velocity command override is set.
 */
inline static bool MCAF_OverrideVelocityCommand(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS 
    return ptest->overrides & TEST_OVERRIDE_VELOCITY_COMMAND;
#else
    return false;
#endif
}

/**
 * Returns <code>true</code> if commutation override is set.
 * 
 * @param ptest testing state
 * @return <code>true</code> if commutation override is set.
 */
inline static bool MCAF_OverrideCommutation(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS 
    return ptest->overrides & TEST_OVERRIDE_COMMUTATION;
#else
    return false;
#endif
}

/**
 * Returns <code>true</code> if DC link compensation override is set.
 * 
 * @param ptest testing state
 * @return <code>true</code> if DC link compensation override is set.
 */
inline static bool MCAF_OverrideDCLinkCompensation(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    return ptest->overrides & TEST_OVERRIDE_DC_LINK_COMPENSATION;
#else
    return false;
#endif
}

/**
 * Returns <code>true</code> if stall detection override is set.
 * 
 * @param ptest testing state
 * @return <code>true</code> if stall detection override is set.
 */
inline static bool MCAF_OverrideStallDetection(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    return ptest->overrides & TEST_OVERRIDE_STALL_DETECTION;
#else
    return false;
#endif
}

/**
 * Returns <code>true</code> if d-axis voltage priority override is set.
 * 
 * @param ptest testing state
 * @return <code>true</code> if d-axis voltage priority override is set.
 *   (d-axis voltage has priority; the default case is that d- and q-axes
 *    have equal competing priority)
 */
inline static bool MCAF_OverrideDAxisVoltagePriority(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    return ptest->overrides & TEST_OVERRIDE_D_AXIS_VOLTAGE_PRIORITY;
#else
    return false;
#endif
}

/**
 * Returns <code>true</code> if startup-pause override is set.
 * (This will keep the motor in open-loop commutation after acceleration.)
 * 
 * @param ptest testing state
 * @return <code>true</code> if startup-pause override is set.
 */
inline static bool MCAF_OverrideStartupPause(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    return ptest->overrides & TEST_OVERRIDE_STARTUP_PAUSE;
#else
    return false;
#endif
}

/**
 * Returns <code>true</code> if flux-control override is set.
 * (This will disable calculation of D-axis current.)
 * 
 * @param ptest testing state
 * @return <code>true</code> if flux-control override is set.
 */
inline static bool MCAF_OverrideFluxControl(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    return ptest->overrides & TEST_OVERRIDE_FLUX_CONTROL;
#else
    return false;
#endif
}

/**
 * Returns <code>true</code> if zero-sequence modulation override is set.
 * (This will disable ZSM calculation.)
 * 
 * @param ptest testing state
 * @return <code>true</code> if zero-sequence modulation override is set.
 */
inline static bool MCAF_OverrideZeroSequenceModulation(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    return ptest->overrides & TEST_OVERRIDE_ZERO_SEQUENCE_MODULATION;
#else
    return false;
#endif
}

/**
 * Returns zero sequence offset override.
 * 
 * @param ptest testing state
 * @return zero sequence offset override
 */
inline static MCAF_U_DUTYCYCLE MCAF_GetOverrideZeroSequenceOffset(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    return ptest->overrideZeroSequenceOffset;
#else
    return 0;
#endif
}

/**
 * Returns <code>true</code> if we're in a normal operating mode.
 * 
 * @param ptest testing state
 * @return <code>true</code> if we're in a normal operating mode.
 */
inline static bool MCAF_OperatingModeNormal(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    return ptest->operatingMode == OM_NORMAL;
#else
    return true;
#endif
}

/**
 * Returns <code>true</code> if we're in an operating mode where the current loop is active
 * 
 * @param ptest testing state
 * @return <code>true</code> if we're in an operating mode where the current loop is active
 */
inline static bool MCAF_OperatingModeCurrentLoopActive(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    return ptest->operatingMode >= OM_FORCE_CURRENT;
#else
    return true;
#endif
}

/**
 * Returns test perturbation velocity 
 * 
 * @param ptest testing state 
 * @return test perturbation velocity
 */
inline static MCAF_U_VELOCITY_ELEC MCAF_TestPerturbationVelocity(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
  #if MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC
    return ptest->sqwave.value * ptest->sqwave.velocity.electrical;
  #else
    return ptest->perturb.enable * ptest->perturb.activePhase->velocity.electrical;
  #endif
#else
    return 0;
#endif
}

/**
 * Returns test perturbation q-axis current
 * 
 * @param ptest testing state 
 * @return test perturbation q-axis current
 */
inline static int16_t MCAF_TestPerturbationIq(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
  #if MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC
    return ptest->sqwave.value * ptest->sqwave.idq.q;
  #else
    return ptest->perturb.enable * ptest->perturb.activePhase->idq.q;
  #endif
#else
    return 0;
#endif
}

/**
 * Returns test perturbation d-axis current
 * 
 * @param ptest testing state 
 * @return test perturbation d-axis current
 */
inline static int16_t MCAF_TestPerturbationId(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
  #if MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC
    return ptest->sqwave.value * ptest->sqwave.idq.d;
  #else
    return ptest->perturb.enable * ptest->perturb.activePhase->idq.d;
  #endif
#else
 return 0;
#endif
}

/**
 * Returns test perturbation q-axis voltage
 * 
 * @param ptest testing state 
 * @return test perturbation q-axis voltage
 */
inline static int16_t MCAF_TestPerturbationVq(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
  #if MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC
    return ptest->sqwave.value * ptest->sqwave.vdq.q;
  #else
    return ptest->perturb.enable * ptest->perturb.activePhase->vdq.q;
  #endif
#else
    return 0;
#endif
}

/**
 * Returns test perturbation d-axis voltage
 * 
 * @param ptest testing state 
 * @return test perturbation d-axis voltage
 */
inline static int16_t MCAF_TestPerturbationVd(const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
  #if MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC
    return ptest->sqwave.value * ptest->sqwave.vdq.d;
  #else
    return ptest->perturb.enable * ptest->perturb.activePhase->vdq.d;
  #endif
#else
    return 0;
#endif
}

/**
 * Reinitializes state variables.
 * 
 * @param ptest testing state
 */
void MCAF_TestHarness_Restart(volatile MCAF_MOTOR_TEST_MANAGER *ptest);

/**
 * Initializes system-wide state variables.
 * 
 * @param ptest testing state
 */
void MCAF_SystemTestHarness_Init(volatile MCAF_SYSTEM_TEST_MANAGER *ptest);

/**
 * Returns whether a clean test restart is required 
 * 
 * @param ptest testing state
 * @return true if clean test restart is required
 */
inline static bool MCAF_TestHarness_TestRestartRequired(volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    return ptest->testRestartRequired;
#else
    return false;
#endif
}

/**
 * Clear restart flag: clean test restart is no longer required
 * 
 * @param ptest testing state
 */
inline static void MCAF_TestHarness_ClearRestartRequired(volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    ptest->testRestartRequired = false;
#endif
}

/**
 * Return commutation frequency used when commutation inputs are overridden by the test harness
 * 
 * @param ptest testing state
 * @return commutation frequency
 */
inline static MCAF_U_VELOCITY_DTHETA_ELEC_DT MCAF_GetOverrideCommutationFrequency (volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    /* return overrideOmegaElectrical for k cycles, 0 for N-k cycles,
     * where N-1 = overrideCommutationOnOff.maxCount
     * and k = overrideCommutationOnOff.threshold
     */
    volatile MCAF_TEST_COMMUTATION_ON_OFF_T *pOnOff = &ptest->overrideCommutationOnOff;
    if (pOnOff->counter == 0)
    {
        pOnOff->counter = pOnOff->maxCount;
    }
    else
    {
        --pOnOff->counter;
    }
    
    if (pOnOff->counter < pOnOff->threshold)
    {
        return ptest->overrideOmegaElectrical;
    }
    else
    {
        return 0;
    }
#else
    return 0;
#endif
}

inline static void MCAF_TestHarness_Init(volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    ptest->operatingMode = OM_NORMAL;
    ptest->forceStateChange = TEST_FORCE_STATE_INACTIVE;
    ptest->stopNow = false;
#endif
}

inline static MCAF_OPERATING_MODE MCAF_GetOperatingMode (const volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    return ptest->operatingMode;
#else
    return OM_NORMAL;
#endif
}

inline static void MCAF_TestPerturbationUpdate(volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
  #if MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC
    const int16_t value = ptest->sqwave.value;
    /* update test perturbation waveform */
    if (value != 0)
    {
        /* Ensure square wave value maintains sign and is +/- 1 */
        const int16_t valueLimited = UTIL_SignFromHighBit(value);
        
        /*
         * Reverse sign after N cycles (N = sqwave.halfperiod)
         */
        if (++ptest->sqwave.count >= ptest->sqwave.halfperiod)
        {
            ptest->sqwave.value = -valueLimited;
            ptest->sqwave.count = 0;
        }    
        else
        {
            ptest->sqwave.value = valueLimited;
        }
    }
  #else // MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC == 0
    if (ptest->perturb.enable == 0)
    {
        return;
    }
    
    ptest->perturb.enable = UTIL_SignFromHighBit(ptest->perturb.enable);
    if (++ptest->perturb.count >= ptest->perturb.activePhase->duration)
    {
        // Time to start over and switch phases!
        ptest->perturb.count = 0;
        ptest->perturb.flags ^= MCAF_TPF_PHASE;

        const int16_t newphase = ptest->perturb.flags & MCAF_TPF_PHASE;
        volatile MCAF_TEST_PERTURB_PHASE * const activePhase = 
            &ptest->perturb.phase[newphase];
        ptest->perturb.activePhase = activePhase;
        if (activePhase->duration == 0)
        {
            /* Do we need to produce some number of steps? */
            if (ptest->perturb.step.count > 1)
            {
                --ptest->perturb.step.count;
                ptest->perturb.phase[0].idq.q += ptest->perturb.step.idq.q;
                ptest->perturb.phase[0].idq.d += ptest->perturb.step.idq.d;
            }
            else
            {
                /* deactivate so that user just has to set perturb.enable = 1
                 */
                ptest->perturb.enable = 0;
            }
            
            /* reset to phase 0 */
            ptest->perturb.flags &= ~MCAF_TPF_PHASE;
            ptest->perturb.activePhase = &ptest->perturb.phase[0];
        }
        else
        {
            const uint16_t alpha = ptest->perturb.autobalanceRatio;
            if (alpha > 0 && newphase != 0)
            {
                /*
                 * Autobalance: set phase[1] values to be scaled from phase[0] values
                 * by the same ratio alpha, and invert sign.
                 */
                const volatile MCAF_TEST_PERTURB_PHASE *phase0 = &ptest->perturb.phase[0];
                activePhase->velocity.electrical = UTIL_MulUSQ16(alpha, phase0->velocity.electrical);
                activePhase->idq.d = UTIL_MulUSQ16(alpha, phase0->idq.d);
                activePhase->idq.q = UTIL_MulUSQ16(alpha, phase0->idq.q);
                activePhase->vdq.d = UTIL_MulUSQ16(alpha, phase0->vdq.d);
                activePhase->vdq.q = UTIL_MulUSQ16(alpha, phase0->vdq.q);
                ptest->perturb.enable = -1;
            }
            else
            {
                ptest->perturb.enable = 1;
            }
        }
    }    
  #endif // MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC
#endif
}

/**
 * Guards test state variables by resetting them to default values
 * if the specified system-wide guard key is not valid.
 * 
 * @param ptest test state
 * @param psystest system-wide test state
 */
inline static void MCAF_TestGuard(volatile MCAF_MOTOR_TEST_MANAGER *ptest, 
                                   const volatile MCAF_SYSTEM_TEST_MANAGER *psystest)
{
#ifdef MCAF_TEST_HARNESS
    if (psystest->guard.key != TEST_GUARD_VALID)
    {
        ptest->overrides     = 0;
        ptest->operatingMode = OM_NORMAL;
  #if MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC
        ptest->sqwave.value  = 0;
  #else
        ptest->perturb.enable = 0;
  #endif
    }
#endif
}

#ifdef MCAF_TEST_HARNESS
/**
 * Causes a "seizure" (infinite loop) if the guard key is valid and
 * the flags are tested against a mask.
 * 
 * @param psystest system-wide test state
 * @param mask test mask
 */
inline static bool MCAF_TestHarness_CheckFlags(
    const volatile MCAF_SYSTEM_TEST_MANAGER *psystest,
    uint16_t mask)
{

    return ((psystest->guard.key == TEST_GUARD_VALID)
         && (psystest->flags & mask));
}

inline static void MCAF_TestHarness_TriggerSeizure()
{
    while (true)
    {
        __builtin_nop(); /* prevent the compiler from optimizing this loop out */
    }
}
#endif

inline static void MCAF_TestHarnessStepIsr(const volatile MCAF_SYSTEM_TEST_MANAGER *psystest)
{
#ifdef MCAF_TEST_HARNESS
    if (MCAF_TestHarness_CheckFlags(psystest, TEST_FLAGS_SEIZURE_ISR))
    {
        MCAF_TestHarness_TriggerSeizure();
    }
#endif
}

inline static void MCAF_TestHarnessHandleForceStateChange(volatile MCAF_MOTOR_TEST_MANAGER *ptest, volatile bool *prun)
{
#ifdef MCAF_TEST_HARNESS
    if (ptest->forceStateChange != TEST_FORCE_STATE_INACTIVE)
    {
        switch (ptest->forceStateChange)
        {
            case TEST_FORCE_STATE_RUN:
                *prun = true;
                break;
            case TEST_FORCE_STATE_STOP:
                *prun = false;
                break;
            case TEST_FORCE_STATE_STOP_NOW:
                *prun = false;
                ptest->stopNow = true;
                break;
            default:
                /* no action */
                break;
        }        
        ptest->forceStateChange = TEST_FORCE_STATE_INACTIVE;
    }
#endif
}

inline static bool MCAF_TestHarnessCheckStopNowAndReset(volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS
    const bool result = ptest->stopNow;
    ptest->stopNow = false;
    return result;
#else
    return false;
#endif    
}

#ifdef MCAF_TEST_HARNESS
/**
 * Causes a stack overflow.
 */
void MCAF_TestHarness_TriggerStackOverflow(void);
#endif

inline static void MCAF_TestHarnessStepMain(const volatile MCAF_SYSTEM_TEST_MANAGER *psystest)
{
#ifdef MCAF_TEST_HARNESS
    if (MCAF_TestHarness_CheckFlags(psystest, TEST_FLAGS_SEIZURE_MAINLOOP))
    {
        MCAF_TestHarness_TriggerSeizure();
    }
    if (MCAF_TestHarness_CheckFlags(psystest, TEST_FLAGS_STACK_OVERFLOW))
    {
        MCAF_TestHarness_TriggerStackOverflow();
    }
#endif
}

/**
 * Capture a timestamp, for diagnostic purposes. 
 * Optimized out if MCAF_TEST_PROFILING is not enabled
 * 
 * @param ptest test state
 * @param k timestamp slot -- this should be known at compile time so the
 *                            compiler can optimize out the if-test
 */
inline static void MCAF_CaptureTimestamp(volatile MCAF_MOTOR_TEST_MANAGER *ptest, int k)
{
#ifdef MCAF_TEST_PROFILING
    if (k >= 0 && k < MCAF_PROFILING_TIMESTAMP_CAPACITY)
    {
        ptest->timestamps[k] = HAL_ProfilingCounter_Get() - ptest->timestampReference;
    }
#endif
}

/**
 * Initialize average calculation routine. This must be called before running
 * MCAF_TriggeredAverage_Step().
 * 
 * @param ptrigavg trigger average data
 * @param sampleCount number of samples to average
 * @param shiftCount number of right-shifts to compute average
 */
inline static void MCAF_TriggeredAverage_Init(MCAF_TRIGGERED_AVERAGE_T *ptrigavg,
                                                    uint16_t sampleCount,
                                                    uint16_t shiftCount)
{
#ifdef MCAF_TEST_HARNESS
    ptrigavg->triggerActive = false;
    ptrigavg->sampleCount = sampleCount;
    ptrigavg->shiftCount = shiftCount;
    ptrigavg->count = sampleCount;
    ptrigavg->sum = 0;
    ptrigavg->average = 0;
#endif
}

/**
 * Executes one step of the average calculation routine. When the desired sample
 * number is reached, the average of the samples is calculated and the routine 
 * is reset and disabled.
 * 
 * @param ptrigavg trigger average data
 * @param input variable to average
 */
inline static void MCAF_TriggeredAverage_Step(MCAF_TRIGGERED_AVERAGE_T *ptrigavg,
                                                    int16_t input)
{
#ifdef MCAF_TEST_HARNESS
    if (ptrigavg->triggerActive)
    {
        ptrigavg->sum += input;
        if (--ptrigavg->count == 0)
        {
            ptrigavg->average = UTIL_ShrS32N16(ptrigavg->sum, ptrigavg->shiftCount);
            ptrigavg->count = ptrigavg->sampleCount;
            ptrigavg->sum = 0;
            ptrigavg->triggerActive = false;
        }
    }
#endif
}

#ifdef __cplusplus
}
#endif

#endif /* __TEST_HARNESS_H */
