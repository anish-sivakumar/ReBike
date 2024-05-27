/**
 * system_state.h
 * 
 * Main system state variable structure definitions
 * 
 * Component: main application
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

#ifndef __SYSTEM_STATE_H
#define __SYSTEM_STATE_H

#include <stdint.h>
#include <stdbool.h>

#include "parameters/options.h" // pick up MCAF_INCLUDE_STALL_DETECT
#include "motor_control_types.h"
#include "units.h"
#include "startup_types.h"
#include "ui_types.h"
#include "sat_PI_types.h"
#include "stall_detect_types.h"
#include "fault_detect_types.h"
#include "state_machine_types.h"
#include "recover.h"
#include "monitor_types.h"
#include "adc_compensation_types.h"
#include "foc_types.h"
#include "test_harness.h"
#include "commutation_types.h"
#include "board_service_types.h"
#include "deadtimecomp_types.h"
#include "flux_control_types.h"
#include "dyn_current_types.h"
#include "current_measure_types.h"
#include "hal/hardware_access_functions_types.h"
#include "mcapi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Main system data. Variables are included here if they pertain
 * to the entire system, not just one motor axis.
 */
typedef struct tagMCAF_SYSTEM_DATA
{
    /** measured DC link voltage */
    MCAF_U_VOLTAGE vDC;
    
    /**
     * Debugging counters for making sure we are aware of resets
     * and how many times the motors have been stopped.
     */
    struct tagDebugCounters
    {
        uint16_t reset;   /** number of resets */
        uint16_t stop;    /** number of times the motors have been stopped */
    } debugCounters;
        
    volatile MCAF_SYSTEM_TEST_MANAGER testing;     /** system-wide test harness state */
    MCAF_BOARD_DATA board;                         /** system-wide board state */
} MCAF_SYSTEM_DATA;

typedef struct tagBridgeTemperature {
    uint16_t raw;               /** raw ADC reading */
    MCAF_U_TEMPERATURE processed; /** scaled/processed temperature */
    struct tagBridgeTemperatureFilter {
        MCAF_U_TEMPERATURE output;   /** filter output */
        uint16_t           gain;     /** filter gain */
        sx1632_t           state;    /** filter state */            
        int16_t            slewRate; /** slew rate */
    } filter;        /** filtering */
    uint16_t gain;              /** scaling gain */
    int16_t  offset;            /** offset */
} MCAF_BRIDGE_TEMPERATURE;       /** bridge temperature */


/**
 * Motor state data
 */
typedef struct tagMOTOR
{
    /* Current loop command */
    MCAF_U_CURRENT_DQ        idqCmdRaw;  /** Input command for the current loops, prior to rate limiting */
    MCAF_U_CURRENT_DQ        idqCmdPerturbed; /** Input command for the current loops, prior to rate limiting */
    MCAF_U_CURRENT_DQ        idqCmd;     /** Input command for the current loops */

    /* Current feedback path */
    MCAF_U_CURRENT_ABC       iabc;       /** phase current measurements */
    MCAF_U_CURRENT_ALPHABETA ialphabeta; /** stationary (alphabeta) frame current measurements */
    MCAF_U_CURRENT           i0;         /** zero-sequence current = (Ia + Ib + Ic)/3 */
    MCAF_U_CURRENT_DQ        idq;        /** rotating (dq) frame current measurements */

    /* Current controllers */
    MCAF_PISTATE_T      idCtrl;  /** controller state for the D axis */
    MCAF_PISTATE_T      iqCtrl;  /** controller state for the Q axis */
     
    /** Output limit for each axis of the current loops, normalized to DC link voltage,
     *  line-to-neutral, so that 0.57735 = 1/sqrt(3) = full line-to-line voltage */
    MCAF_U_NORMVOLTAGE_DQ    idqCtrlOutLimit; 
    
    MCAF_DYNAMIC_CURRENT_LIMIT dynLimit;  /** dynamic current limit */
    MCAF_U_CURRENT           iqCmdLimit;  /** maximum output current amplitude, q-axis */
    
    MCAF_STANDARD_INPUT_SIGNALS_T standardInputs;  /** standard input signals */
    MCAF_MOTOR_PARAMETERS_T       motorParameters; /** motor parameters */
    MCAF_BACKEMF_CALCULATION_T    backEMF;         /** quantities for estimated back-emf calculation */

    /* Current loop forward path */
    MCAF_U_VOLTAGE_DQ        vdqCmd;     /** desired dq-frame voltage, output of current loop */
    MCAF_U_VOLTAGE_DQ        vdq;        /** desired dq-frame voltage */
    MCAF_U_VOLTAGE_ALPHABETA valphabeta; /** desired alphabeta-frame voltage */
    MCAF_U_VOLTAGE_ALPHABETA valphabetaPerturbed; /** desired alphabeta-frame voltage, after perturbation */
    MCAF_U_VOLTAGE_ABC       vabc;       /** desired phase voltage */
    MCAF_U_RVOLTAGE          rVdc;       /** reciprocal of DC link voltage */
    MCAF_U_DUTYCYCLE_ABC     dabcRaw;    /** scaled duty cycle, per-unit, before dead-time compensation */
    MCAF_DEAD_TIME_COMPENSATION deadTimeCompensation; /** dead-time compensation state */
    MCAF_U_DUTYCYCLE_ABC     dabcUnshifted;  /** scaled duty cycle, per-unit, prior to ZSM and clipping */
    MCAF_U_DUTYCYCLE_ABC     dabc;       /** after ZSM + clip */
    MCAF_U_DUTYCYCLE_ALPHABETA dalphabetaOut[3]; /** convert dabc back to alpha-beta frame for estimators, with history */
    MCAF_U_VOLTAGE_ALPHABETA valphabetaOut; /** value of applied alpha-beta voltage, including deadtime compensation */
    MCAF_U_DUTYCYCLE_ABC pwmDutycycle;   /** PWM count */
    MCAF_FLUX_CONTROL_STATE_T fluxControl; /** flux-control state */
    MCAF_FILTER_LOW_PASS_S16_T vqFiltered; /** filtered q-axis voltage, for feedback purposes */
    MCAF_CURRENT_MEASUREMENT currentMeasure; /** current measure state */

    /* Angle and speed, including estimators */
    MCAF_U_ANGLE_ELEC       thetaElectrical;  /** electrical angle */
    MCAF_U_VELOCITY_ELEC    omegaElectrical;  /** electrical frequency */
    MCAF_U_DIMENSIONLESS_SINCOS  sincos;     /** sine and cosine of electrical angle */

    MCAF_ESTIMATOR_T estimator;  /** position and velocity estimator state */

    /* Velocity loop */
    MCAF_U_VELOCITY_ELEC    omegaCmd;   /** input command for the velocity loop */
   
    MCAF_PISTATE_T      omegaCtrl;  /** controller state for the velocity loop */
    MCAF_VELOCITY_CONTROL_DATA velocityControl; /** Control inputs for the velocity loop */
    MCAF_U_CURRENT    iqTorqueCmd;  /** output of the velocity loop */
    uint16_t          controlFlags; /** MCAF_CTRL_FLAGS bitfields */
    uint16_t          stateFlags;   /** MCAF_STATE_FLAGS bitfields */    
    HAL_ADC_SELECT_T  adcSelect;    /** which channel we are scanning */
    int16_t           potInput; /** potentiometer input */
    
    MCAF_BRIDGE_TEMPERATURE bridgeTemperature;  /** bridge temperature */
    
    /* open-loop to closed-loop transition */
    MCAF_MOTOR_STARTUP_DATA    startup;  /** State variables for the startup code */
    
    MCAF_FSM_STATE      state;           /** motor control state machine state */
    
    /** counter for subsampling (e.g. executing something every N counts */
    uint16_t subsampleCounter;            
    
    /** user interface data exchanged via main thread and ISR */
    volatile MCAF_UI_DATA ui;
#if MCAF_INCLUDE_STALL_DETECT      
    MCAF_STALL_DETECT_T stallDetect;     /** stall detect state */
#endif
    MCAF_FAULT_DETECT_T faultDetect;     /** fault detect state */
    MCAF_RECOVERY_DATA_T recovery; /** recovery status */
    MCAF_MONITOR_DATA_T monitor;   /** monitor data */

    /** test harness state, used by ISR, may be shared with main thread in future */
    volatile MCAF_MOTOR_TEST_MANAGER testing;
    
    MCAF_SYSTEM_DATA *psys;       /** pointer to shared system state */
    MCAF_SAT_DETECT_T sat;        /** saturation detection */
    MCAF_STOPPING_STATE stopping; /** Stopping timer state */

    /** current calibration parameters */
    MCAF_CURRENT_COMPENSATION_PARAMETERS currentCalibration;
        
    /** initialization */
    MCAF_MOTOR_INITIALIZATION initialization;  

    /** miscellaneous configurable parameters */
    struct tagConfig {
        /** number of ISR cycles to delay forward-path voltages
         *  to match the delay of the current feedback signals
         */
        uint16_t deadTimeCompensationVoltageDelay;  
    } config;
    
    /** MCAPI related shared data */
    volatile MCAPI_MOTOR_DATA apiData;
    /** MCAPI related feedback data in MCAF that is 
     * published to the application through MCAPI */
    MCAPI_FEEDBACK_SIGNALS apiFeedback;
    
    /** measured DC link voltage */
    MCAF_U_VOLTAGE vDC;
    
    /** measured DC link current*/
    MCAF_U_CURRENT iDC;
    /** measured absolute voltage reference */
    uint16_t vAbsRef;
#if MCAF_TRIGGERED_AVERAGE_EXAMPLE == 1
    MCAF_TRIGGERED_AVERAGE_T iqAverage;  /** Triggered average example implementation */
#endif
} MCAF_MOTOR_DATA;

/**
 * Increments a running count each time the motor is requested to stop.
 * @param pmotor motor state
 */
inline static void MCAF_IncrementStopCount(MCAF_MOTOR_DATA *pmotor)
{
    ++pmotor->psys->debugCounters.stop;
}

/**
 * Returns the upper current limit value
 * @param pmotor motor data
 * @return upper current limit value
 */
static inline int16_t MCAF_CurrentLimitIqUpperGet(volatile MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->omegaCtrl.outMax;
}

/**
 * Returns the lower current limit value
 * @param pmotor motor data
 * @return lower current limit value
 */
static inline int16_t MCAF_CurrentLimitIqLowerGet(volatile MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->omegaCtrl.outMin;
}

/**
 * Gets the measured value of DC link voltage
 * @param pMotor motor data
 * @return measured value of DC link voltage
 */
inline static int16_t MCAF_GetDcLinkVoltage(MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->vDC;
}

/**
 * Mark the motor state as being out of closed-loop commutation
 * @param pmotor motor data
 */
inline static void MCAF_ClearClosedLoopCommutation(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->stateFlags &= ~MSF_CLOSED_LOOP_COMMUTATION;
}

/**
 * Mark the motor state as being in closed-loop commutation
 * @param pmotor motor data
 */
inline static void MCAF_SetClosedLoopCommutation(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->stateFlags |= MSF_CLOSED_LOOP_COMMUTATION;
}

/**
 * Is the motor in closed-loop commutation?
 * @param pmotor motor data
 * @return whether the motor is in closed-loop commutation
 */
inline static bool MCAF_IsClosedLoopCommutation(const MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->stateFlags & MSF_CLOSED_LOOP_COMMUTATION;
}

/**
 * Mark the motor state as being out of closed-loop current control
 * @param pmotor motor data
 */
inline static void MCAF_ClearClosedLoopCurrent(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->stateFlags &= ~MSF_CLOSED_LOOP_CURRENT;
}

/**
 * Mark the motor state as being in closed-loop current control
 * @param pmotor motor data
 */
inline static void MCAF_SetClosedLoopCurrent(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->stateFlags |= MSF_CLOSED_LOOP_CURRENT;
}

/**
 * Is the motor in closed-loop current control?
 * @param pmotor motor data
 * @return whether the motor is in closed-loop current control
 */
inline static bool MCAF_IsClosedLoopCurrent(const MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->stateFlags & MSF_CLOSED_LOOP_CURRENT;
}

/**
 * Mark the motor state as being out of closed-loop velocity control
 * @param pmotor motor data
 */
inline static void MCAF_ClearClosedLoopVelocity(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->stateFlags &= ~MSF_CLOSED_LOOP_VELOCITY;
}

/**
 * Mark the motor state as being in closed-loop velocity control
 * @param pmotor motor data
 */
inline static void MCAF_SetClosedLoopVelocity(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->stateFlags |= MSF_CLOSED_LOOP_VELOCITY;
}

/**
 * Is the motor in closed-loop velocity control?
 * @param pmotor motor data
 * @return whether the motor is in closed-loop velocity control
 */
inline static bool MCAF_IsClosedLoopVelocity(const MCAF_MOTOR_DATA *pmotor)
{
    return pmotor->stateFlags & MSF_CLOSED_LOOP_VELOCITY;
}

/**
 * Clear all closed-loop flags
 * @param pmotor motor data
 */
inline static void MCAF_ClearClosedLoopFlags(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->stateFlags &= (~MSF_CLOSED_LOOP_COMMUTATION
                         & ~MSF_CLOSED_LOOP_CURRENT
                         & ~MSF_CLOSED_LOOP_VELOCITY);
}

/**
 * Initialize system state.
 * @param pmotor motor state data
 * @param psys system state data
 */
void MCAF_SystemStateInit(MCAF_MOTOR_DATA *pmotor, MCAF_SYSTEM_DATA *psys);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_STATE_H */
