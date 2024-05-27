/**
 * startup_types.h
 * 
 * Type definitions for startup and open loop to closed loop transition 
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

#ifndef __STARTUP_TYPES_H
#define __STARTUP_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "units.h"
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif

  
/** Different state for startup and open-loop to close transitioning  */
typedef enum tagMCAF_STARTUP_FSM_STATE
{
   /** Start state (entered only on reset + exiting fault) */
   SSM_START            = 0,
   /** Rotor is locked during align state. 
    * Current ramp is applied during this state, angle held constant */
   SSM_CURRENT_RAMPUP   = 1, 
   /** Rotor is locked during align state. 
    * Current and angle held constant. */
   SSM_ALIGN            = 2, 
   /** Alignment acceleration of motor with forced commutation to overcome cogging torque. 
    * Motor is accelerated slowly from standstill to a certain alignment velocity */
   SSM_ACCEL0           = 3, 
   /** Initial acceleration of motor with forced commutation. 
    * Motor is accelerated from alignment velocity to minimum velocity */
   SSM_ACCEL1           = 4, 
   /** Motor is allowed to recover after initial acceleration during hold state. 
    * Speed and applied current torque command are constant during this state. */     
   SSM_HOLD             = 5,
   /** Current torque command is ramp down during this state. 
    * In this state, startup theta error (imposed theta - estimated theta) is also computed */  
   SSM_CURRENT_RAMPDOWN = 6, 
   /** When startup theta error falls below threshold, open to close loop transition takes place. 
    * Estimated theta is used for commutation after transition */
   SSM_TRANSITION       = 7,
   /** indicates completion of open to close loop transition */
   SSM_COMPLETE         = 8,
   SSM_INACTIVE         = 9  /** inactive state for test modes */
} MCAF_STARTUP_FSM_STATE;

typedef enum tagMCAF_STARTUP_STATUS_T
{
    /** startup in progress; something is going on */
    MSST_UNSPECIFIED = 0,
    /** startup restarting, awaiting further action */
    MSST_RESTART = 1,
    /** align: commutation angle and current are constant */
    MSST_ALIGN = 2,
    /** accel: electrical frequency increases to accelerate the motor */
    MSST_ACCEL = 3,
    /** spin: electrical frequency held constant to support measurements (example: QEI synchronization) */
    MSST_SPIN = 4,
    /** angle lock: electrical frequency may be changed by primary estimator */
    MSST_ANGLE_LOCK = 5,
    /** startup is complete; return to closed-loop operation */
    MSST_COMPLETE = 6
} MCAF_STARTUP_STATUS_T;

/**
 * Motor startup state variables for use during open-loop startup.
 * 
 * Some of these variables appear to duplicate the main state;
 * this is because during open-loop startup, the applied electrical angle
 * and the estimated electrical angle are different.
 * Variables stored here are related to the applied electrical angle.
 */
typedef struct tagMOTOR_STARTUP_DATA
{
     MCAF_U_CURRENT_DQ   idq;              /** dq-frame current */
     /** Slewrate limit for current. It is given by (max openloop Iq/rampup rate) */
     int16_t           iRampupLimit;   

     /** estimated electrical angle (from outside of open-loop operation) */
     MCAF_U_ANGLE_ELEC thetaElectricalEstimated;
     MCAF_U_VELOCITY_ELEC omegaElectricalEstimated;
     MCAF_U_DIMENSIONLESS_SINCOS sincos;           /** Sine and Cosine of electrical angle */
     
     /* ----- open-loop commutation ----- */
     sx1632_t          omegaElectrical;  /** open-loop electrical frequency */
     sx1632_t          thetaElectrical;  /** open-loop electrical angle */
     MCAF_U_ANGLE_ELEC rampupAngle;      /** angle applied during current rampup */
     MCAF_U_ANGLE_ELEC alignAngleDelta;  /** angle shift for align stage */
        /* ----- two-stage acceleration ----- */
     int16_t           acceleration[2];  /** open-loop accelerations  */
     MCAF_U_VELOCITY_ELEC velocityThreshold[2]; /** velocity threshold to complete acceleration */
     int16_t           dt;               /** timestep scaling factor for angle */
     int16_t           dtAcceleration;   /** timestep scaling factor for acceleration */     
     /* --------------------------------- */
     struct
     {
         MCAF_U_CURRENT iqmax;  /** maximum damping current */
         int16_t k;             /** damping constant */
         MCAF_U_VELOCITY_ELEC velocityThreshold; /** velocity threshold to enable */
     } activeDamping;    /** active damping */
     
     /**
      * Time after initial rampup of current at a fixed electrical angle,
      * and before open-loop acceleration during startup
      */
     uint32_t           alignTime;
     /**
      * Time after intial acceleration of velocity
      * and before the rampdown of current during startup
      */
     uint32_t           holdTime;
     /**
      * This value is used to transition from open loop to closed loop.
      * It is difference between forced angle and estimated angle. This difference is stored in
      * thetaError, and added to estimated theta after transition so the
      * effective angle used for commutating the motor is the same at
      * the end of open loop, and at the beginning of closed loop.
      */
     sx1632_t           thetaError;   
     /**
      * When theta error falls below this threshold, open to close loop transition happens 
      */
     MCAF_U_ANGLE_ELEC  thetaDelta;         
     /** Amplitude of applied current during startup */
     MCAF_U_CURRENT     iAmplitude;
     /** Nominal current applied during startup (sign depends on direction) */
     MCAF_U_CURRENT     iNominal;
     /** open-loop state machine state */
     MCAF_STARTUP_FSM_STATE state;
     /** Whether open-loop startup is enabled */
     bool               enable;
     /** Whether open-loop startup is complete */
     bool               complete;
     /** Request for delay */
     bool               delayRequest;
     /** Counter used in openloop for different states */
     uint32_t           counter;  
     /** 32-bit variable for torque command */
     int32_t            torqueCmd32;
     
} MCAF_MOTOR_STARTUP_DATA;

/**
 * Get implementation-independent status category
 * @param pstartup startup state
 * @return status category
 */
inline static MCAF_STARTUP_STATUS_T MCAF_StartupGetStatus(const MCAF_MOTOR_STARTUP_DATA *pstartup)
{
    MCAF_STARTUP_FSM_STATE state = pstartup->state;
    switch (state)
    {
        case SSM_START:      return MSST_RESTART;
        case SSM_ALIGN:      return MSST_ALIGN;
        case SSM_ACCEL0:     return MSST_ACCEL;
        case SSM_ACCEL1:     return MSST_ACCEL;
        case SSM_HOLD:       return MSST_SPIN;
        case SSM_TRANSITION: return MSST_COMPLETE;   // from the outside, once we get to this state, closed-loop is entered
        case SSM_COMPLETE:   return MSST_COMPLETE;
        default:             return MSST_UNSPECIFIED;
    }
}

/* Startup delay mechanism:
 * 
 * Estimators or other algorithms may request the startup routine 
 * to delay progress during certain stages, if the startup algorithm allows it.
 * 
 * MCAF_StartupRequestDelay   -- other algorithm requests a delay in startup
 * MCAF_StartupDelayPermitted -- startup specifies whether it can be delayed
 */

/**
 * Set a delay request flag to remain in certain states (ALIGN and SPIN)
 * NOTE: This is not a persistent flag; it needs to be re-asserted every control ISR
 * @param pstartup startup state
 */
inline static void MCAF_StartupRequestDelay(MCAF_MOTOR_STARTUP_DATA *pstartup)
{
    pstartup->delayRequest = true;
}

/**
 * Determines whether additional delays are allowed
 * 
 * @param pstartup startup state
 * @return whether additional delay requests are accepted
 */
inline static bool MCAF_StartupDelayPermitted(MCAF_MOTOR_STARTUP_DATA *pstartup)
{
    return false;
}

/**
 * Get angle difference from estimator
 * @param pstartup startup state
 */
inline static MCAF_U_ANGLE_ELEC MCAF_StartupGetThetaError(const MCAF_MOTOR_STARTUP_DATA *pstartup)
{
    return pstartup->thetaError.x16.hi;
}

/**
 * Set angle difference from estimator
 * (important for smooth state transitions)
 * 
 * @param pstartup startup state
 * @param thetaError desired angle difference
 */
inline static void MCAF_StartupSetThetaError(MCAF_MOTOR_STARTUP_DATA *pstartup, MCAF_U_ANGLE_ELEC thetaError)
{
    pstartup->thetaError.x16.hi = thetaError;
}

/**
 * Get angle of current vector command in dq-plane
 * used during forced commutation.
 *
 * Note that this is in the forced reference frame
 * (the reference frame defined by the forced electrical angle)
 * and not the actual electrical angle. If the current is sufficiently large,
 * so that maximum torque is much larger than the load torque, 
 * it will tend to pull the rotor's d-axis towards the current vector.
 *
 * A current vector angle of zero (along the d-axis) 
 * will tend to pull the rotor's d-axis toward the d-axis
 * of the forced reference frame.
 *
 * @param pstartup startup state
 * @return angle of current vector
 */
inline static MCAF_U_ANGLE_ELEC MCAF_StartupGetIdqCmdAngle(const MCAF_MOTOR_STARTUP_DATA *pstartup)
{
    /* 
     * If iNominal is positive, we're applying current along the positive Q axis = +90 degrees.
     * otherwise we're applying current along the negative Q axis = -90 degrees.
     */
    
    const MCAF_U_ANGLE_ELEC ninety_degrees = 0x4000;
    return UTIL_CopySign(pstartup->iNominal, ninety_degrees);
}

#ifdef __cplusplus
}
#endif

#endif /* __STARTUP_TYPES_H */