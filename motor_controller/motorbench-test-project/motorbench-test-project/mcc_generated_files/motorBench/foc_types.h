/**
 * foc_types.h
 * 
 * Types for FOC code
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

#ifndef __FOC_TYPES_H
#define __FOC_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "units.h"
#include "util.h"
#include "startup_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Control flags
 */
typedef enum tagMCAF_CTRL_FLAGS
{
    MCF_VOLTAGE_COMPENSATION = 0x01 /** voltage compensation enabled */
} MCAF_CTRL_FLAGS;

/**
 * State flags
 */
typedef enum tagMCAF_STATE_FLAGS
{
    MSF_CLEAN_STARTUP_SIGNAL    = 0x01, /** Startup signal for coherent reinitialization. */
    MSF_CLOSED_LOOP_COMMUTATION = 0x02, /** Is the system in closed-loop commutation? */
    MSF_CLOSED_LOOP_CURRENT     = 0x04, /** Is the system in closed-loop current control? */
    MSF_CLOSED_LOOP_VELOCITY    = 0x08  /** Is the system in closed-loop velocity control? */
} MCAF_STATE_FLAGS;

/**
 * Velocity control data
 */
typedef struct tagMCAF_VELOCITY_CONTROL_DATA
{
    MCAF_U_VELOCITY_MECH velocityCmdApi;         /** Reference velocity received from MCAPI */
    MCAF_U_VELOCITY_MECH velocityCmd;            /** Reference velocity */
    MCAF_U_VELOCITY_MECH velocityCmdRateLimited; /** Reference velocity after rate limiting */
    int16_t velocityCmdGain;      /** Reference velocity gain */
    int16_t slewRateLimit1;       /** rate limit relative to sensed velocity */
    int16_t slewRateLimitAccel;   /** acceleration slew rate limit */
    int16_t slewRateLimitDecel;   /** deceleration slew rate limit */
} MCAF_VELOCITY_CONTROL_DATA;

/**
 * Standard input signals
 * 
 * These are part of the public interface between the FOC module and various 
 * other modules (such as estimators and flux control).
 * 
 * Data within this structure is intended to be prepared (copied) by the FOC module 
 * and available for read-only use by other modules that run within FOC.
 */
typedef struct tagInputSignals
{
    MCAF_U_CURRENT_ALPHABETA ialphabeta;           /** stationary (alphabeta) frame current measurements */
    MCAF_U_VOLTAGE_ALPHABETA valphabeta;           /** value of applied alpha-beta voltage, including deadtime compensation */
    MCAF_U_VOLTAGE_ALPHABETA_Q14 ealphabeta;       /** estimate of back-emf, compensated for I*(R + d(LI)/dt) */
    
    MCAF_U_VOLTAGE_DQ        vdq;                  /** desired dq-frame voltage prior to forward-path transforms */
    MCAF_U_CURRENT_DQ        idq;                  /** dq-frame current */
    
    MCAF_U_VOLTAGE           vDC;                  /** measured DC link voltage */
    
    MCAF_U_VELOCITY_ELEC omegaElectricalEstimated; /** estimated velocity */
    MCAF_U_VELOCITY_ELEC omegaElectricalCommand;   /** velocity command */
    MCAF_STARTUP_STATUS_T    startupStatus;        /** startup status */
    
    /** angle of applying current in the dq plane, when in open-loop forced-commutation startup;
     *  0 degrees = along positive d-axis; 90 degrees = along positive q-axis
     */
    MCAF_U_ANGLE_ELEC        thetaForcedCommutation;

    MCAF_U_CURRENT           iqCmd;                /** Q-axis current command */
    MCAF_U_CURRENT           iCmdLimit;            /** maximum current amplitude */
    
    /** reciprocal of maximum current amplitude, valid for iCmdLimit of at least 2048 counts */
    int16_t                  rICmdLimit;
    
    MCAF_STATE_FLAGS         stateFlags;           /** state flags */
    
    /* ----------------------------------------------------------------
     * the following items are quirky and questionable,
     * and not guaranteed to be present in future MCAF versions.
     * ---------------------------------------------------------------- */
    int16_t direction;                     /** motor direction: +1 for zero or positive velocity, -1 otherwise */
} MCAF_STANDARD_INPUT_SIGNALS_T;


/**
 * Motor parameters
 * 
 * Content of this structure is part of the public interface between the FOC module
 * and other modules that require read-only access to motor parameters at runtime.
 */
typedef struct tagMotorParameters
{
    MCAF_U_STATOR_RESISTANCE rs;                 /** Stator resistance */
    
    MCAF_U_STATOR_INDUCTANCE l0BaseDt;           /** Common-mode stator inductance */
    MCAF_U_STATOR_INDUCTANCE l1BaseDt;           /** Differential-mode stator inductance */    
    MCAF_U_STATOR_INDUCTANCE ldBaseDt;           /** D-axis stator inductance */    
    MCAF_U_STATOR_INDUCTANCE lqBaseDt;           /** Q-axis stator inductance */
    
    MCAF_U_STATOR_INDUCTANCE l0BaseOmegaE;       /** Common-mode stator inductance */
    MCAF_U_STATOR_INDUCTANCE l1BaseOmegaE;       /** Differential-mode stator inductance */    
    MCAF_U_STATOR_INDUCTANCE ldBaseOmegaE;       /** D-axis stator inductance */    
    MCAF_U_STATOR_INDUCTANCE lqBaseOmegaE;       /** Q-axis stator inductance */
    
    MCAF_U_BACKEMF           ke;                 /** Back-EMF constant */
    MCAF_U_BACKEMF_INVERSE   keInverse;          /** Inverse of back-EMF constant */
} MCAF_MOTOR_PARAMETERS_T;

/**
 * Back-emf calculation state variables
 * 
 * Members of this structure are implementation details of the back-emf estimate
 * made available to position/velocity estimators and other algorithms.
 * They are not part of the public interface to the FOC module, and are subject
 * to change.
 * 
 * Algorithms requiring the back-emf estimate should be using ealphabeta
 * within MCAF_STANDARD_INPUT_SIGNALS_T.
 */
typedef struct tagBackEMFCalculation
{
    MCAF_U_VOLTAGE_ALPHABETA_Q14 lastValphabeta;     /** Value of Valphabeta from previous control step */
    MCAF_U_VOLTAGE_ALPHABETA_Q14 lastFluxalphabeta;  /** Value of Fluxalphabeta from previous control step */
    
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
    int16_t l1Sin2Theta;        /** L1 * Sin(2*Theta) */
    int16_t l1Cos2Theta;        /** L1 * Cos(2*Theta) */
    int16_t laa;                /** Lalpha-alpha */
    int16_t lab;                /** Lalpha-beta */
    int16_t lbb;                /** Lbeta-beta */
    int16_t lba;                /** Lbeta-alpha */
    MCAF_U_DIMENSIONLESS_SINCOS sincos2Theta;    /** Sine and cosine component of 2*theta */
    MCAF_U_VOLTAGE_ALPHABETA_Q14 vInductance;        /** Calculated value of voltage across stator inductance */
    MCAF_U_VOLTAGE_ALPHABETA_Q14 irDrop;             /** Calculated value of voltage across stator resistance */

} MCAF_BACKEMF_CALCULATION_T;

#ifdef __cplusplus
}
#endif

#endif /* __FOC_TYPES_H */
