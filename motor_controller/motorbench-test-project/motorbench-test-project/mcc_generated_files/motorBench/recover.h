/**
 * recover.h
 *
 * Module to recover when motor is stall
 * 
 * Component: supervisory
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

#ifndef __RECOVER_H
#define __RECOVER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * State machine states for the recovery behavior
 */    
typedef enum tagMCAF_RECOVERY_FSM_STATE
{
    MCAF_RECOVERY_FSM_NO_ERROR = 0,           /** everything running fine */
    MCAF_RECOVERY_FSM_STOP_PENDING = 1,       /** stall detected, motor stopping */
    MCAF_RECOVERY_FSM_RESTART_PENDING = 2,    /** automatic retry pending a timer */
    MCAF_RECOVERY_FSM_RETRIES_EXCEEDED = 3    /** too many retries */
} MCAF_RECOVERY_FSM_STATE;

/**
 * State machine inputs for the recovery behavior
 */    
typedef enum tagMCAF_RECOVERY_FSM_INPUTS
{
    MCAF_RECOVERY_FSMI_STALL_DETECTED = 1,    /** stall detected */
    MCAF_RECOVERY_FSMI_STOP_COMPLETED = 2     /** stop completed */            
} MCAF_RECOVERY_FSM_INPUTS;

/**
 * State machine outputs for the recovery behavior
 */    
typedef enum tagMCAF_RECOVERY_FSM_OUTPUTS
{
    MCAF_RECOVERY_FSMO_STOP_MOTOR    = 1,     /** stop motor */
    MCAF_RECOVERY_FSMO_RETRY_FAULT   = 2      /** failure: too many retries */
} MCAF_RECOVERY_FSM_OUTPUTS;


/**
 * This structure hosts recovery variables
 */
typedef struct tagMCAF_RECOVERY_DATA
{
    int16_t countdownTrials;         /** number of unsuccessful recovery processes so far */
    uint16_t recoveryTimer;  /** timer counting elapsed time from recovery processing start */
    uint16_t time1stStaticRecovery;   /** time necessary for fault treatment */
    
    struct tagSTATEMACHINE {
        MCAF_RECOVERY_FSM_STATE state;  /** state machine state */
        uint16_t inputs;                /** state machine inputs */
        uint16_t outputs;               /** state machine outputs */
    } stateMachine;
} MCAF_RECOVERY_DATA_T;


/**
 * This function init the recovery module
 *
 * Summary : Initialize the recovery module
 *
 * @param precovery This parameter is pointer to MCAF_RECOVERY_DATA_T structure
 */
void MCAF_RecoveryInit(MCAF_RECOVERY_DATA_T *precovery);

/**
 * This function reset the recovery module
 *
 * Summary : Reset the recovery module
 *
 * @param precovery This parameter is pointer to MCAF_RECOVERY_DATA_T structure
 */
void MCAF_RecoveryReset(MCAF_RECOVERY_DATA_T *precovery);

/**
 * This function handles the recovery process, executing the recovery tasks, such
 * as coasting down the motor in the event of stall
 *
 * Summary : Runs the recovery task
 *
 * @param precovery This parameter is pointer to MCAF_RECOVERY_DATA_T structure
 */
void MCAF_Recovery(MCAF_RECOVERY_DATA_T *precovery);

/**
 * Sets an input flag
 * @param precovery state variable structure
 * @param flag which flag to set
 */
static inline void MCAF_RecoverySetInputFlag(MCAF_RECOVERY_DATA_T *precovery, MCAF_RECOVERY_FSM_INPUTS flag)
{
    precovery->stateMachine.inputs |= flag;
}

/**
 * Tests an output flag
 * 
 * @param precovery state variable structure
 * @param flag which flag to test
 * @return whether the flag was true
 */
static inline bool MCAF_RecoveryTestFlag(const MCAF_RECOVERY_DATA_T *precovery, MCAF_RECOVERY_FSM_OUTPUTS flag)
{
    return (precovery->stateMachine.outputs & flag) != 0;
}

/**
 * This function checks whether there was any failure reported in recovery
 *
 * Summary : Recovery ended with failure, is this true?
 *
 * @param pmotor This parameter is pointer to MCAF_RECOVERY_DATA_T structure
 * @return true if recovery failure is detected (number of recovery trials exceeded)
 */
static inline bool MCAF_RecoveryIsFailureDetected(const MCAF_RECOVERY_DATA_T *precovery)
{
    return MCAF_RecoveryTestFlag(precovery, MCAF_RECOVERY_FSMO_RETRY_FAULT);
}

#ifdef __cplusplus
}
#endif

#endif /* __RECOVER_H */
