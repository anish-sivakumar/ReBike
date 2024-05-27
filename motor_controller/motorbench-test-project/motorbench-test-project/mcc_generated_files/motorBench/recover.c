/**
 * recover.c
 *
 * Module to recover when motor is stall condition
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

#include "recover.h"
#include "hal.h"
#include "util.h"
#include "parameters/recover_params.h"


void MCAF_RecoveryInit(MCAF_RECOVERY_DATA_T *precovery)
{
    /* initialize the configuration structure */
    MCAF_RecoveryReset(precovery);
    precovery->time1stStaticRecovery = MCAF_RECOVERY_COASTDOWN_TIME;
}

void MCAF_RecoveryReset(MCAF_RECOVERY_DATA_T *precovery)
{
    /* initialize the configuration structure */
    precovery->countdownTrials =  MCAF_RECOVERY_STARTUP_ATTEMPTS;
    precovery->recoveryTimer = 0;
    precovery->stateMachine.state = MCAF_RECOVERY_FSM_NO_ERROR;
    precovery->stateMachine.inputs = 0;
    precovery->stateMachine.outputs = 0;
}

void MCAF_Recovery(MCAF_RECOVERY_DATA_T *precovery)
{           
    const MCAF_RECOVERY_FSM_STATE thisState = precovery->stateMachine.state;
    MCAF_RECOVERY_FSM_STATE nextState = thisState;
    switch (thisState)
    {
        case MCAF_RECOVERY_FSM_NO_ERROR:
            if (precovery->stateMachine.inputs & MCAF_RECOVERY_FSMI_STALL_DETECTED)
            {
                nextState = MCAF_RECOVERY_FSM_STOP_PENDING;
                precovery->stateMachine.outputs |= MCAF_RECOVERY_FSMO_STOP_MOTOR;
            }
            break;
        case MCAF_RECOVERY_FSM_STOP_PENDING:
            if (precovery->stateMachine.inputs & MCAF_RECOVERY_FSMI_STOP_COMPLETED)
            {
                if (precovery->countdownTrials == 0)
                {
                    nextState = MCAF_RECOVERY_FSM_RETRIES_EXCEEDED;
                    precovery->stateMachine.outputs |= MCAF_RECOVERY_FSMO_RETRY_FAULT;
                }
                else
                {
                    /** We can retry... start a timer */
                    --precovery->countdownTrials;
                    nextState = MCAF_RECOVERY_FSM_RESTART_PENDING;
                    precovery->recoveryTimer = 0;
                }
            }
            break;
        case MCAF_RECOVERY_FSM_RESTART_PENDING:
            if (++precovery->recoveryTimer >= precovery->time1stStaticRecovery)
            {
                /* Allow motor to restart */
                nextState = MCAF_RECOVERY_FSM_NO_ERROR;
                precovery->stateMachine.outputs &= ~MCAF_RECOVERY_FSMO_STOP_MOTOR;
            }
            break;
        case MCAF_RECOVERY_FSM_RETRIES_EXCEEDED:
            break;
        default:
            nextState = MCAF_RECOVERY_FSM_NO_ERROR;
            break;
    }
    
    /* clear inputs until next time and update state */
    precovery->stateMachine.inputs = 0;
    precovery->stateMachine.state = nextState;
}
