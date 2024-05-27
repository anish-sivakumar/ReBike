/**
 * commutation.h
 *
 * Module to include commutation functions
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

#ifndef __COMMUTATION_H
#define __COMMUTATION_H

#include "system_state.h"
#include "startup.h"
#include "parameters/operating_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * This function initializes commutation
 *
 * Summary : Commutation initialization
 *
 * @param pmotor This parameter is pointer to MCAF_MOTOR_DATA structure
 */
 void MCAF_CommutationInit(MCAF_MOTOR_DATA *pmotor);

/**
 * This function initializes the commutation upon beginning open-loop startup
 *
 * Summary : Commutation restart
 *
 * @param pmotor This parameter is pointer to MCAF_MOTOR_DATA structure
 */
 void MCAF_CommutationStartupInit(MCAF_MOTOR_DATA *pmotor);

/**
 * This function calculate commutation angle each processing step
 *
 * Summary : Commutation angle calculator
 *
 * @param pmotor This parameter is pointer to MCAF_MOTOR_DATA structure
 */
 void MCAF_CommutationStep(MCAF_MOTOR_DATA *pmotor);

 /**
  * Adjusts controller state on transition from open-loop commutation
  * to closed-loop commutation.
  *
  * @param pmotor motor controller state
  */
 inline static void MCAF_CommutationTransitionToClosedLoop(MCAF_MOTOR_DATA *pmotor)
 {
    /*
     * The first time the velocity controller is executed, we want the output
     * to be continuous with the existing current command.
     */
    pmotor->omegaCtrl.integrator = (int32_t) pmotor->idqCmdRaw.q << 16;
    pmotor->startup.counter = 0;
                
    /* 
     * Set flags to notify other parts of MCAF that the velocity loop and commutation
     * are both in closed loop.
     */
    MCAF_SetClosedLoopCommutation(pmotor);
    MCAF_SetClosedLoopVelocity(pmotor);
}

 /**
 * Reinitializes startup information, when first entering motor startup state.
 * Used both in normal mode (going from STOP to STARTING)
 * and in exceptional conditions (exiting error or debug state)
 *
 * @param pmotor motor controller state
 */
inline static void MCAF_CommutationStartupReinit(MCAF_MOTOR_DATA *pmotor)
{
    const int16_t motorDirection = UTIL_SignFromHighBit(pmotor->velocityControl.velocityCmd);
    MCAF_StartupReinit(&pmotor->startup, motorDirection);
    pmotor->standardInputs.startupStatus = MCAF_StartupGetStatus(&pmotor->startup);
}
 
/**
 * Reinitializes commutation state information at motor restart.
 * (upon exiting error or debug state)
 *
 * @param pmotor motor controller state
 */
inline static void MCAF_CommutationRestart(MCAF_MOTOR_DATA *pmotor)
{
    pmotor->velocityControl.slewRateLimit1     = VELOCITY_SLEWRATE_LIMIT1;
    pmotor->velocityControl.slewRateLimitAccel = VELOCITY_SLEWRATE_LIMIT_ACCEL;
    pmotor->velocityControl.slewRateLimitDecel = VELOCITY_SLEWRATE_LIMIT_DECEL;
    MCAF_CommutationStartupReinit(pmotor);
}
 
/**
 * Copy commutation information from estimator to stall detect inputs
 * @param pmotor motor controller state
 */
void MCAF_CommutationPrepareStallDetectInputs(MCAF_MOTOR_DATA *pmotor);

#ifdef __cplusplus
}
#endif

#endif  /* __COMMUTATION_H */
