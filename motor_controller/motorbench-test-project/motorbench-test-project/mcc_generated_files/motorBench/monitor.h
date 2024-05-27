/**
 * monitor.h
 *
 * Module to link recovery and stall detection
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

#ifndef __MONITOR_H
#define __MONITOR_H

#include <stdint.h>
#include <stdbool.h>

#include "system_state.h"
#include "monitor_types.h"
#include "recover.h"
#include "fault_detect.h"
#include "error_codes.h"
#include "mcapi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * System monitor fault types
 */
typedef enum tagMCAF_MONITOR_FAULT
{
    MCAF_MONITOR_RECOVERY_FAULT = 0x1,     /** recovery retrial exceed fault */
    MCAF_MONITOR_OVERVOLTAGE_FAULT = 0x2, /** overvoltage fault */
    MCAF_MONITOR_UNDERVOLTAGE_FAULT = 0x4, /** undervoltage fault */
    MCAF_MONITOR_OVERCURRENT_HW_FAULT = 0x8, /** overcurrent fault */
} MCAF_MONITOR_FAULT_FLAG;

/**
 * This function checks if regular run is accepted, since there is no fault to be acknowledged
 *
 * Summary : Check if running state can be achieved
 *
 * @param pmotor This parameter is pointer to MCAF_MOTOR_DATA structure
 * @return true if run is permitted
 */
static inline bool MCAF_MonitorIsRunPermitted(MCAF_MOTOR_DATA *pmotor)
{
    return !MCAF_RecoveryTestFlag(&pmotor->recovery, MCAF_RECOVERY_FSMO_STOP_MOTOR);
}

/**
 * This function checks if motor achieved stopped state, or is it still recovering
 *
 * Summary : Check if stopped state was achieved, after recovery
 *
 * @param pmotor This parameter is pointer to MCAF_MOTOR_DATA structure
 * @return true if the motor is stopped
 */
static inline bool MCAF_MonitorIsMotorStopped(MCAF_MOTOR_DATA *pmotor)
{
    /* Placeholder -- at this time, we don't actually detect 
     * whether the motor is stopped. */
    return true;
}

/**
 * This function inits the monitor function
 *
 * Summary : Initializes the monitoring function
 *
 * @param pmotor This parameter is pointer to MCAF_MONITOR_DATA_T structure
 */
void MCAF_MonitorInit(MCAF_MONITOR_DATA_T *pmonitor);

/**
 * This function represents the core of monitoring, containing the detection
 * routines for stall and fault - also it provides the glue logic in between
 * stall and fault detection and recovery function, managing the activation
 * registers for each specific post detection action.
 *
 * Summary : Assigns actions to recovery function depending on the stall or fault
 * detection functions output
 *
 * @param pmotor This parameter is pointer to MCAF_MOTOR_DATA structure
 */
void MCAF_MonitorSysDiagnose(MCAF_MOTOR_DATA *pmotor);

/**
 * This function gathers all the monitoring functionalities that have the highest
 * execution priority: recovery function is one of them.
 *
 * Summary : Monitoring system highest priority functions: Recovery
 *
 * @param pmotor This parameter is pointer to MCAF_MOTOR_DATA structure
 */
static inline void MCAF_MonitorStepIsr(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_Recovery(&pmotor->recovery);
}

/**
 * This function check whether there was any failure reported by recovery
 *
 * Summary : Does recovery reported a failure?
 *
 * @param pmotor This parameter is pointer to MCAF_MOTOR_DATA structure
 * @param faultType This parameter indicates the fault for which the Fault state is returned
 * @return true if a fault is detected
 */

bool MCAF_MonitorIsFaultDetected(MCAF_MOTOR_DATA *pmotor, MCAF_MONITOR_FAULT_FLAG faultType);

/**
 * This function acknowledges the faults that were treated by recovery, clearing
 * the activated recovery tasks, indicating to the system that run can be possible
 *
 * Summary : Acknowledge recovery tasks activated, enabling the system running state
 *
 * @param pmotor This parameter is pointer to MCAF_MOTOR_DATA structure
 */
void MCAF_MonitorRecoveryAcknowledged(MCAF_MOTOR_DATA *pmotor);

/**
 * Fault detection mode
 */
typedef enum tagFaultDetectionMode
{
    FDM_TEST    = 0,  /** test modes */
    FDM_PRIMARY = 1   /** "normal" (non-test) modes */
} MCAF_FaultDetectionMode;

/**
 * Translates fault flags into an error code.
 * 
 * This requires prioritization of errors, so that if more than one fault
 * flag is set, the error code results from the highest-priority error only,
 * and lower-priority errors are not visible, at least using the error code.
 * 
 * @param pfaultinfo fault detection state
 * @return appropriate error code (or ERR_NO_ERROR if no faults have been detected
 */
inline static MCAF_ERROR_CODE MCAF_MonitorTranslateFaultFlags(
        const MCAF_FAULT_DETECT_T *pfaultinfo)
{
   MCAF_ERROR_CODE result = ERR_NO_ERROR;
   if (MCAF_IsOvervoltageFlagSet(pfaultinfo))
   {
       result = ERR_DCLINK_OVERVOLTAGE;
   }
   else if (MCAF_IsUndervoltageFlagSet(pfaultinfo))
   {
       result = ERR_DCLINK_UNDERVOLTAGE;
   }
   else if (MCAF_IsOvercurrentFlagSet(pfaultinfo))
   {
       result = ERR_HW_OVERCURRENT;
   }
   else if (MCAF_IsOvertemperatureFlagSet(pfaultinfo))
   {
       result = ERR_OVERTEMPERATURE;
   }
   return result;
}

/**
 * Tests to see if a fault has occurred
 *
 * @param pmotor motor structure
 * @param mode test or primary mode
 * @return whether a fault has been detected
 */
inline static bool MCAF_DetectFault(MCAF_MOTOR_DATA *pmotor,
        MCAF_FaultDetectionMode mode)
{
    bool result = false;
    
    /* ----- First we check for errors that occur in any mode. ----- */
    if (MCAF_IsAnyFaultFlagSet(&pmotor->faultDetect))
    {
        result = true;
    }
    else if (mode != FDM_TEST)
    {
        /* ----- Then we check for errors that we only detect in the primary states. */
        if (MCAF_MonitorIsFaultDetected(pmotor, MCAF_MONITOR_RECOVERY_FAULT))
        {
            result = true;
        }
        else if (MCAPI_ApplicationFaultCodeGet(&pmotor->apiData) != ERR_NO_ERROR)
        {
            result = true;
        }
    }
    return result;
}

/**
 * Gets the error code present in any detected fault
 *
 * @param pmotor motor structure
 * @return error code
 */
inline static MCAF_ERROR_CODE MCAF_GetFaultCode(MCAF_MOTOR_DATA *pmotor)
{
    MCAF_ERROR_CODE code = MCAF_MonitorTranslateFaultFlags(&pmotor->faultDetect);
    if (code == ERR_NO_ERROR)
    {
        if (MCAF_MonitorIsFaultDetected(pmotor, MCAF_MONITOR_RECOVERY_FAULT))
        {
            code = ERR_STALL_RETRY_EXCEEDED;
        }
        else
        {
            code = MCAPI_ApplicationFaultCodeGet(&pmotor->apiData);
        }
    }
    return code;
}

#ifdef __cplusplus
}
#endif

#endif /* __MONITOR_H */
