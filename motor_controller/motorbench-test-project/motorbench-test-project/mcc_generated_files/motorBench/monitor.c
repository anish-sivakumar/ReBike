/**
 * monitor.c
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

#include <stdint.h>
#include "recover.h"
#include "monitor.h"
#include "stall_detect.h"
#include "fault_detect.h"
#include "system_state.h"
#include "util.h"
#include "test_harness.h"

void MCAF_MonitorSysDiagnose(MCAF_MOTOR_DATA *pmotor)
{
    /* fault detection */
    MCAF_FaultDetect(&pmotor->faultDetect, pmotor);

#if MCAF_INCLUDE_STALL_DETECT    
    if (MCAF_StallDetectActive(&pmotor->stallDetect))
    {
        /* check for stall conditions */
        MCAF_StallDetect(&pmotor->stallDetect, pmotor);

        /* ignore a stall condition if we override detection; otherwise handle it */
        if (!MCAF_OverrideStallDetection(&pmotor->testing))
        {
            /* Have we detected a stall by one of the methods that is enabled? */
            uint16_t maskedFlags = MCAF_StallDetectGetMaskedFlags(&pmotor->stallDetect);
            if (maskedFlags != 0)
            {
                MCAF_RecoverySetInputFlag(&pmotor->recovery, MCAF_RECOVERY_FSMI_STALL_DETECTED);
            }
        }
    }
#endif
}

void MCAF_MonitorInit(MCAF_MONITOR_DATA_T *pmonitor)
{
    /* initialize run as true */
    pmonitor->runPermitted = true;
}

void MCAF_MonitorRecoveryAcknowledged(MCAF_MOTOR_DATA *pmotor)
{
    /* once acknowledged, run is permitted */
    pmotor->monitor.runPermitted = true;
    
}

bool MCAF_MonitorIsFaultDetected(MCAF_MOTOR_DATA *pmotor, MCAF_MONITOR_FAULT_FLAG faultType)
{
    bool failure = false;
    if (faultType == MCAF_MONITOR_RECOVERY_FAULT)
    {
        failure = MCAF_RecoveryIsFailureDetected(&pmotor->recovery);
    }
    else
    {
        failure = MCAF_RecoveryIsFailureDetected(&pmotor->recovery)
        || MCAF_Fault_IsFaultDetected(&pmotor->faultDetect);
    }
    
    return failure;
}
