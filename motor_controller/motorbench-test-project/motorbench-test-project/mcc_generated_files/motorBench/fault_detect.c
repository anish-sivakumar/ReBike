/**
 * fault_detect.c
 *
 * Module to detect when motor is in fault condition
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
#include "fault_detect.h"
#include "parameters/fault_detect_params.h"
#include "util.h"
#include "foc.h"

void MCAF_DCLinkVoltageFaultDetectInit(MCAF_DCLINKVOLTAGE_DETECT_T *pOvervoltageDetect)
{
    pOvervoltageDetect->vDCOvervoltageThreshold = VDC_OVERVOLTAGE_THRESHOLD;
    pOvervoltageDetect->vDCUndervoltageThreshold = VDC_UNDERVOLTAGE_THRESHOLD;
    
}

void MCAF_FaultDetectInit(MCAF_FAULT_DETECT_T *pFaultDetect)
{
    MCAF_DCLinkVoltageFaultDetectInit(&pFaultDetect->voltage);
    pFaultDetect->overtemperatureThreshold = MCAF_OVERTEMPERATURE_THRESHOLD;
    MCAF_FaultDetectReset(pFaultDetect);
}

void MCAF_FaultDetectReset(MCAF_FAULT_DETECT_T *pFaultDetect)
{
    pFaultDetect->faultDetectFlag = 0;
}

void MCAF_FaultDetect(MCAF_FAULT_DETECT_T *pfaultDetect, const MCAF_MOTOR_DATA *pmotor)
{
    uint16_t flags = 0;
    
    if (MCAF_ADCInitializationReady(pmotor))
    {
        if (MCAF_OvervoltageDetect(&pfaultDetect->voltage, pmotor))
        {
            flags = UTIL_SetBits(flags, MCAF_OVERVOLTAGE_FAULT_DETECT);
        }
        if (MCAF_UndervoltageDetect(&pfaultDetect->voltage, pmotor))
        {
            flags = UTIL_SetBits(flags, MCAF_UNDERVOLTAGE_FAULT_DETECT);
        }        
        if (MCAF_OvertemperatureDetect(pfaultDetect->overtemperatureThreshold, pmotor))
        {
            flags = UTIL_SetBits(flags, MCAF_OVERTEMPERATURE_FAULT_DETECT);
        }
    }
    if (MCAF_OvercurrentHWFlagValid(pmotor) && MCAF_OvercurrentHWDetect())
    {
        flags = UTIL_SetBits(flags, MCAF_OVERCURRENT_HW_FAULT_DETECT);
    }

    pfaultDetect->faultDetectFlag = flags;
}
