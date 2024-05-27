/**
 * stall_detect.h
 *
 * Module to detect when motor is stall
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
#include <stdbool.h>
#include "system_state.h"
#include "stall_detect_types.h"

#ifndef STALL_DETECT_H
#define STALL_DETECT_H

#ifdef __cplusplus
extern "C" {
#endif

void MCAF_StallDetectInit(MCAF_STALL_DETECT_T *pstallDetect);

void MCAF_StallDetectReset(MCAF_STALL_DETECT_T *pstallDetect);

/**
* Updates status of stall detect flag
*
* @param pstallDetect stall detect state
* @param pmotor input signals for various indicators for stall detection
*/
void MCAF_StallDetect(MCAF_STALL_DETECT_T *pstallDetect, const MCAF_MOTOR_DATA *pmotor);

/**
 * Returns whether stall detect is active
 * @param pstallDetect stall detect state
 * @return true if active
 */
inline static bool MCAF_StallDetectActive(const MCAF_STALL_DETECT_T *pstallDetect)
{
    return pstallDetect->active;
}

/**
 * Activates stall detection
 * @param pstallDetect stall detect state
 */
inline static void MCAF_StallDetectActivate(MCAF_STALL_DETECT_T *pstallDetect)
{
    pstallDetect->active = true;
}

/**
 * Returns stall detection flag
 * @param pstallDetect stall detect state
 */
inline static uint16_t MCAF_StallDetectGetMaskedFlags(const MCAF_STALL_DETECT_T *pstallDetect)
{
    return pstallDetect->stallDetectFlag & pstallDetect->stallDetectFlagMask;
}

/**
 * Clears stall detection flag
 * @param pstallDetect stall detect state
 */
inline static uint16_t MCAF_StallDetectClearMaskFlags(MCAF_STALL_DETECT_T *pstallDetect, uint16_t mask)
{
    pstallDetect->stallDetectFlag &= mask;
    return pstallDetect->stallDetectFlag;
}

/**
 * Deactivates stall detection
 * @param pstallDetect stall detect state
 */
inline static void MCAF_StallDetectDeactivate(MCAF_STALL_DETECT_T *pstallDetect)
{
    pstallDetect->active = false;
}

#ifdef __cplusplus
}
#endif

#endif /* STALL_DETECT_H */