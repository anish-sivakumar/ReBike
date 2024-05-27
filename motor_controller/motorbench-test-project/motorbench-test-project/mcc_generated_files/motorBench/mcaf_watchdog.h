/**
 * mcaf_watchdog.h
 * 
 * Functions for handling watchdog timer.
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

#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include <stdint.h>
#include "hal.h"

#ifdef  __cplusplus
extern "C" {
#endif

enum 
{
    /** Threshold for main-loop timeout, in terms of ISR cycles. */
    MCAF_WATCHDOG_MAINLOOP_TIMEOUT = 10000
};
    
/**
 * Watchdog state
 */
typedef struct tagWatchdog
{
    uint16_t isrCount; /** count of ISRs */
} MCAF_WATCHDOG_T;

/**
 * Manages the watchdog in the main loop.
 * 
 * @param pwatchdog watchdog state
 */
inline static void MCAF_WatchdogManageMainLoop(volatile MCAF_WATCHDOG_T *pwatchdog)
{
    pwatchdog->isrCount = 0;
}

/**
 * Care for the watchdog.
 */
inline static void MCAF_CareForWatchdog(void)
{
    HAL_WATCHDOG_Timer_Clear();
}

/**
 * Manages the watchdog in the ISR.
 * 
 * @param pwatchdog watchdog state
 */
inline static void MCAF_WatchdogManageIsr(volatile MCAF_WATCHDOG_T *pwatchdog)
{
    if (pwatchdog->isrCount < MCAF_WATCHDOG_MAINLOOP_TIMEOUT)
    {
        ++pwatchdog->isrCount;
        MCAF_CareForWatchdog();
    }
}

#ifdef  __cplusplus
}
#endif

#endif  /* __WATCHDOG_H */

