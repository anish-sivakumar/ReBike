/**
 * test_harness_timestamps.h
 * 
 * Test harness timestamp definitions
 * 
 * Component: test harness
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

#ifndef __TEST_HARNESS_TIMESTAMPS_H
#define __TEST_HARNESS_TIMESTAMPS_H

/* 
 * Note: this is not a normal #include file; it is context-sensitive and
 * should be included within the context of a MCTH_TIMESTAMP macro:
 * 
 *    #define MCTH_TIMESTAMP(NAME,VAL)   // something here
 *    #include "test_harness_timestamps.h"
 *    #undef MCTH_TIMESTAMP
 * 
 * This file should also contain ONLY comments and usages of MCTH_TIMESTAMP.
 * 
 * The semantics of MCTH_TIMESTAMP is to declare a timestamp name and value.
 * Timestamp values of 0 to MCAF_PROFILING_TIMESTAMP_CAPACITY-1 are valid;
 * timestamp values of -1 are deactivated.
 */

MCTH_TIMESTAMP(STATEMACH_START,            0)
MCTH_TIMESTAMP(STATEMACH_ON_ALL_STATES,    1)
MCTH_TIMESTAMP(STATEMACH_NEXT_STATE,       2)
MCTH_TIMESTAMP(STATEMACH_DISPATCH,         3)
MCTH_TIMESTAMP(STATEMACH_END,              4)
MCTH_TIMESTAMP(DIAGNOSTICS,                6)
MCTH_TIMESTAMP(END_OF_ISR,                 7)
        
MCTH_TIMESTAMP(FLUX_CONTROL_START,        -1)
MCTH_TIMESTAMP(FLUX_CONTROL_END,          -1)

#endif /* __TEST_HARNESS_TIMESTAMPS_H */
