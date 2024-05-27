/**
 * filter_types.h
 * 
 * Type definitions for first-order filters
 * 
 * Component: miscellaneous
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

#ifndef __FILTER_TYPES_H
#define __FILTER_TYPES_H

#include <stdint.h>
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * State variables for 16 bit implementation of low pass filter 
 * (NOTE: this is a legacy implementation using Q15 coefficients,
 *  not to mention that the naming has two "filter" components
 * LPF and Filter)
 */
typedef struct tagLpfFilterx16
{
    /** coefficient for LPF given by f3db*Ts*2*pi 
     * where f3db is cutoff frequency and Ts is sampling time */
    int16_t coeff;    
    int32_t stateVar; /** Stores previous state */
    int16_t output;   /** Output of LPF */
} MCAF_LPF_FILTER_X16_T;

/**
 * State variables for 16 bit implementation of low pass filter 
 */
typedef struct tagFilterLowPassS16
{
    sx1632_t state;    /** State of LPF (high-order bits are output) */
    /** coefficient for LPF given by f3db*Ts*2*pi 
     * where f3db is cutoff frequency and Ts is sampling time */
    uint16_t coeff;    
} MCAF_FILTER_LOW_PASS_S16_T;

/**
 * State variables for 16 bit implementation of high pass filter 
 */
typedef struct tagHpfFilterx16
{
    /** coefficient for HPF given by f3db*Ts*2*pi 
     * where f3db is cutoff frequency and Ts is sampling time */
    int16_t coeff;    
    int16_t output;
    int32_t stateVar; /** Stores previous state */
} MCAF_HPF_FILTER_X16_T;

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_TYPES_H */
