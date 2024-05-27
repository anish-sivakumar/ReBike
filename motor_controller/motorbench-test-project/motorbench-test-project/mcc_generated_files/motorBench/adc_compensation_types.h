/**
 * adc_compensation_types.h
 * 
 * Type definitions for ADC compensation
 * 
 * Component: ADC compensation
 *
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

#ifndef __ADC_COMPENSATION_TYPES_H
#define __ADC_COMPENSATION_TYPES_H

#include <stdbool.h>
#include <stdint.h>
#include "units.h"
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Compensation for current measurements.
 * 
 * These are applied as follows:
 * If Ia[0] and Ib[0] are the original ADC measurements, we calculate:
 * (note: sign is correct since our current sensing gain into the ADC is negative)
 * 
 * Ia[1] = -(Ia[0] - offseta)
 * Ib[1] = -(Ib[0] - offsetb)
 * 
 * Ia[final] = qKaa * Ia[1] + qKab * Ib[1]
 * Ib[final] = qKba * Ia[1] + qKbb * Ib[1]
 * 
 * The cross-terms are important to correct cross-coupling of the ADC measurements.
 */
typedef struct
{
    MCAF_U_NORMALIZED_GAIN   qKaa;        /** Q15 phase A gain */
    MCAF_U_NORMALIZED_GAIN   qKab;        /** Q15 cross-coupling gain B->A */
    int16_t                  offseta;     /** phase A offset */

    MCAF_U_NORMALIZED_GAIN   qKba;        /** Q15 cross-coupling gain A->B */
    MCAF_U_NORMALIZED_GAIN   qKbb;        /** Q15 phase B gain */
    int16_t                  offsetb;     /** phase B offset */

    MCAF_U_NORMALIZED_GAIN   qKcc;        /** Q15 phase C gain */
    int16_t                  offsetc;     /** phase C offset */
    
    MCAF_U_NORMALIZED_GAIN   qKidc;       /** Q15 DC link gain */
    int16_t                  offsetIdc;   /** DC link offset */
} MCAF_CURRENT_COMPENSATION_PARAMETERS;

/**
 * Motor initialization state variables
 * Used to calibrate current offset for two phases.
 */
typedef struct tagMCAF_MOTOR_INITIALIZATION
{
    int16_t  kfilter;          /** filter constant */
    sx1632_t offsetLPF[3];     /** low-pass filter (one for each phase) */
    uint16_t sampleCountLimit; /** number of samples used to calibrate */
    uint16_t sampleCount;      /** counter of samples acquired for calibration */
    uint16_t faultLatchDelay;  /** number of ISR cycles for fault latch delay */
    bool     ready;            /** whether initialization is ready */
} MCAF_MOTOR_INITIALIZATION;

#ifdef __cplusplus
}
#endif

#endif /* __ADC_COMPENSATION_TYPES_H */
