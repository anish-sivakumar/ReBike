/**
 * sat_PI_types.h
 * 
 * Module holds typedef used in sat_PI.h 
 * 
 * Component: FOC
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

#ifndef SAT_PI_TYPES_H
#define SAT_PI_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Bit flags to indicate status of current/voltage saturation
 */
typedef enum
{
    MCAF_OUT_OF_SATURATION = 0x01, /** Out of saturation State */
    MCAF_INTO_SATURATION = 0x02,   /** Enter saturation state */
} MCAF_SAT_FLAGS;

/**
 * State variables related to current/voltage saturation
 */ 
typedef struct tagSat
{
    int16_t thresholdDownLimit;  /** Threshold to come out of saturation state */
    int16_t thresholdUpLimit;    /** Threshold to enter saturation state  */
    uint16_t satFlag;            /** Flags to indicate saturation state */
} MCAF_SAT_T;

/**
 * Status of saturation
 */
typedef enum tagSatState
{
    MCAF_SAT_NONE = 0,             /** No saturation indicator */
    MCAF_SAT_VOLT = 1,             /** Voltage saturation state indicator */
    MCAF_SAT_CURRENT = 2,           /** Current saturation state indicator */
    MCAF_SAT_VOLT_AND_CURRENT = 3  /** Voltage and Current saturation state indicator */
} MCAF_SAT_STATE_T;

/**
 * State variables related to saturation detect
 */
typedef struct tagsatdetect
{
    MCAF_SAT_STATE_T state; /** Existing saturation state */
    MCAF_SAT_T voltSat;     /** Parameters related to voltage saturation */
    MCAF_SAT_T currentSat;  /** Parameters related to current saturation */
} MCAF_SAT_DETECT_T;

/**
 * State variables related to PI controller 
 */
typedef struct
{
    int32_t integrator; /** Integrator sum */
    int16_t kp;         /** Proportional gain coefficient term */
    int16_t ki;         /** Integral gain coefficient term */
    int16_t kc;         /** Antiwindup gain coefficient term */
    int16_t nkp;        /** Normalizing term for proportional coefficient */
    int16_t nki;        /** Normalizing term for integral coefficient */
    int16_t outMax;     /** Maximum output limit */
    int16_t outMin;     /** Minimum output limit */
} MCAF_PISTATE_T;

#ifdef __cplusplus
}
#endif

#endif /* SAT_PI_TYPES_H */
