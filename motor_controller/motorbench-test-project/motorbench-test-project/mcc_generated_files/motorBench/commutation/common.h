/**
 * commutation/common.h
 *
 * Common commutation elements
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

#ifndef __COMMUTATION_COMMON_H
#define __COMMUTATION_COMMON_H

#include <stdint.h>
#include "units.h"
#include "startup_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Estimator outputs
 */
typedef struct tagMCAF_ESTIMATOR_OUTPUTS
{
    MCAF_U_VELOCITY_ELEC omegaElectrical; /** Estimated velocity */
    MCAF_U_ANGLE_ELEC thetaElectrical;    /** Estimated electrical angle */    
} MCAF_ESTIMATOR_OUTPUTS_T;

/**
 * Function pointer type for updating estimator output state
 * 
 * @param obj opaque pointer to object state
 * @param new_output new values of output state
 */
typedef void (*MCAF_ESTIMATOR_OUTPUT_UPDATE_FUNCTION)(void *obj, const MCAF_ESTIMATOR_OUTPUTS_T *new_output);

/**
 * General estimator-related actions that can be returned to influence startup behavior
 */
typedef enum {
    MESACT_NOP           = 0,  /** no action */
    MESACT_COMPLETE      = 1,  /** complete! */
    MESACT_DELAY         = 2,  /** delay further progress */
    MESACT_SET_CURRENT   = 4,  /** set current */
    MESACT_SET_ANGLE     = 8,  /** set commutation angle */
    MESACT_SET_FREQUENCY = 16  /** set commutation frequency */
} MCAF_ESTIMATOR_STARTUP_ACTION_FLAG_T;

/** one or more MESACT flags OR'd together */
typedef uint16_t MCAF_ESTIMATOR_STARTUP_ACTION_T; 

#ifdef __cplusplus
}
#endif

#endif /* __COMMUTATION_COMMON_H */
