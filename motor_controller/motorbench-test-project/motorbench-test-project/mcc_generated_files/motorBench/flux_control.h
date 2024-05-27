/**
 * flux_control.h
 *
 * Flux control: none
 * 
 * Component: flux_control
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

#ifndef __FLUX_CONTROL_H
#define __FLUX_CONTROL_H

#include <stdint.h>
#include "units.h"
#include "flux_control_types.h"
#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize flux-control state
 * @param pstate flux-control state
 */        
inline static void MCAF_FluxControlInit(MCAF_FLUX_CONTROL_STATE_T *pstate) {}

/**
 * Reinitialize flux-control state on motor startup
 * @param pstate flux-control state
 */        
inline static void MCAF_FluxControlStartupInit(MCAF_FLUX_CONTROL_STATE_T *pstate) {}

/**
 * Execute one step of flux control update
 * 
 * @param pstate flux control state
 * @param pinput standard inputs
 * @param pmotor motor parameters
 * @return D-axis current command
 */
inline static MCAF_U_CURRENT MCAF_FluxControlStep(MCAF_FLUX_CONTROL_STATE_T *pstate,
                            const MCAF_STANDARD_INPUT_SIGNALS_T *pinput,
                            const MCAF_MOTOR_PARAMETERS_T *pmotor) {
    return 0;
}

/**
 * Get D-axis current command
 *
 * @param pstate flux control state
 * @return D-axis current command
 */
inline static MCAF_U_CURRENT MCAF_FluxControlGetIdCommand(MCAF_FLUX_CONTROL_STATE_T *pstate) {
    return 0;
}

/**
 * Get Q-axis current limit
 *
 * @param pstate flux control state
 * @param iDefaultLimit default current limit
 * @return Q-axis current limit
 */
inline static MCAF_U_CURRENT MCAF_FluxControlGetIqLimit(MCAF_FLUX_CONTROL_STATE_T *pstate, MCAF_U_CURRENT iDefaultLimit) {
    return iDefaultLimit;
}


#ifdef __cplusplus
}
#endif

#endif /* __FLUX_CONTROL_H */
