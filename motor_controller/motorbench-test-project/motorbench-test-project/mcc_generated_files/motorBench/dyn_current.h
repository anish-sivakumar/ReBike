/**
 * dyn_current.h
 *
 * Dynamic current limit: none (empty placeholder implementation)
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

#ifndef __DYN_CURRENT_H
#define __DYN_CURRENT_H

#include <stddef.h>
#include "dyn_current_types.h"
#include "units.h"
#include "parameters/foc_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize dynamic current limit
 * @param pdynlim dynamic current limit state
 */    
inline static void MCAF_DynamicCurrentLimitInit(MCAF_DYNAMIC_CURRENT_LIMIT *pdynlim)
{
    pdynlim->iLimit = MCAF_VELOCITY_CTRL_IQ_OUT_LIMIT;
}
/**
 * Get dynamic current limit
 * 
 * @param pdynlim dynamic current limit state
 * @return current limit
 */
inline static MCAF_U_CURRENT MCAF_DynamicCurrentLimitGet(MCAF_DYNAMIC_CURRENT_LIMIT *pdynlim)
{
    return pdynlim->iLimit;
}
/**
 * Update dynamic current limit
 * 
 * Outside critical-path code.
 * 
 * @param pdynlim dynamic current limit state
 * @param pIdq measured dq current
 */
inline static void MCAF_DynamicCurrentLimitUpdate(MCAF_DYNAMIC_CURRENT_LIMIT *pdynlim,
                                                  const MCAF_U_CURRENT_DQ *pIdq)
{
    // do nothing
}
#ifdef __cplusplus
}
#endif

#endif /* __DYN_CURRENT_H */
