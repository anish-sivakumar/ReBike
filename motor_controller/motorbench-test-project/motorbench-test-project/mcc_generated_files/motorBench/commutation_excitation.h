/**
 * commutation_excitation.h
 * 
 * Inline definitions of excitation functions for commutation
 *
 * Component: commutation
 */ /*
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
 *
 ******************************************************************************/

#ifndef __COMMUTATION_EXCITATION_H
#define __COMMUTATION_EXCITATION_H

#include <stdint.h>
#include "units.h"
#include "commutation/common.h"
#include "commutation/pll.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Gets alpha-axis excitation voltage
 *
 * @param motor state
 * @return alpha-axis voltage
 */
inline static MCAF_U_VOLTAGE MCAF_CommutationExcitationValpha(const MCAF_MOTOR_DATA *pmotor)
{
   return 0;
}

/**
 * Gets beta-axis excitation voltage
 *
 * @param motor state
 * @return beta-axis voltage
 */
inline static MCAF_U_VOLTAGE MCAF_CommutationExcitationVbeta(const MCAF_MOTOR_DATA *pmotor)
{
   return 0;
}

/**
 * Gets d-axis excitation voltage
 *
 * @param motor state
 * @return d-axis voltage
 */
inline static MCAF_U_VOLTAGE MCAF_CommutationExcitationVd(const MCAF_MOTOR_DATA *pmotor)
{
   return 0;
}

/**
 * Gets q-axis excitation voltage
 *
 * @param motor state
 * @return q-axis voltage
 */
inline static MCAF_U_VOLTAGE MCAF_CommutationExcitationVq(const MCAF_MOTOR_DATA *pmotor)
{
   return 0;
}

#ifdef __cplusplus
}
#endif

#endif  /* __COMMUTATION_EXCITATION_H */
