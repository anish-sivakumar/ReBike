/**
 * system_init.h
 * 
 * System initialization tasks
 * 
 * Component: main application
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

#ifndef __SYSTEM_INIT_H
#define __SYSTEM_INIT_H

#include "system_state.h"
#include "parameters/hal_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize system state.
 * This function should be called once and only once at the beginning of
 * the main() code. One of the tasks of this function is to increment
 * a reset counter.
 * 
 * @param psys system state variables
 */
void MCAF_SystemInit(MCAF_SYSTEM_DATA *psys);

/**
 * Start system (including interrupts)
 * 
 * @param psys system state variables
 */
void MCAF_SystemStart(MCAF_SYSTEM_DATA *psys);

/**
 * Configures the PWM module as required.
 * This function is intended to be called after initializing the PWM module
 * and before enabling the PWM module.
 */
inline static void MCAF_ConfigurationPwmUpdate(void)
{
    HAL_PWM_SetPeriodIdentical(HAL_PARAM_PWM_PERIOD_COUNTS);
    HAL_PWM_SetDeadtimeIdentical_Motor1(HAL_PARAM_DEADTIME_COUNTS);
    HAL_PWM_SetADCTrigger();
    HAL_PWM_DutyCycle_SetIdentical(HAL_PARAM_MIN_DUTY_COUNTS);
    HAL_PWM_Outputs_Disable();
}

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_INIT_H */
