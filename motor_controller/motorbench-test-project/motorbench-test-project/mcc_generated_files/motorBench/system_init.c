/**
 * system_init.c
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

#include <stdint.h>
#include "system_state.h"
#include "system_init.h"
#include "diagnostics.h"
#include "hal.h"
#include "util.h"
#include "timing.h"
#include "parameters/options.h"

/**
 * Reset counter; incremented each time we have a reset and
 * a new system initialization.
 * 
 * The persistent attribute is necessary to prevent the compiler's
 * initialization code from overwriting it.
 * 
 * Unfortunately we can't put it into a larger structure;
 * the compiler applies the "persistent" attribute to the entire object,
 * so there's no way, for example, to mark one member of the main system
 * structure (MCAF_SYSTEM_DATA) as persistent. So it "lives" here,
 * and we copy it to the debugCounters.reset member of MCAF_SYSTEM_DATA.
 */
static uint16_t MCAF_resetCounter __attribute__ ((persistent, section("MCAF_persistent")));

void MCAF_SystemInit(MCAF_SYSTEM_DATA *psys)
{
    psys->debugCounters.reset = ++MCAF_resetCounter;
    
    MCAF_PIN_MANAGER_Initialize();
    HAL_Initialize();
    MCAF_ConfigurationPwmUpdate();
    if (MCAF_OpAmpsEnabled())
    {
        HAL_OpAmpsEnable();
        HAL_OpAmpsInputVoltageRangeSelect();
    }
    HAL_InterruptPrioritySet();
    HAL_CMP_SetComparatorOvercurrentThreshold(HAL_PARAM_DAC_OVERCURRENT_THRESHOLD);
    HAL_ADC_SignalsInit();
    HAL_ADC_ResolutionInit();
    HAL_ADC_Enable();
    HAL_UART_ReconfigureIoMapping();
    MCAF_DiagnosticsInit();
}

void MCAF_SystemStart(MCAF_SYSTEM_DATA *psys)
{
    /* Output a short pulse as a testpoint signal,
     * enable PWMs,
     * enable ADC interrupt,
     * begin main loop timing,
     * and start board timer */

    HAL_TestpointGp1_Activate();
    MCAF_DelayNanoseconds(500);
    HAL_TestpointGp1_Deactivate();
    HAL_PWM_ADCTrigger1AEnable();

    if (MCAF_SingleChannelEnabled())
    {
        HAL_PWM_ModeDualEdgeSingleUpdate();
        HAL_PWM_SelectLocalPhase();
        HAL_PWM_ADCTrigger2BEnable();
        HAL_PWM_ADCTrigger2CEnable();
    }
    else
    {
        HAL_PWM_SelectMasterPhase();
        
        if (MCAF_IsDoubleUpdatePwmAllowed())
        {
            HAL_PWM_ModeDoubleUpdate();
        }      
        else
        {
            HAL_PWM_ModeSingleUpdate();
        }
    }
    
    HAL_PWM_ModuleEnable();
    HAL_ADC_InterruptFlag_Clear();
    HAL_ADC_Interrupt_Enable();
}
