/* 
 * File:   main
 * Author: Chris Hyggen, Anish Sivakumar
 *
 */


#include "motorBench/timing.h"
#include "hal/hardware_access_functions.h"
#include "motorbench/diagnostics.h"
#include "timer/delay.h"

#include "X2CScope.h"
#include "rb_library/rb_hall.h"
#include "rb_library/rb_control.h"
#include "rb_library/rb_pwm.h"
#include "rb_library/rb_mcp.h"



void RB_MainInit(void);
void RB_SystemStart(void);

// temporary checks for SPI comms
uint16_t canInitErrors;
uint16_t canTest1;
uint16_t canTest2;
CAN_FRAME testFrame;
uint8_t canTestArr[20];


/*
    Main application
*/
int main(void)
{
    RB_MainInit();

    while(1)
    {
        X2CScope_Communicate(); 
    }    
}


/**
 * Initialize hardware including ADC, PWM, and X2CScope
 * and Control Parameters
 * @return initialization success 
 */
void RB_MainInit (void)
{
    SYSTEM_Initialize();  
    
    // Error counter added for testing, can be removed later
    canInitErrors = RB_MCP_Init();
    

    /* PWM Init from MCAF_ConfigurationPwmUpdate */
    RB_PWMInit();
          
    if (MCAF_OpAmpsEnabled())
    {
        HAL_OpAmpsEnable();
    }
    HAL_InterruptPrioritySet();
    HAL_CMP_SetComparatorOvercurrentThreshold(HAL_PARAM_DAC_OVERCURRENT_THRESHOLD);
    HAL_ADC_SignalsInit();
    HAL_ADC_ResolutionInit();
    HAL_ADC_Enable();
    MCAF_DiagnosticsInit();
    RB_SystemStart();
    HAL_TMR_TICK_Start();
    
    MCC_TMR_PROFILE_Start(); // start timer 1
    
    // move to initialization state before ISR starts
    RB_ISR_StateInit();
    
}


/**
 * Starts system - copy of MCAF function
 * @return 
 */
void RB_SystemStart(void)
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
        HAL_PWM_ADCTrigger2BEnable(); // trigger2 for DC current
         
    }
    
    HAL_PWM_ModuleEnable();
    HAL_ADC_InterruptFlag_Clear();
    HAL_ADC_Interrupt_Enable();
}
