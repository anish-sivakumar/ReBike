/*
� [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/
#include "mcc_generated_files/motorBench/mcaf_main.h"
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/system/pins.h"

#include <stdbool.h>
#include "board_service.h"
#include "hal.h"
#include "system_state.h"
#include "state_machine.h"
#include "system_init.h"
#include "foc.h"
#include "diagnostics.h"
#include "monitor.h"
#include "stall_detect.h"
#include "recover.h"
#include "mcaf_watchdog.h"
#include "mcaf_traps.h"
#include "ui.h"
#include "parameters/init_params.h"
#include "mcaf_main.h"
#include "test_harness.h"
#include "fault_detect.h"
#include "mcapi.h"
#include "mcaf_sample_application.h"
#include "xc.h"
#include "current_measure.h"

#include "X2CScope.h"
#include "rb_library/rb_hall.h"
#include "rb_library/rb_control.h"
#include "rb_library/rb_pwm.h"

/* Global Variables */

/** Global instance of the main set of motor state variables */
RB_MOTOR_DATA PMSM;
/** Global instance of the main set of system state variables */
MCAF_SYSTEM_DATA sysData;


extern volatile MCAF_WATCHDOG_T watchdog;

volatile uint16_t ISR_testing;

bool MainInit(void);



/*
    Main application
*/
int main(void)
{
    MainInit();

    while(1)
    {
        
        
        /* State variables are fine to access w/o volatile qualifier for ISR
        * (since no interruptions)
        * but in main loop, the ISR may interrupt + we need to assume volatile.
        */
        //volatile RB_HALL_DATA *pHall = &hall;
        volatile RB_MOTOR_DATA *pPMSM = &PMSM;
        volatile MCAF_SYSTEM_DATA *pSysData = &sysData;
        volatile MCAF_WATCHDOG_T *pWatchdog = &watchdog;
        
        // Re-integrate these as needed
        //MCAF_UiStepMain(&pPMSM->ui);
        //MCAF_SystemStateMachine_StepMain(pPMSM);
        //MCAF_WatchdogManageMainLoop(pWatchdog);
        //MCAF_TestHarnessStepMain(&pSysData->testing);
        X2CScope_Communicate(); 
    }    
}


/**
 * Initialize hardware including ADC, PWM, and X2CScope
 * and Control Parameters
 * @return initialization success 
 */
bool MainInit (void)
{
    SYSTEM_Initialize();
    
    /* PWM Init from MCAF_ConfigurationPwmUpdate */
    RB_PWMInit();
    
      
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
    MCAF_DiagnosticsInit(); // UART and X2C scope
    MCAF_SystemStart(&sysData);
    HAL_TMR_TICK_Start();
    
    // Configure Hall ISRs and data
    RB_HALL_Init();
    
    
    
    // These might be needed. Need to modify functions
    MCAF_FaultDetectInit(&PMSM.faultDetect);
    RB_InitControlParameters(&PMSM);
    //MCAF_MotorControllerOnRestartInit(pmotor); !!!! IMPORTANT?
    MCAF_FaultDetectInit(&PMSM.faultDetect);

    
    MCC_TMR_PROFILE_Start(); // start timer 1
    
    bool success = RB_FocInit(&PMSM);
    if (success)
    {
        MCAF_SystemStart(&sysData);

        /* Ideally, MCAF has nothing to do with the application timer and hence this
           function should be called from the main() in main.c */
        HAL_TMR_TICK_Start();
    }
    return success;
}
