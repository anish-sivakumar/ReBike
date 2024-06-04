/*
© [2024] Microchip Technology Inc. and its subsidiaries.

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

#include "mc_library/mc_hall.h"

/* Global Variables */

/** Global instance of the main set of motor state variables */
MCAF_MOTOR_DATA PMSM;
/** Global instance of the main set of system state variables */
MCAF_SYSTEM_DATA sysData;
/** Global instance of the hall sensor variables */
MC_HALL_DATA hall;

extern volatile MCAF_WATCHDOG_T watchdog;

bool MainInit(void);


/*
    Main application
*/
int main(void)
{
    SYSTEM_Initialize();
    MainInit();
    
    
    // Configure Hall ISRs and data
    //volatile int16_t thetaElectrical = 0;
    MC_HALL_Init(&hall);
    IO_RE8_SetInterruptHandler(&MC_HALL_ISR);
    IO_RE9_SetInterruptHandler(&MC_HALL_ISR);
    IO_RE10_SetInterruptHandler(&MC_HALL_ISR);

    while(1)
    {
        //MCAF_MainLoop();
        
        /* State variables are fine to access w/o volatile qualifier for ISR
        * (since no interruptions)
        * but in main loop, the ISR may interrupt + we need to assume volatile.
        */
        volatile MC_HALL_DATA *pHall = &hall;
        volatile MCAF_MOTOR_DATA *pPMSM = &PMSM;
        volatile MCAF_SYSTEM_DATA *pSysData = &sysData;
        volatile MCAF_WATCHDOG_T *pWatchdog = &watchdog;

        MCAF_UiStepMain(&pPMSM->ui);
        MCAF_SystemStateMachine_StepMain(pPMSM);
        MCAF_WatchdogManageMainLoop(pWatchdog);
        MCAF_TestHarnessStepMain(&pSysData->testing);
        MCAF_DiagnosticsStepMain();
    }    
}


/** FIGURE OUT WHAT WE NEED HERE **************
 */
bool MainInit(void)
{
    MCAF_SystemStateInit(&PMSM, &sysData);
    MCAF_SystemInit(&sysData);
    MCAF_BoardServiceInit(&sysData.board);
    MCAF_UiInit(&PMSM.ui);
    MCAF_MonitorInit(&PMSM.monitor);
#if MCAF_INCLUDE_STALL_DETECT  
    MCAF_StallDetectInit(&PMSM.stallDetect);
#endif
    MCAF_FaultDetectInit(&PMSM.faultDetect);
    MCAF_RecoveryInit(&PMSM.recovery);
    MCAF_SystemStateMachine_Init(&PMSM);
    MCAF_SystemTestHarness_Init(&sysData.testing);
    
    /* Check reset cause and act upon it, prior to clearing the watchdog,
     * (see notes in declaration of MCAF_CheckResetCause)
     */
    MCAF_CheckResetCause();
    HAL_WATCHDOG_Timer_Enable();
    MCAF_WatchdogManageMainLoop(&watchdog);
    
    MCAPI_Initialize(&PMSM.apiData);
    
    /* Ideally, MCAF has nothing to do with the application and hence this
       function should be called from the main() in main.c */
    APP_ApplicationInitialize(&PMSM.apiData, &sysData.board);
    
    MCAF_InitControlParameters_Motor1(&PMSM);
    
    bool success = MCAF_FocInit(&PMSM);
    if (success)
    {
        MCAF_SystemStart(&sysData);

        /* Ideally, MCAF has nothing to do with the application timer and hence this
           function should be called from the main() in main.c */
        HAL_TMR_TICK_Start();
    }
    return success;
}

