/**
 * mcaf_main.c
 *
 * Main program entry point
 * 
 * Component: main application
 */
/*
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

/** Global instance of the main set of motor state variables */
MCAF_MOTOR_DATA motor;
/** Global instance of the main set of system state variables */
MCAF_SYSTEM_DATA systemData;

extern volatile MCAF_WATCHDOG_T watchdog;

bool MCAF_MainInit(void)
{
    MCAF_SystemStateInit(&motor, &systemData);
    MCAF_SystemInit(&systemData);
    MCAF_BoardServiceInit(&systemData.board);
    MCAF_UiInit(&motor.ui);
    MCAF_MonitorInit(&motor.monitor);
#if MCAF_INCLUDE_STALL_DETECT  
    MCAF_StallDetectInit(&motor.stallDetect);
#endif
    MCAF_FaultDetectInit(&motor.faultDetect);
    MCAF_RecoveryInit(&motor.recovery);
    MCAF_SystemStateMachine_Init(&motor);
    MCAF_SystemTestHarness_Init(&systemData.testing);
    
    /* Check reset cause and act upon it, prior to clearing the watchdog,
     * (see notes in declaration of MCAF_CheckResetCause)
     */
    MCAF_CheckResetCause();
    HAL_WATCHDOG_Timer_Enable();
    MCAF_WatchdogManageMainLoop(&watchdog);
    
    MCAPI_Initialize(&motor.apiData);
    
    /* Ideally, MCAF has nothing to do with the application and hence this
       function should be called from the main() in main.c */
    APP_ApplicationInitialize(&motor.apiData, &systemData.board);
    
    MCAF_InitControlParameters_Motor1(&motor);
    
    bool success = MCAF_FocInit(&motor);
    if (success)
    {
        MCAF_SystemStart(&systemData);

        /* Ideally, MCAF has nothing to do with the application timer and hence this
           function should be called from the main() in main.c */
        HAL_TMR_TICK_Start();
    }
    return success;
}

void MCAF_MainLoop(void)
{
    /* State variables are fine to access w/o volatile qualifier for ISR
     * (since no interruptions)
     * but in main loop, the ISR may interrupt + we need to assume volatile.
     */
    volatile MCAF_MOTOR_DATA *pmotor = &motor;
    volatile MCAF_SYSTEM_DATA *psystemData = &systemData;
    volatile MCAF_WATCHDOG_T *pwatchdog = &watchdog;
    
    MCAF_UiStepMain(&pmotor->ui);
    MCAF_SystemStateMachine_StepMain(pmotor);
    MCAF_WatchdogManageMainLoop(pwatchdog);
    MCAF_TestHarnessStepMain(&psystemData->testing);
    MCAF_DiagnosticsStepMain();
}
