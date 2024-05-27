/**
 * test_harness.c
 * 
 * Test harness definitions
 * 
 * Component: test harness
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

#include "test_harness.h"

void MCAF_TestHarness_Restart(volatile MCAF_MOTOR_TEST_MANAGER *ptest)
{
#ifdef MCAF_TEST_HARNESS 
  #if MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC
    ptest->sqwave.idq.d = 0;
    ptest->sqwave.idq.q = 0;
    ptest->sqwave.vdq.d = 0;
    ptest->sqwave.vdq.q = 0;
    ptest->sqwave.velocity.electrical = 0;
    ptest->sqwave.value = 0;
    ptest->sqwave.count = 0;
  #else // MCAF_TEST_HARNESS_PERTURBATION_SYMMETRIC == 0
    int16_t i;
    for (i = 0; i < 2; ++i)
    {
        volatile MCAF_TEST_PERTURB_PHASE *phase = &ptest->perturb.phase[i];
        phase->idq.d = 0;
        phase->idq.q = 0;
        phase->vdq.d = 0;
        phase->vdq.q = 0;
        phase->velocity.electrical = 0;
        phase->duration = 0;
    }
    ptest->perturb.flags = 0;
    ptest->perturb.enable = 0;
    ptest->perturb.count = 0;
    ptest->perturb.autobalanceRatio = 0;
    ptest->perturb.activePhase = &ptest->perturb.phase[0];
    ptest->perturb.step.idq.d = 0;
    ptest->perturb.step.idq.q = 0;
    ptest->perturb.step.count = 0;
  #endif
    ptest->overrides = 0;
    ptest->overrideOmegaElectrical = 0;
    ptest->overrideCommutationOnOff.maxCount = 0;
    ptest->overrideCommutationOnOff.threshold = 1;
    ptest->overrideZeroSequenceOffset = 0;
    MCAF_TestHarness_ClearRestartRequired(ptest);
#endif
}

void MCAF_SystemTestHarness_Init(volatile MCAF_SYSTEM_TEST_MANAGER *ptest)
{
    ptest->flags = 0;
    ptest->guard.key = TEST_GUARD_RESET;
}

#ifdef MCAF_TEST_HARNESS
static uint16_t stack_overflow_helper(void)
{
    volatile uint16_t wasted_space[8];
    wasted_space[0] = 0;
    return stack_overflow_helper() + wasted_space[0];
}

void MCAF_TestHarness_TriggerStackOverflow(void)
{
    stack_overflow_helper();
}
#endif