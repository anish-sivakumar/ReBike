/**
 * mcaf_traps.c
 * 
 * Trap handlers
 * 
 * See DS70600C for more information on the non-maskable traps.
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

#include <xc.h>
#include <stdint.h>
#include "ui.h"
#include "error_codes.h"
#include "mcaf_watchdog.h"
#include "hal.h"

/**
 * a private place to store the error code if we run into a severe error
 */
static volatile uint16_t error_code = -1;

/** Global instance of watchdog data -- persistent so we can
 * distinguish main-loop failure and ISR failure
 */
volatile MCAF_WATCHDOG_T watchdog __attribute__((persistent, section("MCAF_persistent")));

/**
 * Halts and displays an error code
 * 
 * @param code error code
 */
void __attribute__((naked, noreturn)) halt_on_error(uint16_t code)
{
    error_code = code;
#ifdef __DEBUG    
    __builtin_software_breakpoint();
    /* If we are in debug mode, cause a software breakpoint in the debugger */
#endif

    MCAF_UiFlashErrorCodeForever(code);
}

#define ERROR_HANDLER __attribute__((interrupt, no_auto_psv, keep, section("error_handler")))
#define ERROR_HANDLER_NORETURN ERROR_HANDLER __attribute__((noreturn))

inline static uint16_t translate_error_code(uint16_t mcc_code)
{
    switch (mcc_code)
    {
        case TRAPS_OSC_FAIL:
            return ERR_OSC_FAIL;
        case TRAPS_STACK_ERR:
            return ERR_STACK_ERROR;
        case TRAPS_ADDRESS_ERR:
            return ERR_ADDRESS_ERROR;
        case TRAPS_MATH_ERR:
            return ERR_MATH;
        case TRAPS_HARD_ERR:
            return ERR_HARD_TRAP;
        case TRAPS_DOOVR_ERR:
            return ERR_SOFT_TRAP;
        default:
            return ERR_UNEXPECTED_TRAP;
    }
}

/** Reserved trap error */
void ERROR_HANDLER_NORETURN _ReservedTrap7(void) { halt_on_error(ERR_RESERVED_TRAP7); }

/** 
 * Reports unexpected interrupt, by displaying error vector number.
 */
void ERROR_HANDLER_NORETURN _DefaultInterrupt(void) {
    halt_on_error(ERR_UNEXPECTED_INTERRUPT_BASE + HAL_InterruptVector_Get());
}

void MCAF_CheckResetCause(void)
{
    const uint16_t rconcopy = RCON;
    uint16_t errorCode = 0;
    if (rconcopy & 0x8000) /* TRAPR */
    {
        errorCode = MCAF_ERR_RCON_TRAPR;
    }
    else if (rconcopy & 0x4000) /* IOPUWR */
    {
        errorCode = MCAF_ERR_RCON_IOPUWR;    
    }
    else if (rconcopy & 0x0200) /* CM */
    {
        errorCode = MCAF_ERR_RCON_CM;    
    }
    else if (rconcopy & 0x0010) /* WDTO */
    {
        if (watchdog.isrCount < MCAF_WATCHDOG_MAINLOOP_TIMEOUT)
        {
            errorCode = MCAF_ERR_RCON_WDTO_ISR;    
        }
        else
        {
            errorCode = ERR_RCON_WDTO_MAINLOOP;
        }
    }
    RCON = 0;    // clear RCON bits so that at next reset they will be accurate
    
    if (errorCode != 0)
    {
        MCAF_UiFlashErrorCodeForever(errorCode);
    }            
}

/**
 * Error handler for traps
 * This overrides MCC's error handler (which has a "weak" attribute)
 */
void __attribute__((naked)) TRAPS_halt_on_error(uint16_t code)
{
    if (code == TRAPS_STACK_ERR)
        halt_on_error(ERR_STACK_ERROR);
#ifdef __dsPIC33E__
    if (code == TRAPS_DMAC_ERR)
    {
        if (HAL_DMA_ErrorHandler())
        {
            return; //we can recover, no need to halt
        }
    }
#endif
    halt_on_error(translate_error_code(code));
}
