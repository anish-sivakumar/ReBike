/**
 * error_codes.h
 * 
 * Application error codes
 * 
 * Component: external interface
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

#ifndef __ERROR_CODES_H
#define __ERROR_CODES_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Error codes
 */
typedef enum tagMCAF_ERROR_CODE
{
    ERR_NO_ERROR                  =  0,  /** absence of error */
            
    /* ----- Traps ----- */
    ERR_OSC_FAIL                  =  1,  /** oscillator failure */
    ERR_ADDRESS_ERROR             =  2,  /** address error (e.g. alignment) */
    ERR_HARD_TRAP                 =  3,  /** hard trap */
    ERR_STACK_ERROR               =  4,  /** stack pointer error */
    ERR_MATH                      =  5,  /** arithmetic error */
    ERR_RESERVED_TRAP5            =  6,  /** code for trap 5 unused */
    ERR_SOFT_TRAP                 =  7,  /** hard trap */
    ERR_RESERVED_TRAP7            =  8,  /** reserved trap #7 */

    ERR_UNEXPECTED_TRAP           = 255, /** unexpected trap from MCC trap handler */
    
    /* ----- Application errors ----- */
    ERR_STALL_RETRY_EXCEEDED      = 16,  /** number of stall retries exceeded */
    ERR_INVALID_STARTUP_FSM_STATE = 17,  /** invalid startup FSM state */
    ERR_HW_OVERCURRENT            = 18,  /** hardware overcurrent */
    ERR_DCLINK_OVERVOLTAGE        = 19,  /** DC link overvoltage */
    ERR_DCLINK_UNDERVOLTAGE       = 20,  /** DC link undervoltage */
    ERR_OVERTEMPERATURE           = 21,  /** Overtemperature */
    ERR_COMMUTATION_FAIL          = 22,  /** Commutation failure, detected within estimator */
    ERR_APP_COMMUTATION_FAIL      = 23,  /** Commutation failure, detected within application */
            
    MCAF_ERR_RCON_TRAPR                = 32,  /** trap conflict */
    MCAF_ERR_RCON_IOPUWR               = 33,  /** IOPUWR illegal opcode / uninitialized W */
    MCAF_ERR_RCON_CM                   = 34,  /** configuration mismatch */
    MCAF_ERR_RCON_WDTO_ISR             = 35,  /** watchdog timeout, ISR */
    ERR_RCON_WDTO_MAINLOOP        = 36,  /** watchdog timeout, main loop */
    ERR_BOARD_CONFIG_FAIL         = 37,  /** board configuration failed */
    ERR_BOARD_FAULT               = 38,  /** board fault detected */

    // Note: 240-255 are reserved for other MCAF errors
    
    /* ----- Unexpected interrupt errors ----- */
    ERR_UNEXPECTED_INTERRUPT_BASE = 256,
} MCAF_ERROR_CODE;

#ifdef __cplusplus
}
#endif

#endif /* __ERROR_CODES_H */
