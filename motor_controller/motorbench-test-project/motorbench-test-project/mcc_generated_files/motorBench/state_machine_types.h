/**
 * state_machine_types.h
 * 
 * Defines the state machine states and other type definitions
 * 
 * Component: state machine
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

#ifndef __STATE_MACHINE_TYPES_H
#define __STATE_MACHINE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Top-level finite state machine state for motor controller 
 */
typedef enum tagMCAF_FSM_STATE 
{ 
    MCSM_RESTART      = 0,  /** restart */
    MCSM_STOPPED      = 1,  /** motor is stopped */
    MCSM_STARTING     = 2,  /** motor is starting */
    MCSM_RUNNING      = 3,  /** motor is running */
    MCSM_STOPPING     = 4,  /** motor is stopping */
    MCSM_FAULT        = 5,  /** fault detected */
    MCSM_TEST_DISABLE = 6,  /** test mode: motor disabled */
    MCSM_TEST_ENABLE  = 7,  /** test mode: motor enabled */
    MCSM_TEST_RESTART = 8   /** test mode: clean restart prior to enable */
} MCAF_FSM_STATE;

/**
 * Stopping timer state
 */
typedef struct tagStopping
{
    struct {
        int32_t duration; /** length of coastdown time */
        int32_t count;    /** timer count to keep track of coastdown time */
        uint16_t rate;    /** rate at which timer decrements */
    } timer;              /** timer state */
    MCAF_U_VELOCITY_ELEC speedThreshold; /** threshold for declaring stopping is complete */
} MCAF_STOPPING_STATE;

#ifdef __cplusplus
}
#endif

#endif /* __STATE_MACHINE_TYPES_H */
