/**
 * units.h
 * 
 * Unit definitions
 * 
 * Component: miscellaneous
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

#ifndef __UNITS_H
#define __UNITS_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_control_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Dimensionless quantities like sine and cosine */    
typedef int16_t MCAF_U_DIMENSIONLESS;

/** Sine and cosine are dimensionless */
typedef MC_SINCOS_T MCAF_U_DIMENSIONLESS_SINCOS;

/** Dimensionless three-phase quantity */
typedef MC_ABC_T MCAF_U_DIMENSIONLESS_ABC;

/** Dimensionless three-phase quantity, Q14 */
typedef MC_ABC_T MCAF_U_DIMENSIONLESS_ABC_Q14;

/** Dimensionless two-phase quantity, Q14 */
typedef MC_ALPHABETA_T MCAF_U_DIMENSIONLESS_ALPHABETA_Q14;

/** Normalized gain (example: current gain compensation) */    
typedef int16_t MCAF_U_NORMALIZED_GAIN;

/** Electrical frequency */
typedef int16_t MCAF_U_VELOCITY_ELEC;

/** Mechanical velocity */
typedef int16_t MCAF_U_VELOCITY_MECH;

/** Velocity -- interchangeable at runtime; can be either electrical or mechanical
 *  as they have the same fullscale value. (This interchangeability is
 *  NOT true for angle.)
 * 
 *  Example: for a 6-pole motor,
 *  fullscale velocity might be 1000rad/s mechanical = 3000rad/s electrical
 */
typedef union
{
    MCAF_U_VELOCITY_ELEC electrical;
    MCAF_U_VELOCITY_MECH mechanical;
} MCAF_U_VELOCITY;

/** Electrical angle
 * (wraparound semantics dictates 32768 counts = pi rad electrical) */
typedef int16_t MCAF_U_ANGLE_ELEC;

/** Mechanical angle
 * (wraparound semantics dictates 32768 counts = pi rad mechanical) */
typedef int16_t MCAF_U_ANGLE_MECH;

/** Electrical frequency defined as a change in electrical
 *  angle per sampling period
 */
typedef int16_t MCAF_U_VELOCITY_DTHETA_ELEC_DT;

/** Current */
typedef int16_t MCAF_U_CURRENT;

/** Current, DQ vector (synchronous frame) */
typedef MC_DQ_T MCAF_U_CURRENT_DQ;

/** Current, alpha-beta vector (orthogonal stationary frame) */
typedef MC_ALPHABETA_T MCAF_U_CURRENT_ALPHABETA;

/** Current, ABC vector (per-phase stationary frame) */
typedef MC_ABC_T MCAF_U_CURRENT_ABC;
    
/** Voltage */
typedef int16_t MCAF_U_VOLTAGE;

/** Voltage
 *  scaled by 0.5 to allow room for overflows */
typedef int16_t MCAF_U_VOLTAGE_Q14;

/** Voltage, DQ vector (synchronous frame) */
typedef MC_DQ_T MCAF_U_VOLTAGE_DQ;

/** Voltage, DQ vector (synchronous frame),
 *  scaled by 0.5 to allow room for overflows */
typedef MC_DQ_T MCAF_U_VOLTAGE_DQ_Q14;

/** Voltage, alpha-beta vector (orthogonal stationary frame) */
typedef MC_ALPHABETA_T MCAF_U_VOLTAGE_ALPHABETA;

/** Voltage, alpha-beta vector (orthogonal stationary frame),
 *  scaled by 0.5 to allow room for overflows */
typedef MC_ALPHABETA_T MCAF_U_VOLTAGE_ALPHABETA_Q14;

/** Voltage, ABC vector (per-phase stationary frame) */
typedef MC_ABC_T MCAF_U_VOLTAGE_ABC;

/** Voltage, normalized to DC link voltage, line-to-neutral, 
 *so that 0.57735 = 1/sqrt(3) = full line-to-line voltage */
typedef MC_DQ_T MCAF_U_NORMVOLTAGE_DQ;

/** Reciprocal voltage */
typedef int16_t MCAF_U_RVOLTAGE;

/** Stator resistance */
typedef int16_t MCAF_U_STATOR_RESISTANCE;

/** Stator inductance */
typedef int16_t MCAF_U_STATOR_INDUCTANCE;

/** Back-EMF constant */
typedef int16_t MCAF_U_BACKEMF;

/** Inverse of back-EMF constant */
typedef int16_t MCAF_U_BACKEMF_INVERSE;

/** Magnetic flux */
typedef int16_t MCAF_U_FLUX_LINKAGE;

/** Duty cycle */
typedef int16_t MCAF_U_DUTYCYCLE;

/** Duty cycle, alpha-beta vector (orthogonal stationary frame) */
typedef MC_ALPHABETA_T MCAF_U_DUTYCYCLE_ALPHABETA;

/** Duty cycle (per-phase stationary frame) */
typedef MC_ABC_T MCAF_U_DUTYCYCLE_ABC;
    
/** Temperature */
typedef int16_t MCAF_U_TEMPERATURE;

#ifdef __cplusplus
}
#endif

#endif /* __UNITS_H */
