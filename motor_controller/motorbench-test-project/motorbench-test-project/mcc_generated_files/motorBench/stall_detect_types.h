/**
 * stall_detect_types.h
 * 
 * This Module holds typedef structure used in stall_detect.h 
 * 
 * Component: supervisory
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

#ifndef __STALL_DETECT_TYPES_H
#define __STALL_DETECT_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "motor_control_types.h"
#include "filter_types.h"

/**
 * State variables related to variance/loss of lock stall detector 
 */
typedef struct tagVarianceDetectx16
{
    /** 1st stage of decimated LPF for EsSqr */
    MCAF_LPF_FILTER_X16_T lpf1EsSqr; 
    /** 2nd stage of decimated LPF for EsSqr */
    MCAF_LPF_FILTER_X16_T lpf2EsSqr; 
    /** 1st stage of decimated LPF for output of High pass filter for EsSqr */
    MCAF_LPF_FILTER_X16_T lpf1HpEsSqr; 
    /** 2nd stage of decimated LPF for output of High pass filter for EsSqr */
    MCAF_LPF_FILTER_X16_T lpf2HpEsSqr;
    /** HPF for Ed */
    MCAF_HPF_FILTER_X16_T hpfEd;
    /** HPF for Eq */
    MCAF_HPF_FILTER_X16_T hpfEq;
    /** output of 1st stage decimated LPF for high pass filtered EsSqr */
    int16_t esLp;
    /** Timer for active fault state */
    uint32_t timer; 
    /** Threshold for timer to trigger fault */
    uint32_t timerThreshold; 
} MCAF_VARIANCE_DETECT_T;

/**
* State variables related to over current stall detector
*/
typedef struct tagOverCurrent
{
    MCAF_LPF_FILTER_X16_T lpfIs; /** LPF for Is */
    MCAF_U_CURRENT overcurrentThreshold;  /** Threshold for overcurrent stall detect */
    int16_t is;                    /** Id^2 + Iq^2  */
    uint32_t timer;                /** Timer for active fault state */
    uint32_t timerThreshold;       /** Threshold for timer to trigger fault */ 
} MCAF_OVERCURRENT_SW_DETECT_T;

/**
 * State variables related to negative Ed stall detector
 */
typedef struct tagNegativeEdDetect
{
    int16_t edThreshold;             /** Threshold for negative Ed stall detect */
    uint32_t timerInactive;          /** Timer for inactive fault state */
    uint32_t timerInactiveThreshold; /** Threshold to reset timer */
    uint32_t timerActive;            /** Timer for active fault state */
    uint32_t timeActiveThreshold;    /** Threshold for timer to trigger fault */
} MCAF_NEGATIVE_ED_DETECT_T;

#define MCAF_TORQUE_ANGLE_POLY_COEF_LEN 3

/**
 * State variables related to torque angle based stall detector
 * 
 * This includes intermediate computations we use to
 * determine the threshold for torque angle detection,
 * comparing g(abs(x))*Es^2 with k*abs(x)*Vs^2,
 * where x is the normalized velocity (velocity / velocity scaling factor),
 * and g(x) is a polynomial that approximates an ideal nonpolynomial 
 * threshold function.
 * 
 * For further details, please read the MCAF documentation
 * on stall detection.
 */
typedef struct tagTorqueAngleDetect
{
    int16_t esSqr;                    /** Ed^2 + Eq^2 */
    int16_t vsSqr;                    /** Vq^2 + Vd^2 */

   /** Stall constant is used for calculating the theoretical ratio in
    * between Vs and Es before exceeding maximum torque angle */
    uint32_t timerInActive;          /** Timer for active fault state */
    uint32_t timerInActiveThreshold; /** Threshold to reset timer */
    uint32_t timerActive;            /** Timer for inactive fault state */
    uint32_t timerActiveThreshold;   /** Threshold for timer to trigger fault */
    MCAF_U_VELOCITY_ELEC velocityThreshold; /** Maximum velocity for torque angle detection */
    
    int16_t k;        /** scaling factor for torque angle threshold */
    int16_t kx;       /** k*abs(x) */
    
    /** g(abs(x)) evaluated using a polynomial */
    int16_t g;                       
    
    /** g(abs(x))*Es^2 */
    int16_t gEsSqr;
    
    /** k*abs(x)*Vs^2 */    
    int16_t kxVsSqr;
    
    /** an array of polynomial coefficients;
     * element 0 is the constant term,
     * element 1 is the linear term,
     * etc.
     */
    int16_t polycoef[MCAF_TORQUE_ANGLE_POLY_COEF_LEN];
    
    /**
     * gEsSqr / kxVsSqr
     * 
     * Note: this computation is present only for testing purposes
     * (not part of the torque angle detection algorithm itself)
     * and is likely to be removed in a future revision.
     */
    int16_t quotient;
} MCAF_TORQUE_ANGLE_DETECT_T;

/**
 * State variables related to low speed stall detection
 */
typedef struct tagLowSpeedDetect
{
    MCAF_U_VELOCITY_ELEC lowSpeedThreshold;  /** Threshold for low speed stall detect */
    uint32_t timerInActive;             /** Timer for inactive fault state */
    uint32_t timerInActiveThreshold;    /** Threshold to reset timer */
    uint32_t timerActive;               /** Timer for active fault state */
    uint32_t timerActiveThreshold;      /** Threshold for timer to trigger fault */
} MCAF_LOW_SPEED_DETECT_T;

/**
 * Flags to indicate active/inactive condition of stall detectors 
 */
typedef enum
{
    MCAF_OVERCURRENT_STALL_DETECT = 0x01,       /** Overcurrent stall indicator */
    MCAF_LOSS_OF_LOCK_STALL_DETECT = 0x02,      /** Loss of Lock stall indicator */
    MCAF_LOW_SPEED_STALL_DETECT = 0x04,         /** Low speed stall indicator */
    MCAF_NEGATIVE_ED_STALL_DETECT = 0x08,       /** Negative Ed stall indicator */
    MCAF_TORQUE_ANGLE_STALL_DETECT = 0x10,      /** Torque angle based stall indicator */
    
    MCAF_ALL_STALL_DETECT_METHODS_ENABLED = ~0  /** Mask for enabling all detection methods. */
} MCAF_STALL_DETECT_FLAG;
  
/**
 * Inputs needed for stall detect
 */
typedef struct tagStallDetectInput
{
    MCAF_U_VOLTAGE_DQ esdq;                       /** Estimated stator back-emf Es */
    MCAF_U_VOLTAGE_DQ esdqFiltered;               /** Filtered version of Es */
    MCAF_U_VOLTAGE_ALPHABETA valphabeta;          /** Stationary-frame alpha-beta stator voltage */
    MCAF_U_VELOCITY_ELEC omegaElectrical;    /** Electrical frequency */
} MCAF_STALL_DETECT_INPUT_T;

/**
 * State variables related to stall detect
 */
typedef struct tagStallDetect
{
    MCAF_STALL_DETECT_INPUT_T inputs;             /** Input signals */
    MCAF_VARIANCE_DETECT_T varianceDetect;        /** Ed and Eq variance detect */
    MCAF_OVERCURRENT_SW_DETECT_T overcurrentDetect;  /** Overcurrent detect */
    MCAF_NEGATIVE_ED_DETECT_T negativeEdDetect;   /** Negative Ed based stall detect */
    MCAF_LOW_SPEED_DETECT_T lowSpeedDetect;       /** Low speed detect */
    MCAF_TORQUE_ANGLE_DETECT_T torqueAngleDetect; /** Torque angle based stall detect */
    uint16_t stallDetectFlag;           /** Holds status of various stall detectors */
    uint16_t stallDetectFlagMask;       /** Controls which stall detect flags cause a fault */
    uint16_t decimationTimer;           /** Timer for decimation based variance detect */
    uint16_t decimationTimerThreshold;  /** Threshold for decimation timer */
    bool active;                        /** whether stall detection is active */
} MCAF_STALL_DETECT_T;

#ifdef __cplusplus
}
#endif

#endif /* STALL_DETECT_TYPES_H */
