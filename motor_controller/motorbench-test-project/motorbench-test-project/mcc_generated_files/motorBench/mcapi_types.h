/**
 * mcapi.h
 * 
 * Provides APIs that can be used by an application to control the motor.
 * 
 * Component: MCAPI
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

#ifndef __MCAPI_TYPES_H
#define __MCAPI_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "units.h"
#include "filter_types.h"


#ifdef __cplusplus
extern "C" {
#endif

/** Abstracted motor status data */
typedef enum tagMCAPI_MOTOR_STATE
{
    MCAPI_MOTOR_STOPPED      = 1,  /** motor is stopped */
    MCAPI_MOTOR_STARTING     = 2,  /** motor is starting */
    MCAPI_MOTOR_RUNNING      = 3,  /** motor is running */
    MCAPI_MOTOR_STOPPING     = 4,  /** motor is stopping */
    MCAPI_MOTOR_FAULT        = 5,  /** fault detected */
    MCAPI_MOTOR_DIAGSTATE    = 6   /** motor drive in diagnostic state */
} MCAPI_MOTOR_STATE;

/** Abstracted motor drive fault data */
typedef enum tagMCAPI_FAULT_FLAGS
{
    MCAPI_FAULT_FLAG_NO_FAULT        = 0,      /** no faults detected */
    MCAPI_FAULT_FLAG_OVERCURRENT     = 0x01,   /** over-current fault has occurred */
    MCAPI_FAULT_FLAG_UNDERVOLTAGE    = 0x02,   /** under-voltage fault has occurred */
    MCAPI_FAULT_FLAG_OVERVOLTAGE     = 0x04,   /** over-voltage fault has occurred */
    MCAPI_FAULT_FLAG_OVERTEMPERATURE = 0x08,   /** over-temperature fault has occurred */
    MCAPI_FAULT_FLAG_MOTOR_DRIVE     = 0x10,   /** higher level fault that encapsulates 
                                                * all other unspecified faults */
} MCAPI_FAULT_FLAGS;

/** MCAPI motor data */
typedef struct tagMCAPImotordata
{
    /** flag used by MCAF to detect if an API function call
     * was interrupted by the MCAF ISR. If this flag is set
     * then MCAF will hold off on updating/reading the shared
     * data in this data structure to avoid concurrency issues */
    bool apiBusy;
    /** flag used by MCAPI to request MCAF to turn on the motor */
    bool runMotorRequest;
    /** flag used by MCAPI to request MCAF to turn off the motor */
    bool stopMotorRequest;                    
    /** previous value of velocity reference set by the application */
    MCAF_U_VELOCITY_ELEC velocityReferencePrevious;
    /** velocity reference set by the application */
    MCAF_U_VELOCITY_ELEC velocityReference;
    /** low-pass filtered value of velocity estimated in MCAF */
    MCAF_U_VELOCITY_ELEC velocityMeasured;
    /** minimum velocity reference supported by MCAF */
    MCAF_U_VELOCITY_ELEC velocityMinimum;
    /** maximum velocity command supported by MCAF */
    MCAF_U_VELOCITY_ELEC velocityMaximum;
    /** low-pass filtered squared value of current magnitude measured in MCAF */
    int16_t isMagSquaredFiltered;
    /** low-pass filtered value of q-axis current measured in MCAF */
    int16_t iqFiltered;
    /** measured value of DC link voltage read from MCAF */
    MCAF_U_VOLTAGE dcLinkVoltage;
    /** q axis current upper limit used by the MCAF velocity control loop */
    int16_t currentLimitIqUpper;
    /** q axis current lower limit used by the MCAF velocity control loop */
    int16_t currentLimitIqLower;
    /** bit field of individual faults read from MCAF */
    uint16_t faultFlags;
    /** bit field of individual faults that are requested by MCAPI in order to 
     * be cleared by MCAF */
    uint16_t faultClearFlags;
    /** abstracted motor state read from MCAF */
    MCAPI_MOTOR_STATE motorStatus;
    /** application-level fault code */
    uint16_t appFaultCode;
} MCAPI_MOTOR_DATA;

/** MCAPI related data that is intended to be 
 * published from MCAF through MCAPI */
typedef struct tagMCAPI_FEEDBACK_SIGNALS
{
    /** low pass filtered measured current in rotating (q) frame */
    MCAF_U_CURRENT           iqFiltered;
    /** squared magnitude of measured current */
    MCAF_U_CURRENT           isSquared;
    /** low pass filtered squared magnitude of measured current */
    MCAF_U_CURRENT           isSquaredFiltered;
    /** state data for LPF on Iq */
    MCAF_LPF_FILTER_X16_T    iqFiltState;
    /** state data for LPF on Is^2 */
    MCAF_LPF_FILTER_X16_T    isSquaredFiltState;
} MCAPI_FEEDBACK_SIGNALS;

#ifdef __cplusplus
}
#endif

#endif /* __MCAPI_TYPES_H */