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

#ifndef __MCAPI_H
#define __MCAPI_H

#include <stdint.h>
#include <stdbool.h>
#include "mcapi_types.h"
#include "math_asm.h"
#include "util.h"
#include "parameters/mcapi_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes MCAPI internal state data.
 * This function should be called once before calling any of the MCAPI
 * functions for the first time.
 * @param pMotor
 */
static inline void MCAPI_Initialize(volatile MCAPI_MOTOR_DATA *pMotor)
{
   pMotor->apiBusy = false; 
   pMotor->appFaultCode = 0;
   pMotor->currentLimitIqLower = 0; 
   pMotor->currentLimitIqUpper = 0; 
   pMotor->dcLinkVoltage = 0; 
   pMotor->faultClearFlags = MCAPI_FAULT_FLAG_NO_FAULT; 
   pMotor->faultFlags = MCAPI_FAULT_FLAG_NO_FAULT; 
   pMotor->iqFiltered = 0; 
   pMotor->isMagSquaredFiltered = 0; 
   pMotor->motorStatus = MCAPI_MOTOR_STOPPED; 
   pMotor->runMotorRequest = false; 
   pMotor->stopMotorRequest = false; 
   pMotor->velocityMeasured = 0; 
   pMotor->velocityMinimum = MCAPI_MINIMUM_VELOCITY;
   pMotor->velocityMaximum = MCAPI_MAXIMUM_VELOCITY;
   pMotor->velocityReference = pMotor->velocityMinimum;
   pMotor->velocityReferencePrevious = pMotor->velocityMinimum;
}
    
/**
 * Starts the specified motor. There will be no change if the motor is already 
 * running. If there is an active MCAF fault, this function will have no effect.
 * @param pMotor
 */
static inline void MCAPI_MotorStart(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    pMotor->runMotorRequest = true;
    pMotor->apiBusy = false;
}

/**
 * Stops the specified motor. There will be no change if the motor is already 
 * in fault, stopped or stopping states.
 * @param pMotor
 */
static inline void MCAPI_MotorStop(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    pMotor->stopMotorRequest = true;
    pMotor->apiBusy = false;
}

/**
 * Uses the specified value to update velocity reference for the specified motor. 
 * Sign of the input value will determine the direction of rotation of the motor. 
 * Changes in velocity command when the motor is not in running state will be 
 * saved and will take effect only when the motor reaches this state.
 * Before this function is called for the first time after initialization, the 
 * set value of velocity reference is either zero, or equal to the minimum 
 * value of velocity reference specified by MCAPI_VelocityReferenceMinimumGet()
 * @param pMotor
 * @param velocity reference value, signed Q15 with the scaling factor 
 * specified by MCAPI_FullscaleVelocityGet()
 */
static inline void MCAPI_VelocityReferenceSet(volatile MCAPI_MOTOR_DATA *pMotor, int16_t velocity)
{
    pMotor->apiBusy = true;
    pMotor->velocityReference = velocity;
    pMotor->apiBusy = false;
}

/**
 * Returns velocity reference value for the specified motor. 
 * The return value will exactly match the set value set by the "set velocity" 
 * function.
 * Before the "set velocity" function is called for the first time after 
 * initialization, the return value of velocity reference is either zero, or 
 * equal to the minimum value of velocity reference specified 
 * by MCAPI_VelocityReferenceMinimumGet()
 * @param pMotor
 * @return velocity reference for the specified motor, signed Q15 with the 
 * scaling factor specified by MCAPI_FullscaleVelocityGet()
 */
static inline int16_t MCAPI_VelocityReferenceGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    int16_t velocityReference = pMotor->velocityReference;
    pMotor->apiBusy = false;
    return velocityReference;
}

/**
 * Returns velocity of the specified motor as measured by an estimator in MCAF.
 * Sign of the input value will determine the direction of rotation of the motor.
 * @param pMotor
 * @return measured motor velocity, signed Q15 with the scaling factor 
 * specified by MCAPI_FullscaleVelocityGet()
 */
static inline int16_t MCAPI_VelocityMeasuredGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    int16_t velocityMeasured = pMotor->velocityMeasured;
    pMotor->apiBusy = false;
    return velocityMeasured;
}

/**
 * Returns a value that represents the amplitude of current in the 
 * specified motor, that is sqrt(Id*Id + Iq*Iq).
 * Since this value is derived from a square root operation, this function
 * will require additional CPU cycles to execute when compared to other
 * getter functions in MCAPI.
 * Since this value is derived from signals with low pass filtering, this is 
 * not an instantaneous value and will have some delay when compared to the 
 * actual current in the motor.
 * @param pMotor
 * @return current amplitude in the motor, signed Q15 with the scaling factor 
 * specified by MCAPI_FullscaleCurrentGet()
 */
static inline int16_t MCAPI_CurrentMagnitudeGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    int16_t isMagSquaredFiltered = pMotor->isMagSquaredFiltered;
    pMotor->apiBusy = false;
    return Q15SQRT(isMagSquaredFiltered);
}

/**
 * Returns a bit-field with individual flags for each fault type for the 
 * specified motor.
 * @param pMotor
 * @return fault flags
 */
static inline uint16_t MCAPI_FaultStatusGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    uint16_t faultFlags = pMotor->faultFlags;
    pMotor->apiBusy = false;
    return faultFlags;
}

/**
 * Clears the specified fault flags for a specified motor. This function does 
 * not implicitly start the motor after clearing one or more active faults.
 * @param pMotor
 * @param mask bit mask with desired fault flags to be cleared set to 1
 */
static inline void MCAPI_FaultStatusClear(volatile MCAPI_MOTOR_DATA *pMotor, 
                                          uint16_t mask)
{
    pMotor->apiBusy = true;
    pMotor->faultClearFlags = UTIL_SetBits(pMotor->faultClearFlags, mask);
    pMotor->apiBusy = false;
}

/**
 * Gets status of the specified motor.
 * @param pMotor
 * @return state value that categorizes the motor operation status
 */
static inline MCAPI_MOTOR_STATE MCAPI_OperatingStatusGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    uint16_t motorStatus = pMotor->motorStatus;
    pMotor->apiBusy = false;
    return motorStatus;
}

/**
 * Returns a value that represents the measured q-axis current in the 
 * specified motor. 
 * Since this value is derived from signals with low pass filtering, this 
 * is not an instantaneous value and will have some delay compared to the 
 * actual current in the motor.
 * @param pMotor
 * @return q-axis current, signed Q15 with the scaling factor 
 * specified by MCAPI_FullscaleCurrentGet()
 */
static inline int16_t MCAPI_CurrentIqGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    int16_t iqFiltered = pMotor->iqFiltered;
    pMotor->apiBusy = false;
    return iqFiltered;
}

/**
 * Returns the full-scale value of current used by MCAF for the specified 
 * motor, as a floating-point number.
 * @param pMotor
 * @return full-scale value of current in Amperes
 */
static inline float MCAPI_FullscaleCurrentGet(const volatile MCAPI_MOTOR_DATA *pMotor)
{
    return MCAPI_FULLSCALE_CURRENT;
}

/**
 * Returns the full-scale value of voltage used by MCAF for the specified 
 * motor, as a floating-point number.
 * @param pMotor
 * @return full-scale value of voltage in Volts
 */
static inline float MCAPI_FullscaleVoltageGet(const volatile MCAPI_MOTOR_DATA *pMotor)
{
    return MCAPI_FULLSCALE_VOLTAGE;
}

/**
 * Returns the full-scale value of electromagnetic torque used by MCAF for 
 * the specified motor, as a floating-point number.
 * Factors such as reluctance torque, iron saturation and changes in magnet 
 * flux due to temperature can impact the accuracy of this scaling factor. Hence,
 * this is to be consumed by the application on a "for indication only" basis.
 * @param pMotor
 * @return full-scale value of torque in Nm
 */
static inline float MCAPI_FullscaleTorqueGet(const volatile MCAPI_MOTOR_DATA *pMotor)
{
    return MCAPI_FULLSCALE_TORQUE;
}

/**
 * Returns the full-scale value of velocity used by MCAF for the specified 
 * motor, as a floating-point number.
 * @param pMotor
 * @return full-scale value of velocity in RPM
 */
static inline float MCAPI_FullscaleVelocityGet(const volatile MCAPI_MOTOR_DATA *pMotor)
{
    return MCAPI_FULLSCALE_VELOCITY;
}

/**
 * Returns a value of DC link voltage in the sytem as measured by MCAF.
 * @param pMotor
 * @return measured value of DC link voltage, signed Q15 with the scaling factor 
 * specified by MCAPI_FullscaleVoltageGet()
 */
static inline int16_t MCAPI_VoltageDclinkGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    int16_t dcLinkVoltage = pMotor->dcLinkVoltage;
    pMotor->apiBusy = false;
    return dcLinkVoltage;
}

/**
 * Returns a value that represents upper bound of the current limit that 
 * is implemented in MCAF.
 * This function does not synchronize its output with its counterpart function
 * that gets the lower bound of current limit. So, while calling these two 
 * functions right after one another, there will always be some skew in the instance 
 * of time when the current limit pair value are acquired.
 * @param pMotor
 * @return upper bound of current limit, signed Q15 with the scaling factor 
 * specified by MCAPI_FullscaleCurrentGet()
 */
static inline int16_t MCAPI_CurrentLimitIqUpperGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    int16_t currentLimitIqUpper = pMotor->currentLimitIqUpper;
    pMotor->apiBusy = false;
    return currentLimitIqUpper;
}

/**
 * Returns a value that represents lower bound of the current limit that 
 * is implemented in MCAF.
 * This function does not synchronize its output with its counterpart function
 * that gets the upper bound of current limit. So, while calling these two 
 * functions right after one another, there will always be some skew in the instance 
 * of time when the current limit pair value are acquired.
 * @param pMotor
 * @return lower bound of current limit, signed Q15 with the scaling factor 
 * specified by MCAPI_FullscaleCurrentGet()
 */
static inline int16_t MCAPI_CurrentLimitIqLowerGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    int16_t currentLimitIqLower = pMotor->currentLimitIqLower;
    pMotor->apiBusy = false;
    return currentLimitIqLower;
}

/**
 * Gets the minimum value of velocity reference that is currently supported
 * in the given configuration of MCAF.
 * @param pMotor
 * @return minimum value of velocity reference, signed Q15 with the scaling factor 
 * specified by MCAPI_FullscaleVelocityGet()
 */
static inline int16_t MCAPI_VelocityReferenceMinimumGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    int16_t velocityMinimum = pMotor->velocityMinimum;
    pMotor->apiBusy = false;
    return velocityMinimum;
}

/**
 * Gets the maximum value of velocity command that is currently supported
 * in the given configuration of MCAF.
 * @param pMotor
 * @return maximum value of velocity command, signed Q15 with the scaling factor 
 * specified by MCAPI_FullscaleVelocityGet()
 */
static inline int16_t MCAPI_VelocityReferenceMaximumGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    pMotor->apiBusy = true;
    int16_t velocityMaximum = pMotor->velocityMaximum;
    pMotor->apiBusy = false;
    return velocityMaximum;
}

/**
 * Gets an application-level fault code, or ERR_NO_ERROR = 0 if no error.
 * 
 * @param pMotor
 * @return application-level fault code
 */
static inline uint16_t MCAPI_ApplicationFaultCodeGet(volatile MCAPI_MOTOR_DATA *pMotor)
{
    return pMotor->appFaultCode;
}

/**
 * Sets an application-level fault code
 * 
 * @param pMotor
 * @param faultcode application-level fault code
 */
static inline void MCAPI_ApplicationFaultCodeSet(volatile MCAPI_MOTOR_DATA *pMotor, uint16_t faultCode)
{
    pMotor->appFaultCode = faultCode;
}

#ifdef __cplusplus
}
#endif

#endif /* __MCAPI_H */