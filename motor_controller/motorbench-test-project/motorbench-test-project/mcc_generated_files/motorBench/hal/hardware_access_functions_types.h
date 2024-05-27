/**
 * hardware_access_functions_types.h
 *
 * This module provides types for hardware access functions module.
 *
 * Component: HAL
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
 ******************************************************************************/

#ifndef __HAF_TYPES_H
#define __HAF_TYPES_H

#include <stdbool.h>
#include <stdint.h>


/******************************************************************************/
/* Section: Public type defines                                               */                                        
/******************************************************************************/

typedef enum tagHAL_BOARD_STATUS
{ 
    /** Board is not ready */
    HAL_BOARD_NOT_READY = 0,  
    /** Board error */
    HAL_BOARD_ERROR = 1,
    /** Board is busy, try again later */
    HAL_BOARD_BUSY = 3,
    /** Board ready/OK */
    HAL_BOARD_READY = 4
} HAL_BOARD_STATUS;

typedef enum tagHAL_ADC_SELECT_T
{
    HADC_POTENTIOMETER = 0,
    HADC_VDC           = 1
} HAL_ADC_SELECT_T;

typedef struct tagHAL_ADC_INPUTS_T
{
    uint16_t potentiometer;
    uint16_t vDC;
} HAL_ADC_INPUTS_T;

typedef struct tagHAL_DATA_T
{
    HAL_ADC_SELECT_T adcSelect;
    HAL_ADC_INPUTS_T adcInputs;
} HAL_DATA_T;

#endif /* __HAF_TYPES_H */
