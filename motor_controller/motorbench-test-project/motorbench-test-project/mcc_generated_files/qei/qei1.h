/**
 * QEI1 Generated Driver Header File 
 * 
 * @file      qei1.h
 *            
 * @ingroup   qeidriver
 *            
 * @brief     This is the generated driver header file for the QEI1 driver
 *            
 * @version   Firmware Driver Version 1.2.2
 *
 * @version   PLIB Version 1.3.0
 *            
 * @skipline  Device : PIC24/dspIC/PIC32MM
*/

/*
© [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#ifndef QEI1_H
#define QEI1_H

// Section: Included Files

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include "qei_types.h"
#include "qei_interface.h"

// Section: Data Type Definitions

/**
 * @ingroup  qeidriver
 * @brief    Structure object of type QEI_INTERFACE with the custom name
 *           given by the user in the Melody Driver User interface. The default name 
 *           e.g. QEI1 can be changed by the user in the QEI user interface. 
 *           This allows defining a structure with application specific name using 
 *           the 'Custom Name' field. Application specific name allows the API Portability.
*/
extern const struct QEI_INTERFACE MCC_QEI;
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_Disable API
 */
#define MCC_QEI_Disable QEI1_Disable
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_Enable API
 */
#define MCC_QEI_Enable QEI1_Enable
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_PositionCountRead API
 */
#define MCC_QEI_PositionCountRead QEI1_PositionCountRead
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_PositionCount16bitRead API
 */
#define MCC_QEI_PositionCount16bitRead QEI1_PositionCount16bitRead
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_PositionCountWrite API
 */
#define MCC_QEI_PositionCountWrite QEI1_PositionCountWrite
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_ModuloRangeSet API
 */
#define MCC_QEI_ModuloRangeSet QEI1_ModuloRangeSet
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_PhaseInputSwappedGet API
 */
#define MCC_QEI_PhaseInputSwappedGet QEI1_PhaseInputSwappedGet
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_PhaseInputSwappedSet API
 */
#define MCC_QEI_PhaseInputSwappedSet QEI1_PhaseInputSwappedSet
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_PositionCaptureEnable API
 */
#define MCC_QEI_PositionCaptureEnable QEI1_PositionCaptureEnable
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_PositionCaptureDisable API
 */
#define MCC_QEI_PositionCaptureDisable QEI1_PositionCaptureDisable
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_PositionCaptureGet API
 */
#define MCC_QEI_PositionCaptureGet QEI1_PositionCaptureGet
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_PositionCapture16bitGet API
 */
#define MCC_QEI_PositionCapture16bitGet QEI1_PositionCapture16bitGet
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_PositionCaptureSet API
 */
#define MCC_QEI_PositionCaptureSet QEI1_PositionCaptureSet
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_CounterModeSet API
 */
#define MCC_QEI_CounterModeSet QEI1_CounterModeSet
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_IMVGatedValueSet API
 */
#define MCC_QEI_IMVGatedValueSet QEI1_IMVGatedValueSet
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_Initialize API
 */
#define MCC_QEI_Initialize QEI1_Initialize
/**
 * @ingroup  qeidriver
 * @brief    This macro defines the Custom Name for \ref QEI1_Deinitialize API
 */
#define MCC_QEI_Deinitialize QEI1_Deinitialize
// Section: QEI1 Module APIs

/**
 * @ingroup  qeidriver
 * @brief    This inline function disables the QEI1 module
 * @return   none  
 */
inline static void QEI1_Disable(void)
{
    QEI1CONbits.QEIEN = 0;        
}
    
/**
 * @ingroup  qeidriver
 * @brief    This inline function enables the QEI1 module
 * @return   none
 */
inline static void  QEI1_Enable(void)
{
    QEI1CONbits.QEIEN = 1;        
}

/**
 * @ingroup  qeidriver
 * @brief    This inline function returns the 32-bit position count value 
 * @return   Returns the 32-bit position count register value
 */
inline static uint32_t QEI1_PositionCountRead(void)
{
 /* Note:  A read to the LSW of the position count in POSCNTL must happen first.  
 *         This will latch the MSW synchronously to the POSHLD register,
 *         to ensure a correct 32 bit readout.
 */
    return ( POS1CNTL | ((uint32_t)POS1HLD << 16U) );
}

/**
 * @ingroup  qeidriver
 * @brief    This inline function returns the 16-bit position count value from the 
 *           QEI1 position count register.
 * @return   Returns the LSB 16 bits of the QEI1 position 
 *           count register.
 */
inline static uint16_t QEI1_PositionCount16bitRead(void)
{
    return POS1CNTL;
}

/**
 * @ingroup    qeidriver
 * @brief      This inline function sets the QEI1 position count value
 * @param[in]  positionCount - 32-bit position count value 
 * @return     none  
 */
inline static void QEI1_PositionCountWrite(uint32_t positionCount)
{
 /* Note the MSW is written first to POSHLD register followed by the LSW to the lower 16 bits of
    the actual position count register. This sequence is important for a successful write operation.*/
    POS1HLD = (uint16_t)(positionCount >> 16U);
    POS1CNTL = (uint16_t)positionCount;
}

/**
 * @ingroup    qeidriver
 * @brief      This inline function sets the 32bit modulo count value when QEI is configured to
 *             operate in 'Modulo Count' mode. The lower bound controlled by the QEILEC register is set to 0.
 * @param[in]  countsPerRevolution - Modulus number of counts per wraparound  
 * @return     none  
 */
inline static void QEI1_ModuloRangeSet(uint32_t countsPerRevolution)
{
    uint32_t maxCount = countsPerRevolution - (uint32_t)1;
    QEI1LECL = 0;
    QEI1LECH = 0;
    QEI1GECL = (uint16_t) maxCount;
    QEI1GECH = (uint16_t)(maxCount >> 16U);
}

/**
 * @ingroup  qeidriver
 * @brief    This inline function gets the status of QEI input phase swap configuration  
 * @return   true   - Phase inputs are swapped
 * @return   false  - Phase inputs are not swapped
 */
inline static bool QEI1_PhaseInputSwappedGet(void)
{
    return QEI1IOCbits.SWPAB;
}

/**
 * @ingroup    qeidriver
 * @brief      This inline function sets whether the QEA and QEB pins are 
 *             swapped prior to quadrature decoder logic.
 * @param[in]  swapEnabled - specifies whether QEA and QEB pins need 
 *             to be swapped prior to quadrature decoder logic  
 * @return     none   
 */
inline static void QEI1_PhaseInputSwappedSet(bool swapEnabled)
{

    if (swapEnabled)
    {
        QEI1IOCbits.SWPAB = 1;
    }
    else
    {
        QEI1IOCbits.SWPAB = 0;   
    }  
}

/**
 * @ingroup  qeidriver
 * @brief    This inline function enables the QEI position count capture on an index event
 * @return   none  
 */
inline static void QEI1_PositionCaptureEnable(void)
{
    QEI1IOCbits.QCAPEN = 1;
}

/**
 * @ingroup  qeidriver
 * @brief    This inline function disables the QEI position count capture on an index event
 * @return   none  
 */
inline static void QEI1_PositionCaptureDisable(void)
{
    QEI1IOCbits.QCAPEN = 0;
}

/**
 * @ingroup  qeidriver
 * @brief    This inline function reads the 32-bit position capture value from the
 *           QEIICx 32 bit register.
 * @return   Returns position capture value
 */
inline static uint32_t QEI1_PositionCaptureGet(void)
{
    return (((uint32_t) QEI1ICH << 16U) | QEI1ICL );
}

/**
 * @ingroup  qeidriver
 * @brief    This inline function returns the 16-bit position capture value from 
 *           QEIICL position count capture register
 * @return   Returns the LSB 16 bits of position capture value
 */
inline static uint16_t QEI1_PositionCapture16bitGet(void)
{
    return QEI1ICL;
}

/**
 * @ingroup    qeidriver
 * @brief      This inline function initializes the 32 bit position capture register value
 * @param[in]  initValue - 32 bit position capture register value  
 * @return     none  
 */
inline static void QEI1_PositionCaptureSet(uint32_t initValue)
{
    QEI1ICL = (uint16_t) initValue;
    QEI1ICH = (uint16_t)(initValue >> 16U);
}

/**
 * @ingroup    qeidriver
 * @brief      This inline function sets the QEI Counter mode
 * @param[in]  mode - Sets the QEI counter mode 
 * @return     none  
 */
inline static void QEI1_CounterModeSet(enum QEI_MODE mode)
{
    QEI1CONbits.PIMOD = mode;
}

/**
 * @ingroup    qeidriver
 * @brief      This inline function sets the QEI Index Match Value 
 * @param[in]  mode - Sets the QEI Index Match Value
 * @return     none  
 */
inline static void QEI1_IMVGatedValueSet(enum QEI_IMV_STATE state)
{
    QEI1CONbits.IMV = state;
}

/**
 * @ingroup  qeidriver
 * @brief    Initializes the QEI module
 * @return   none 
 */
void QEI1_Initialize(void);

/**
 * @ingroup  qeidriver
 * @brief    Deinitializes the QEI1 to POR values
 * @return   none  
 */
void QEI1_Deinitialize(void);

#endif // QEI1.H
