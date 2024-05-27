/**
 * OPA2 Generated Driver Header File
 * 
 * @file      opa2.h
 * 
 * @ingroup   opadriver
 * 
 * @brief     This is the generated driver header file for the OPA2 driver
 *
 * @version   Firmware Driver Version 1.2.1
 *
 * @version   PLIB Version 1.3.0
 *
 * @skipline  Device : dsPIC33CK256MP508
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

#ifndef OPA2_H
#define OPA2_H

// Section: Included Files

#include <xc.h>
#include <stdint.h>
#include "opa_types.h"
#include "opa_interface.h"
// Section: Data Type Definitions

/**
 * @ingroup  opadriver
 * @brief    Structure object of type OPA_INTERFACE with the custom name
 *           given by the user in the Melody Driver User interface. The default name 
 *           e.g. OPA2 can be changed by the user in the OPA user interface. 
 *           This allows defining a structure with application specific name using 
 *           the 'Custom Name' field. Application specific name allows the API Portability.
*/
extern const struct OPA_INTERFACE MCC_OPA_IB;

/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA2_Initialize API
 */
#define MCC_OPA_IB_Initialize OPA2_Initialize
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA2_Deinitialize API
 */
#define MCC_OPA_IB_Deinitialize OPA2_Deinitialize
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA2_Enable API
 */
#define MCC_OPA_IB_Enable OPA2_Enable
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA2_Disable API
 */
#define MCC_OPA_IB_Disable OPA2_Disable
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA2_InputVoltageRangeSelect API
 */
#define MCC_OPA_IB_InputVoltageRangeSelect OPA2_InputVoltageRangeSelect


// Section: Interface Routines

/**
 * @ingroup  opadriver
 * @brief    Initializes the OPA2 module
 * @return   none  
 */
void OPA2_Initialize (void);

/**
 * @ingroup  opadriver
 * @brief    Deinitializes the OPA2 to POR values
 * @return   none  
 */
void OPA2_Deinitialize(void);

/**
 * @ingroup  opadriver
 * @brief    This inline function enables the OPA2 module
 * @pre      The OPA2_Initialize function should be called for the specified 
 *           OPA2 driver instance
 * @return   none  
 */
inline static void OPA2_Enable( void )
{
     AMPCON1Lbits.AMPEN2 = 1; //Enable opa2;
}

/**
 * @ingroup  opadriver
 * @brief    This inline function disables the OPA2 module
 * @return   none  
 */
inline static void OPA2_Disable( void )
{
     AMPCON1Lbits.AMPEN2 = 0; //Disable opa2;
}

/**
 * @ingroup    opadriver
 * @brief      This inline function selects OPA2 Wide Input or Lowered input voltage range
 * @param[in]  input - Wide Input or Lowered Input
 * @return     none  
 */
inline static void OPA2_InputVoltageRangeSelect(enum OPA_INPUT_VOLTAGE_RANGE input)
{
	AMPCON1Hbits.NCHDIS2 = input;
}

#endif //OPA2_H

/**
 End of File
*/




