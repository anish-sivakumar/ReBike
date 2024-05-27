/**
 * OPA1 Generated Driver Header File
 * 
 * @file      opa1.h
 * 
 * @ingroup   opadriver
 * 
 * @brief     This is the generated driver header file for the OPA1 driver
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

#ifndef OPA1_H
#define OPA1_H

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
 *           e.g. OPA1 can be changed by the user in the OPA user interface. 
 *           This allows defining a structure with application specific name using 
 *           the 'Custom Name' field. Application specific name allows the API Portability.
*/
 extern const struct OPA_INTERFACE MCC_OPA_IA;

/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA1_Initialize API
 */
#define MCC_OPA_IA_Initialize OPA1_Initialize
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA1_Deinitialize API
 */
#define MCC_OPA_IA_Deinitialize OPA1_Deinitialize
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA1_Enable API
 */
#define MCC_OPA_IA_Enable OPA1_Enable
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA1_Disable API
 */
#define MCC_OPA_IA_Disable OPA1_Disable
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA1_InputVoltageRangeSelect API
 */
#define MCC_OPA_IA_InputVoltageRangeSelect OPA1_InputVoltageRangeSelect

// Section: Interface Routines

/**
 * @ingroup  opadriver
 * @brief    Initializes the OPA1 module
 * @return   none  
 */
void OPA1_Initialize (void);

/**
 * @ingroup  opadriver
 * @brief    Deinitializes the OPA1 to POR values
 * @return   none  
 */
void OPA1_Deinitialize(void);

/**
 * @ingroup  opadriver
 * @brief    This inline function enables the OPA1 module
 * @pre      The OPA1_Initialize function should be called for the specified 
 *           OPA1 driver instance.
 * @return   none  
 */
inline static void OPA1_Enable( void )
{
	AMPCON1Lbits.AMPEN1 = 1; //Enable opa1;
}

/**
 * @ingroup  opadriver
 * @brief    This inline function disables the OPA1 module
 * @return   none  
 */
inline static void OPA1_Disable( void )
{
	AMPCON1Lbits.AMPEN1 = 0; //Disable opa1;
}

/**
 * @ingroup    opadriver
 * @brief      This inline function selects OPA1 Wide Input or Lowered input voltage range
 * @param[in]  input - Wide Input or Lowered Input
 * @return     none  
 */
inline static void OPA1_InputVoltageRangeSelect(enum OPA_INPUT_VOLTAGE_RANGE input)
{
	AMPCON1Hbits.NCHDIS1 = input;
}

#endif //OPA1_H

/**
 End of File
*/




