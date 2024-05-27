/**
 * OPA3 Generated Driver Header File
 * 
 * @file      opa3.h
 * 
 * @ingroup   opadriver
 * 
 * @brief     This is the generated driver header file for the OPA3 driver
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

#ifndef OPA3_H
#define OPA3_H

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
 *           e.g. OPA3 can be changed by the user in the OPA user interface. 
 *           This allows defining a structure with application specific name using 
 *           the 'Custom Name' field. Application specific name allows the API Portability.
*/
extern const struct OPA_INTERFACE MCC_OPA_IDC;

/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA3_Initialize API
 */
#define MCC_OPA_IDC_Initialize OPA3_Initialize
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA3_Deinitialize API
 */
#define MCC_OPA_IDC_Deinitialize OPA3_Deinitialize
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA3_Enable API
 */
#define MCC_OPA_IDC_Enable OPA3_Enable
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA3_Disable API
 */
#define MCC_OPA_IDC_Disable OPA3_Disable
/**
 * @ingroup  opadriver
 * @brief    This macro defines the Custom Name for \ref OPA3_InputVoltageRangeSelect API
 */
#define MCC_OPA_IDC_InputVoltageRangeSelect OPA3_InputVoltageRangeSelect


// Section: Interface Routines

/**
 * @ingroup  opadriver
 * @brief    Initializes OPA3 module
 * @return   none  
 */
void OPA3_Initialize (void);

/**
 * @ingroup  opadriver
 * @brief    Deinitializes the OPA3 to POR values
 * @return   none  
 */
void OPA3_Deinitialize(void);

/**
 * @ingroup  opadriver
 * @brief    This inline function enables the OPA3 module
 * @pre      The OPA3_Initialize function should be called for the specified 
 *           OPA3 driver instance.
 * @return   none  
 */
inline static void OPA3_Enable( void )
{
     AMPCON1Lbits.AMPEN3 = 1; //Enable opa3;
}

/**
 * @ingroup  opadriver
 * @brief    This inline function disables the OPA3 module
 * @return   none  
 */
inline static void OPA3_Disable( void )
{
     AMPCON1Lbits.AMPEN3 = 0; //Disable opa3;
}

/**
 * @ingroup    opadriver
 * @brief      This inline function selects OPA2 Wide Input or Lowered input voltage range
 * @param[in]  input - Wide Input or Lowered Input
 * @return     none  
 */
inline static void OPA3_InputVoltageRangeSelect(enum OPA_INPUT_VOLTAGE_RANGE input)
{
	AMPCON1Hbits.NCHDIS3 = input;
}

#endif //OPA3_H

/**
 End of File
*/




