/**
 * OPA2 Generated Driver Source File
 * 
 * @file      opa2.c
 * 
 * @ingroup   opadriver
 * 
 * @brief     This is the generated driver source file for OPA2 driver
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

// Section: Included Files

#include <stdlib.h>
#include "../opa2.h"

// Section: File specific functions
// Section: File specific data type definitions

/**
 * @ingroup  opadriver
 * @brief    Defines an object for OPA_INTERFACE.
 *           For hardware dependent APIs, make sure the API is not NULL before calling it.  
 * 
 * @b Example:
 * @code
 *      if(customerName.OPA_GainSet != NULL)
 *            customerName.OPA_GainSet();
 * @endcode
 */
const struct OPA_INTERFACE MCC_OPA_IB = {
    .Intitialize        	 = &OPA2_Initialize,
    .Deintitialize      	 = &OPA2_Deinitialize,
    .Enable           		 = &OPA2_Enable,
    .Disable            	 = &OPA2_Disable,
    /**< Hardware dependent APIs */
	.InputVoltageRangeSelect = &OPA2_InputVoltageRangeSelect,
    .GainSet            	 = NULL,             
};
// Section: Driver Interface Function Definitions

void OPA2_Initialize (void)
{

    AMPCON1Lbits.AMPON = 1; // Enable the OPA module
    OPA2_InputVoltageRangeSelect( OPA_WIDE_INPUT ); // wide input range
    OPA2_Enable();
    
}

void OPA2_Deinitialize (void)
{
    AMPCON1Lbits.AMPON = 0; 

    OPA2_Disable(); 
}

/**
 End of File
*/
