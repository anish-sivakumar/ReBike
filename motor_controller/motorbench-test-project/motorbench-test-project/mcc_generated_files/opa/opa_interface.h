/**
 * OPA Generated Driver Interface Header File
 * 
 * @file      opa_interface.h
 * 
 * @defgroup  opadriver OPA Driver
 * 
 * @brief     Operational Amplifier Driver used for a wide variety of purposes, 
 *            including signal conditioning and filtering using dsPIC MCUs.
 *
 * @version   Firmware Driver Version 1.2.1
 *
 * @version   PLIB Version 1.3.0
 *
 * @skipline  Device : dsPIC33CK256MP508
*/

/*disclaimer*/

#ifndef OPA_INTERFACE_H
#define OPA_INTERFACE_H

// Section: Included Files

#include "opa_types.h"

// Section: Data Type Definitions
        
/**
 @ingroup  opadriver
 @struct   OPA_INTERFACE
 @brief    Structure containing the function pointers of OPA driver.
*/
struct OPA_INTERFACE
{   
    void (*Intitialize)(void);      
    ///< Pointer to OPAx_Initialize e.g. \ref OPA1_Initialize
    
    void (*Deintitialize)(void);    
    ///< Pointer to OPAx_Deinitialize e.g. \ref OPA1_Deinitialize
    
    void (*Enable)(void);           
    ///< Pointer to OPAx_Enable e.g. \ref OPA1_Enable
    
    void (*Disable)(void);          
    ///< Pointer to OPAx_Disable e.g. \ref OPA1_Disable
	
	void (*InputVoltageRangeSelect) (enum OPA_INPUT_VOLTAGE_RANGE input);
    ///< Pointer to OPAx_InputVoltageRangeSelect e.g. \ref OPA1_InputVoltageRangeSelect
	
    void (*GainSet)(enum OPA_GAIN gainValue);    
    ///< Pointer to OPAx_GainSet e.g. \ref OPA1_GainSet (This API is hardware dependent)
};

#endif //OPA_INTERFACE_H

