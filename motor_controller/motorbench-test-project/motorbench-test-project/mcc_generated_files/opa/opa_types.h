/**
 * OPA Generated Driver Interface Header File
 * 
 * @file      opa_interface.h
 * 
 * @ingroup   opadriver
 * 
 * @brief     This is the generated driver interface header file for the OPA driver
 *
 * @version   Firmware Driver Version 1.2.1
 *
 * @version   PLIB Version 1.3.0
 *
 * @skipline  Device : dsPIC33CK256MP508
*/

/*disclaimer*/

#ifndef OPA_TYPES_H
#define    OPA_TYPES_H

// Section: Type defines
    
/**
 @ingroup  opadriver
 @enum     OPA_GAIN
 @brief    This defines the gain enumeration for OPA
*/
enum OPA_GAIN{

     OPA_GAIN_DISABLED         /**< OPA Gain is disabled */

};

/**
 @ingroup  opadriver
 @enum     OPA_INPUT_VOLTAGE_RANGE
 @brief    This defines input voltage range enumeration for OPA channels
*/
enum OPA_INPUT_VOLTAGE_RANGE
{ 
	OPA_WIDE_INPUT 	   = 0,    /**< Wide Input voltage range selected */
	OPA_LOWERED_INPUT  = 1     /**< Lowered Input voltage range selected */
}; 
 
#endif  //OPA_TYPES_H
/**
 End of File 
*/
