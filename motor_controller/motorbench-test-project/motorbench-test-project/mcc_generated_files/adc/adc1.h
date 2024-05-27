/**
 * ADC1 Generated Driver Header File
 * 
 * @file      adc1.h
 *            
 * @ingroup   adcdriver
 *            
 * @brief     This is the generated driver header file for the ADC1 driver
 *            
 * @version   Firmware Driver Version 1.4.3
 *
 * @version   PLIB Version 2.3.0
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

#ifndef ADC1_H
#define ADC1_H

// Section: Included Files

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>
#include "adc_types.h"
#include "adc_interface.h"
#ifdef __cplusplus  //Provide C++ Compatibility

    extern "C" {

#endif

// Section: Data Types

/** 
  @ingroup  adcdriver
  @brief    Defines the scan option selection done for the shared channels
*/
#define ADC1_SCAN_MODE_SELECTED true

/** 
  @ingroup  adcdriver
  @brief    Defines the ADC Resolution
*/
#define ADC1_RESOLUTION 12

// Section: Data Type Definitions

/**
 * @ingroup  adcdriver
 * @brief    Structure object of type ADC_INTERFACE with the custom name
 *           given by the user in the Melody Driver User interface. The default name 
 *           e.g. ADC1 can be changed by the user in the ADC user interface. 
 *           This allows defining a structure with application specific name using 
 *           the 'Custom Name' field. Application specific name allows the API Portability.
*/
extern const struct ADC_INTERFACE MCC_ADC;

/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_Initialize API
 */
#define MCC_ADC_Initialize ADC1_Initialize
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_Deinitialize API
 */
#define MCC_ADC_Deinitialize ADC1_Deinitialize
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_Enable API
 */
#define MCC_ADC_Enable ADC1_Enable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_Disable API
 */
#define MCC_ADC_Disable ADC1_Disable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_SoftwareTriggerEnable API
 */
#define MCC_ADC_SoftwareTriggerEnable ADC1_SoftwareTriggerEnable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_SoftwareTriggerDisable API
 */
#define MCC_ADC_SoftwareTriggerDisable ADC1_SoftwareTriggerDisable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ChannelSelect API
 */
#define MCC_ADC_ChannelSelect ADC1_ChannelSelect
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ConversionResultGet API
 */
#define MCC_ADC_ConversionResultGet ADC1_ConversionResultGet
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_IsConversionComplete API
 */
#define MCC_ADC_IsConversionComplete ADC1_IsConversionComplete
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ResolutionSet API
 */
#define MCC_ADC_ResolutionSet ADC1_ResolutionSet
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_InterruptEnable API
 */
#define MCC_ADC_InterruptEnable ADC1_InterruptEnable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_InterruptDisable API
 */
#define MCC_ADC_InterruptDisable ADC1_InterruptDisable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_InterruptFlagClear API
 */
#define MCC_ADC_InterruptFlagClear ADC1_InterruptFlagClear
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_InterruptPrioritySet API
 */
#define MCC_ADC_InterruptPrioritySet ADC1_InterruptPrioritySet
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_CommonCallbackRegister API
 */
#define MCC_ADC_CommonCallbackRegister ADC1_CommonCallbackRegister
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_Tasks API
 */
#define MCC_ADC_Tasks ADC1_Tasks
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_IndividualChannelInterruptEnable API
 */
#define MCC_ADC_IndividualChannelInterruptEnable ADC1_IndividualChannelInterruptEnable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_IndividualChannelInterruptDisable API
 */
#define MCC_ADC_IndividualChannelInterruptDisable ADC1_IndividualChannelInterruptDisable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_IndividualChannelInterruptFlagClear API
 */
#define MCC_ADC_IndividualChannelInterruptFlagClear ADC1_IndividualChannelInterruptFlagClear
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_IndividualChannelInterruptPrioritySet API
 */
#define MCC_ADC_IndividualChannelInterruptPrioritySet ADC1_IndividualChannelInterruptPrioritySet
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ChannelCallbackRegister API
 */
#define MCC_ADC_ChannelCallbackRegister ADC1_ChannelCallbackRegister
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ComparatorCallbackRegister API
 */
#define MCC_ADC_ComparatorCallbackRegister ADC1_ComparatorCallbackRegister
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_ChannelTasks API
 */
#define MCC_ADC_ChannelTasks ADC1_ChannelTasks
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_CorePowerEnable API
 */
#define MCC_ADC_CorePowerEnable ADC1_CorePowerEnable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_SharedCorePowerEnable API
 */
#define MCC_ADC_SharedCorePowerEnable ADC1_SharedCorePowerEnable
/**
 * @ingroup  adcdriver
 * @brief    This macro defines the Custom Name for \ref ADC1_PWMTriggerSourceSet API
 */
#define MCC_ADC_PWMTriggerSourceSet ADC1_PWMTriggerSourceSet

// Section: Driver Interface Functions

/**
 * @ingroup  adcdriver
 * @brief    Initializes ADC1 module, using the given initialization data
 *           This function must be called before any other ADC1 function is called
 * @return   none  
 */
void ADC1_Initialize (void);

/**
 * @ingroup  adcdriver
 * @brief    Deinitializes the ADC1 to POR values
 * @return   none  
 */
void ADC1_Deinitialize(void);

/**
 * @ingroup  adcdriver
 * @brief    This inline function enables the ADC1 module
 * @pre      \ref ADC1_Initialize function should have been 
 *           called  before calling this function.
 * @return   none  
 */
inline static void ADC1_Enable(void)
{
   ADCON1Lbits.ADON = 1;
}

/**
 * @ingroup  adcdriver
 * @brief    This inline function disables the ADC1 module
 * @pre      \ref ADC1_Initialize function should have been 
 *           called  before calling this function.
 * @return   none  
 */
inline static void ADC1_Disable(void)
{
   ADCON1Lbits.ADON = 0;
}

/**
 * @ingroup  adcdriver
 * @brief    This inline function sets software common trigger
 * @pre      \ref ADC1_Initialize function should have been 
 *           called before calling this function.
 * @return   none  
 */
inline static void ADC1_SoftwareTriggerEnable(void)
{
   ADCON3Lbits.SWCTRG = 1;
}

/**
 * @ingroup  adcdriver
 * @brief    This inline function resets software common trigger
 * @pre      ADC1_Initialize function should have been 
 *           called before calling this function.
 * @return   none  
 */
inline static void ADC1_SoftwareTriggerDisable(void)
{
   ADCON3Lbits.SWCTRG = 0;
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function allows selection of a channel for conversion
 * @pre        \ref ADC1_Initialize() function should have been
 *             called before calling this function.
 * @param[in]  channel - Channel for conversion  
 * @return     none  
 */
inline static void ADC1_ChannelSelect( enum ADC_CHANNEL channel )
{
    //This function does not have any implementation since 
    //Shared channels are selected from UI.
    //Dedicated channels are selected from UI.
    
    (void)channel;
}

/**
 * @ingroup    adcdriver
 * @brief      Returns the conversion value for the channel selected
 * @pre        This inline function returns the conversion value only after the conversion is complete. 
 *             Conversion completion status can be checked using 
 *             \ref ADC1_IsConversionComplete(channel) function.
 * @param[in]  channel - Selected channel  
 * @return     Returns the analog to digital converted value  
 */
inline static uint16_t ADC1_ConversionResultGet( enum ADC_CHANNEL channel )
{
    uint16_t result = 0x0U;

    switch(channel)
    {
        case MCAF_ADC_DCLINK_CURRENT:
                result = ADCBUF4;
                break;
        case MCAF_ADC_PHASEC_CURRENT:
                result = ADCBUF10;
                break;
        case MCAF_ADC_POTENTIOMETER:
                result = ADCBUF11;
                break;
        case MCAF_ADC_BRIDGE_TEMPERATURE:
                result = ADCBUF12;
                break;
        case MCAF_ADC_DCLINK_VOLTAGE:
                result = ADCBUF15;
                break;
        case MCAF_ADC_PHASEA_VOLTAGE:
                result = ADCBUF17;
                break;
        case MCAF_ADC_PHASEC_VOLTAGE:
                result = ADCBUF22;
                break;
        case MCAF_ADC_PHASEB_VOLTAGE:
                result = ADCBUF23;
                break;
        case MCAF_ADC_PHASEA_CURRENT:
                result = ADCBUF0;
                break;
        case MCAF_ADC_PHASEB_CURRENT:
                result = ADCBUF1;
                break;
        default:
                break;
    }
    return result;
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function returns the status of conversion.This function is used to 
 *             determine if conversion is completed. When conversion is complete 
 *             the function returns true otherwise false.
 * 
 * @pre        \ref ADC1_SoftwareTriggerEnable() function should have been 
 *             called before calling this function.
 * @param[in]  channel - Selected channel  
 * @return     true - Conversion is complete.
 * @return     false - Conversion is not complete.  
 */
inline static bool ADC1_IsConversionComplete(enum ADC_CHANNEL channel)
{
    bool status = false;

    switch(channel)
    {
        case MCAF_ADC_DCLINK_CURRENT:
                status = ADSTATLbits.AN4RDY;
                break;
        case MCAF_ADC_PHASEC_CURRENT:
                status = ADSTATLbits.AN10RDY;
                break;
        case MCAF_ADC_POTENTIOMETER:
                status = ADSTATLbits.AN11RDY;
                break;
        case MCAF_ADC_BRIDGE_TEMPERATURE:
                status = ADSTATLbits.AN12RDY;
                break;
        case MCAF_ADC_DCLINK_VOLTAGE:
                status = ADSTATLbits.AN15RDY;
                break;
        case MCAF_ADC_PHASEA_VOLTAGE:
                status = ADSTATHbits.AN17RDY;
                break;
        case MCAF_ADC_PHASEC_VOLTAGE:
                status = ADSTATHbits.AN22RDY;
                break;
        case MCAF_ADC_PHASEB_VOLTAGE:
                status = ADSTATHbits.AN23RDY;
                break;
        case MCAF_ADC_PHASEA_CURRENT:
                status = ADSTATLbits.AN0RDY;
                break;
        case MCAF_ADC_PHASEB_CURRENT:
                status = ADSTATLbits.AN1RDY;
                break;
        default:
                break;
    }

    return status;
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function helps to configure all cores with same resolution
 * 
 * @param[in]  resolution - Resolution type  
 * @return     none 
 */
inline static void ADC1_ResolutionSet(enum ADC_RESOLUTION_TYPE resolution)
{
   ADCORE0Hbits.RES = resolution;
   ADCORE1Hbits.RES = resolution;
   ADCON1Hbits.SHRRES = resolution;
}

/**
 * @ingroup  adcdriver
 * @brief    This inline function enables the ADC1 interrupt
 * @return   none  
 */
inline static void ADC1_InterruptEnable(void)
{
    IEC5bits.ADCIE = 1;
}

/**
 * @ingroup  adcdriver
 * @brief    This inline function disables the ADC1 interrupt
 * @return   none  
 */
inline static void ADC1_InterruptDisable(void)
{
    IEC5bits.ADCIE = 0;
}

/**
 * @ingroup  adcdriver
 * @brief    Clears interrupt flag manually
 * @return   none  
 */
inline static void ADC1_InterruptFlagClear(void)
{
    IFS5bits.ADCIF = 0;
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function allows selection of priority for interrupt
 * @param[in]  priorityValue  -  The numerical value of interrupt priority
 * @return     none  
 */
inline static void ADC1_InterruptPrioritySet( uint16_t priorityValue )
{
    IPC22bits.ADCIP = (uint16_t)0x7 & priorityValue;
}

/**
 * @ingroup    adcdriver
 * @brief      This function can be used to override default callback and to 
 *             define custom callback for ADC1 Common event
 * @param[in]  callback - Address of the callback function.  
 * @return     none  
 *             
 */
void ADC1_CommonCallbackRegister(void(*callback)(void));

/**
 * @ingroup  adcdriver
 * @brief    This is the default callback with weak attribute. The user can override and implement the default callback without weak attribute
 *           or can register a custom callback function using  ADC1_CommonCallbackRegister
 * @return   none  
 */
void ADC1_CommonCallback(void);

 
/**
 * @ingroup  adcdriver
 * @brief    This function is used to implement the tasks for polled implementations
 * @pre      \ref ADC1_Initialize() function should have been
 *           called before calling this function.
 * @return   none
 * @note     This function has to be polled to notify callbacks and clear 
 *           the interrupt flags in non-interrupt mode of ADC
 */
void ADC1_Tasks(void);

/**
 * @ingroup    adcdriver
 * @brief      This inline function enables individual channel interrupt
 * @param[in]  channel - Selected channel  
 * @return     none  
 */
inline static void ADC1_IndividualChannelInterruptEnable(enum ADC_CHANNEL channel)
{
    switch(channel)
    {
        case MCAF_ADC_DCLINK_CURRENT:
                IEC5bits.ADCAN4IE = 1;
                ADIELbits.IE4 = 1;
                break;
        case MCAF_ADC_PHASEC_CURRENT:
                IEC6bits.ADCAN10IE = 1;
                ADIELbits.IE10 = 1;
                break;
        case MCAF_ADC_POTENTIOMETER:
                IEC6bits.ADCAN11IE = 1;
                ADIELbits.IE11 = 1;
                break;
        case MCAF_ADC_BRIDGE_TEMPERATURE:
                IEC6bits.ADCAN12IE = 1;
                ADIELbits.IE12 = 1;
                break;
        case MCAF_ADC_DCLINK_VOLTAGE:
                IEC6bits.ADCAN15IE = 1;
                ADIELbits.IE15 = 1;
                break;
        case MCAF_ADC_PHASEA_VOLTAGE:
                IEC6bits.ADCAN17IE = 1;
                ADIEHbits.IE17 = 1;
                break;
        case MCAF_ADC_PHASEC_VOLTAGE:
                IEC7bits.ADCAN22IE = 1;
                ADIEHbits.IE22 = 1;
                break;
        case MCAF_ADC_PHASEB_VOLTAGE:
                IEC7bits.ADCAN23IE = 1;
                ADIEHbits.IE23 = 1;
                break;
        case MCAF_ADC_PHASEA_CURRENT:
                IEC5bits.ADCAN0IE = 1;
                ADIELbits.IE0 = 1;
                break;
        case MCAF_ADC_PHASEB_CURRENT:
                IEC5bits.ADCAN1IE = 1;
                ADIELbits.IE1 = 1;
                break;
        default:
                break;
    }
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function disables individual channel interrupt
 * @param[in]  channel - Selected channel  
 * @return     none  
 */
inline static void ADC1_IndividualChannelInterruptDisable(enum ADC_CHANNEL channel)
{
    switch(channel)
    {
        case MCAF_ADC_DCLINK_CURRENT:
                IEC5bits.ADCAN4IE = 0;
                ADIELbits.IE4 = 0;
                break;
        case MCAF_ADC_PHASEC_CURRENT:
                IEC6bits.ADCAN10IE = 0;
                ADIELbits.IE10 = 0;
                break;
        case MCAF_ADC_POTENTIOMETER:
                IEC6bits.ADCAN11IE = 0;
                ADIELbits.IE11 = 0;
                break;
        case MCAF_ADC_BRIDGE_TEMPERATURE:
                IEC6bits.ADCAN12IE = 0;
                ADIELbits.IE12 = 0;
                break;
        case MCAF_ADC_DCLINK_VOLTAGE:
                IEC6bits.ADCAN15IE = 0;
                ADIELbits.IE15 = 0;
                break;
        case MCAF_ADC_PHASEA_VOLTAGE:
                IEC6bits.ADCAN17IE = 0;
                ADIEHbits.IE17 = 0;
                break;
        case MCAF_ADC_PHASEC_VOLTAGE:
                IEC7bits.ADCAN22IE = 0;
                ADIEHbits.IE22 = 0;
                break;
        case MCAF_ADC_PHASEB_VOLTAGE:
                IEC7bits.ADCAN23IE = 0;
                ADIEHbits.IE23 = 0;
                break;
        case MCAF_ADC_PHASEA_CURRENT:
                IEC5bits.ADCAN0IE = 0;
                ADIELbits.IE0 = 0;
                break;
        case MCAF_ADC_PHASEB_CURRENT:
                IEC5bits.ADCAN1IE = 0;
                ADIELbits.IE1 = 0;
                break;
        default:
                break;
    }
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function clears individual channel interrupt flag
 * @param[in]  channel - Selected channel  
 * @return     none  
 */
inline static void ADC1_IndividualChannelInterruptFlagClear(enum ADC_CHANNEL channel)
{
    switch(channel)
    {
        case MCAF_ADC_DCLINK_CURRENT:
                IFS5bits.ADCAN4IF = 0;
                break;
        case MCAF_ADC_PHASEC_CURRENT:
                IFS6bits.ADCAN10IF = 0;
                break;
        case MCAF_ADC_POTENTIOMETER:
                IFS6bits.ADCAN11IF = 0;
                break;
        case MCAF_ADC_BRIDGE_TEMPERATURE:
                IFS6bits.ADCAN12IF = 0;
                break;
        case MCAF_ADC_DCLINK_VOLTAGE:
                IFS6bits.ADCAN15IF = 0;
                break;
        case MCAF_ADC_PHASEA_VOLTAGE:
                IFS6bits.ADCAN17IF = 0;
                break;
        case MCAF_ADC_PHASEC_VOLTAGE:
                IFS7bits.ADCAN22IF = 0;
                break;
        case MCAF_ADC_PHASEB_VOLTAGE:
                IFS7bits.ADCAN23IF = 0;
                break;
        case MCAF_ADC_PHASEA_CURRENT:
                IFS5bits.ADCAN0IF = 0;
                break;
        case MCAF_ADC_PHASEB_CURRENT:
                IFS5bits.ADCAN1IF = 0;
                break;
        default:
                break;
    }
}

/**
 * @ingroup    adcdriver
 * @brief      This inline function allows selection of priority for individual channel interrupt
 * @param[in]  channel - Selected channel 
 * @param[in]  priorityValue  -  The numerical value of interrupt priority
 * @return     none  
 */
inline static void ADC1_IndividualChannelInterruptPrioritySet(enum ADC_CHANNEL channel, enum INTERRUPT_PRIORITY priorityValue)
{
	switch(channel)
	{
		case MCAF_ADC_DCLINK_CURRENT:
				IPC23bits.ADCAN4IP = priorityValue;
				break;
		case MCAF_ADC_PHASEC_CURRENT:
				IPC25bits.ADCAN10IP = priorityValue;
				break;
		case MCAF_ADC_POTENTIOMETER:
				IPC25bits.ADCAN11IP = priorityValue;
				break;
		case MCAF_ADC_BRIDGE_TEMPERATURE:
				IPC25bits.ADCAN12IP = priorityValue;
				break;
		case MCAF_ADC_DCLINK_VOLTAGE:
				IPC26bits.ADCAN15IP = priorityValue;
				break;
		case MCAF_ADC_PHASEA_VOLTAGE:
				IPC27bits.ADCAN17IP = priorityValue;
				break;
		case MCAF_ADC_PHASEC_VOLTAGE:
				IPC28bits.ADCAN22IP = priorityValue;
				break;
		case MCAF_ADC_PHASEB_VOLTAGE:
				IPC28bits.ADCAN23IP = priorityValue;
				break;
		case MCAF_ADC_PHASEA_CURRENT:
				IPC22bits.ADCAN0IP = priorityValue;
				break;
		case MCAF_ADC_PHASEB_CURRENT:
				IPC23bits.ADCAN1IP = priorityValue;
				break;
		default:
				break;
	}
}

/**
 * @ingroup    adcdriver
 * @brief      This function can be used to override default callback \ref ADC1_ChannelCallback
 *             and to define custom callback for ADC1 Channel event. 
 *             Read the conversion result of the corresponding channel in the custom callback.
 * @param[in]  callback - Address of the callback function.  
 * @return     none  
 */
void ADC1_ChannelCallbackRegister(void(*callback)(enum ADC_CHANNEL channel, uint16_t adcVal));

/**
 * @ingroup    adcdriver
 * @brief      This is the default callback function for all the analog channels. 
 *             This callback is triggered once the channel conversion is done for a
 *             channel and to read the conversion result of the corresponding channel
 * @param[in]  channel - conversion completed channel
 * @param[in]  adcVal - conversion result of channel  
 * @return     none  
 */
void ADC1_ChannelCallback(enum ADC_CHANNEL channel, uint16_t adcVal);


/**
 * @ingroup    adcdriver
 * @brief      This function can be used to override default callback and to 
 *             define custom callback for ADC1_Comparator event
 * @param[in]  callback - Address of the callback function.  
 * @return     none  
 */
void ADC1_ComparatorCallbackRegister(void(*callback)(enum ADC_CMP comparator));

/**
 * @ingroup    adcdriver
 * @brief      Comparator callback function
 * @param[in]  comparator - comparator in which compare event occurred  
 * @return     none  
 */
void ADC1_ComparatorCallback(enum ADC_CMP comparator);


/**
 * @ingroup    adcdriver
 * @brief      This function call used only in polling mode, if channel 
 *             conversion is done for requested channel, the calls the 
 *             respective callback function
 * @pre        \ref ADC1_Initialize() function should have been  
 *             called before calling this function.
 * @param[in]  channel - Selected channel.  
 * @return     none  
 * @note       This function has to be polled to notify channel callbacks and clear 
 *             the channel interrupt flags in non-interrupt mode of ADC
 */
void ADC1_ChannelTasks(enum ADC_CHANNEL channel);


// Section: Interface functions: Dedicated Core

/**
 * @ingroup    adcdriver
 * @brief      Enables analog and digital power for ADC1 dedicated core
 * @param[in]  core - Selected core  
 * @return     none  
 */
void ADC1_CorePowerEnable(enum ADC_DEDICATED_CORE core);


/**
 * @ingroup  adcdriver
 * @brief    Enables power for ADC1 shared Core
 *           This function is used to set the analog and digital power for 
 *           ADC1 shared Core.
 * @return   none  
 */
void ADC1_SharedCorePowerEnable(void);


/**
 * @ingroup  adcdriver
 * @brief    Sets PWM trigger source for corresponding analog input 
 * @param[in]  channel - Selected channel  
 * @param[in]  pwmInstance - PWM instance for the trigger source
 * @param[in]  triggerNumber - 1, for PWMx Trigger 1
 * @param[in]  triggerNumber - 2, for PWMx Trigger 2
 * @return   none  
 * @note     Configure PWM trigger value using \ref PWM_TriggerACompareValueSet, \ref PWM_TriggerBCompareValueSet
 *           or \ref PWM_TriggerCCompareValueSet before calling this funcion and enable corresponding 
 *           PWM trigger using \ref PWM_Trigger1Enable or \ref PWM_Trigger2Enable post calling it.
 */
void ADC1_PWMTriggerSourceSet(enum ADC_CHANNEL channel, enum ADC_PWM_INSTANCE pwmInstance, enum ADC_PWM_TRIGGERS triggerNumber);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif //_ADC1_H
    
/**
 End of File
*/

