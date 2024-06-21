/**
 * PINS Generated Driver Header File 
 * 
 * @file      pins.h
 *            
 * @defgroup  pinsdriver Pins Driver
 *            
 * @brief     The Pin Driver directs the operation and function of 
 *            the selected device pins using dsPIC MCUs.
 *
 * @version   Firmware Driver Version 1.0.1
 *
 * @version   PLIB Version 1.1.0
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

#ifndef PINS_H
#define PINS_H
// Section: Includes
#include <xc.h>

// Section: Device Pin Macros

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RB10 GPIO Pin which has a custom name of SPI1_CS to High
 * @pre      The RB10 must be set as Output Pin             
 * @return   none  
 */
#define SPI1_CS_SetHigh()          (_LATB10 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RB10 GPIO Pin which has a custom name of SPI1_CS to Low
 * @pre      The RB10 must be set as Output Pin
 * @return   none  
 */
#define SPI1_CS_SetLow()           (_LATB10 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RB10 GPIO Pin which has a custom name of SPI1_CS
 * @pre      The RB10 must be set as Output Pin
 * @return   none  
 */
#define SPI1_CS_Toggle()           (_LATB10 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RB10 GPIO Pin which has a custom name of SPI1_CS
 * @return   none  
 */
#define SPI1_CS_GetValue()         _RB10

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RB10 GPIO Pin which has a custom name of SPI1_CS as Input
 * @return   none  
 */
#define SPI1_CS_SetDigitalInput()  (_TRISB10 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RB10 GPIO Pin which has a custom name of SPI1_CS as Output
 * @return   none  
 */
#define SPI1_CS_SetDigitalOutput() (_TRISB10 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE4 GPIO Pin which has a custom name of MCAF_TESTPOINT1 to High
 * @pre      The RE4 must be set as Output Pin             
 * @return   none  
 */
#define MCAF_TESTPOINT1_SetHigh()          (_LATE4 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE4 GPIO Pin which has a custom name of MCAF_TESTPOINT1 to Low
 * @pre      The RE4 must be set as Output Pin
 * @return   none  
 */
#define MCAF_TESTPOINT1_SetLow()           (_LATE4 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE4 GPIO Pin which has a custom name of MCAF_TESTPOINT1
 * @pre      The RE4 must be set as Output Pin
 * @return   none  
 */
#define MCAF_TESTPOINT1_Toggle()           (_LATE4 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE4 GPIO Pin which has a custom name of MCAF_TESTPOINT1
 * @return   none  
 */
#define MCAF_TESTPOINT1_GetValue()         _RE4

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE4 GPIO Pin which has a custom name of MCAF_TESTPOINT1 as Input
 * @return   none  
 */
#define MCAF_TESTPOINT1_SetDigitalInput()  (_TRISE4 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE4 GPIO Pin which has a custom name of MCAF_TESTPOINT1 as Output
 * @return   none  
 */
#define MCAF_TESTPOINT1_SetDigitalOutput() (_TRISE4 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE6 GPIO Pin which has a custom name of MCAF_LED1 to High
 * @pre      The RE6 must be set as Output Pin             
 * @return   none  
 */
#define MCAF_LED1_SetHigh()          (_LATE6 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE6 GPIO Pin which has a custom name of MCAF_LED1 to Low
 * @pre      The RE6 must be set as Output Pin
 * @return   none  
 */
#define MCAF_LED1_SetLow()           (_LATE6 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE6 GPIO Pin which has a custom name of MCAF_LED1
 * @pre      The RE6 must be set as Output Pin
 * @return   none  
 */
#define MCAF_LED1_Toggle()           (_LATE6 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE6 GPIO Pin which has a custom name of MCAF_LED1
 * @return   none  
 */
#define MCAF_LED1_GetValue()         _RE6

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE6 GPIO Pin which has a custom name of MCAF_LED1 as Input
 * @return   none  
 */
#define MCAF_LED1_SetDigitalInput()  (_TRISE6 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE6 GPIO Pin which has a custom name of MCAF_LED1 as Output
 * @return   none  
 */
#define MCAF_LED1_SetDigitalOutput() (_TRISE6 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE7 GPIO Pin which has a custom name of MCAF_LED2 to High
 * @pre      The RE7 must be set as Output Pin             
 * @return   none  
 */
#define MCAF_LED2_SetHigh()          (_LATE7 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE7 GPIO Pin which has a custom name of MCAF_LED2 to Low
 * @pre      The RE7 must be set as Output Pin
 * @return   none  
 */
#define MCAF_LED2_SetLow()           (_LATE7 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE7 GPIO Pin which has a custom name of MCAF_LED2
 * @pre      The RE7 must be set as Output Pin
 * @return   none  
 */
#define MCAF_LED2_Toggle()           (_LATE7 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE7 GPIO Pin which has a custom name of MCAF_LED2
 * @return   none  
 */
#define MCAF_LED2_GetValue()         _RE7

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE7 GPIO Pin which has a custom name of MCAF_LED2 as Input
 * @return   none  
 */
#define MCAF_LED2_SetDigitalInput()  (_TRISE7 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE7 GPIO Pin which has a custom name of MCAF_LED2 as Output
 * @return   none  
 */
#define MCAF_LED2_SetDigitalOutput() (_TRISE7 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE8 GPIO Pin which has a custom name of IO_RE8 to High
 * @pre      The RE8 must be set as Output Pin             
 * @return   none  
 */
#define IO_RE8_SetHigh()          (_LATE8 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE8 GPIO Pin which has a custom name of IO_RE8 to Low
 * @pre      The RE8 must be set as Output Pin
 * @return   none  
 */
#define IO_RE8_SetLow()           (_LATE8 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE8 GPIO Pin which has a custom name of IO_RE8
 * @pre      The RE8 must be set as Output Pin
 * @return   none  
 */
#define IO_RE8_Toggle()           (_LATE8 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE8 GPIO Pin which has a custom name of IO_RE8
 * @return   none  
 */
#define IO_RE8_GetValue()         _RE8

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE8 GPIO Pin which has a custom name of IO_RE8 as Input
 * @return   none  
 */
#define IO_RE8_SetDigitalInput()  (_TRISE8 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE8 GPIO Pin which has a custom name of IO_RE8 as Output
 * @return   none  
 */
#define IO_RE8_SetDigitalOutput() (_TRISE8 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE9 GPIO Pin which has a custom name of IO_RE9 to High
 * @pre      The RE9 must be set as Output Pin             
 * @return   none  
 */
#define IO_RE9_SetHigh()          (_LATE9 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE9 GPIO Pin which has a custom name of IO_RE9 to Low
 * @pre      The RE9 must be set as Output Pin
 * @return   none  
 */
#define IO_RE9_SetLow()           (_LATE9 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE9 GPIO Pin which has a custom name of IO_RE9
 * @pre      The RE9 must be set as Output Pin
 * @return   none  
 */
#define IO_RE9_Toggle()           (_LATE9 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE9 GPIO Pin which has a custom name of IO_RE9
 * @return   none  
 */
#define IO_RE9_GetValue()         _RE9

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE9 GPIO Pin which has a custom name of IO_RE9 as Input
 * @return   none  
 */
#define IO_RE9_SetDigitalInput()  (_TRISE9 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE9 GPIO Pin which has a custom name of IO_RE9 as Output
 * @return   none  
 */
#define IO_RE9_SetDigitalOutput() (_TRISE9 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE10 GPIO Pin which has a custom name of IO_RE10 to High
 * @pre      The RE10 must be set as Output Pin             
 * @return   none  
 */
#define IO_RE10_SetHigh()          (_LATE10 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE10 GPIO Pin which has a custom name of IO_RE10 to Low
 * @pre      The RE10 must be set as Output Pin
 * @return   none  
 */
#define IO_RE10_SetLow()           (_LATE10 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE10 GPIO Pin which has a custom name of IO_RE10
 * @pre      The RE10 must be set as Output Pin
 * @return   none  
 */
#define IO_RE10_Toggle()           (_LATE10 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE10 GPIO Pin which has a custom name of IO_RE10
 * @return   none  
 */
#define IO_RE10_GetValue()         _RE10

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE10 GPIO Pin which has a custom name of IO_RE10 as Input
 * @return   none  
 */
#define IO_RE10_SetDigitalInput()  (_TRISE10 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE10 GPIO Pin which has a custom name of IO_RE10 as Output
 * @return   none  
 */
#define IO_RE10_SetDigitalOutput() (_TRISE10 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE11 GPIO Pin which has a custom name of MCAF_BUTTON1 to High
 * @pre      The RE11 must be set as Output Pin             
 * @return   none  
 */
#define MCAF_BUTTON1_SetHigh()          (_LATE11 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE11 GPIO Pin which has a custom name of MCAF_BUTTON1 to Low
 * @pre      The RE11 must be set as Output Pin
 * @return   none  
 */
#define MCAF_BUTTON1_SetLow()           (_LATE11 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE11 GPIO Pin which has a custom name of MCAF_BUTTON1
 * @pre      The RE11 must be set as Output Pin
 * @return   none  
 */
#define MCAF_BUTTON1_Toggle()           (_LATE11 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE11 GPIO Pin which has a custom name of MCAF_BUTTON1
 * @return   none  
 */
#define MCAF_BUTTON1_GetValue()         _RE11

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE11 GPIO Pin which has a custom name of MCAF_BUTTON1 as Input
 * @return   none  
 */
#define MCAF_BUTTON1_SetDigitalInput()  (_TRISE11 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE11 GPIO Pin which has a custom name of MCAF_BUTTON1 as Output
 * @return   none  
 */
#define MCAF_BUTTON1_SetDigitalOutput() (_TRISE11 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE12 GPIO Pin which has a custom name of MCAF_BUTTON2 to High
 * @pre      The RE12 must be set as Output Pin             
 * @return   none  
 */
#define MCAF_BUTTON2_SetHigh()          (_LATE12 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Sets the RE12 GPIO Pin which has a custom name of MCAF_BUTTON2 to Low
 * @pre      The RE12 must be set as Output Pin
 * @return   none  
 */
#define MCAF_BUTTON2_SetLow()           (_LATE12 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Toggles the RE12 GPIO Pin which has a custom name of MCAF_BUTTON2
 * @pre      The RE12 must be set as Output Pin
 * @return   none  
 */
#define MCAF_BUTTON2_Toggle()           (_LATE12 ^= 1)

/**
 * @ingroup  pinsdriver
 * @brief    Reads the value of the RE12 GPIO Pin which has a custom name of MCAF_BUTTON2
 * @return   none  
 */
#define MCAF_BUTTON2_GetValue()         _RE12

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE12 GPIO Pin which has a custom name of MCAF_BUTTON2 as Input
 * @return   none  
 */
#define MCAF_BUTTON2_SetDigitalInput()  (_TRISE12 = 1)

/**
 * @ingroup  pinsdriver
 * @brief    Configures the RE12 GPIO Pin which has a custom name of MCAF_BUTTON2 as Output
 * @return   none  
 */
#define MCAF_BUTTON2_SetDigitalOutput() (_TRISE12 = 0)

/**
 * @ingroup  pinsdriver
 * @brief    Initializes the PINS module
 * @return   none  
 */
void PINS_Initialize(void);

/**
 * @ingroup  pinsdriver
 * @brief    This function is callback for IO_RE10 Pin
 * @return   none   
 */
void IO_RE10_CallBack(void);

/**
 * @ingroup  pinsdriver
 * @brief    This function is callback for IO_RE8 Pin
 * @return   none   
 */
void IO_RE8_CallBack(void);

/**
 * @ingroup  pinsdriver
 * @brief    This function is callback for IO_RE9 Pin
 * @return   none   
 */
void IO_RE9_CallBack(void);


/**
 * @ingroup    pinsdriver
 * @brief      This function assigns a function pointer with a callback address
 * @param[in]  InterruptHandler - Address of the callback function 
 * @return     none  
 */
void IO_RE10_SetInterruptHandler(void (* InterruptHandler)(void));

/**
 * @ingroup    pinsdriver
 * @brief      This function assigns a function pointer with a callback address
 * @param[in]  InterruptHandler - Address of the callback function 
 * @return     none  
 */
void IO_RE8_SetInterruptHandler(void (* InterruptHandler)(void));

/**
 * @ingroup    pinsdriver
 * @brief      This function assigns a function pointer with a callback address
 * @param[in]  InterruptHandler - Address of the callback function 
 * @return     none  
 */
void IO_RE9_SetInterruptHandler(void (* InterruptHandler)(void));


#endif
