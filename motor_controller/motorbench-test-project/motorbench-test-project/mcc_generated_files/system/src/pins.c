/**
 * PINS Generated Driver Source File 
 * 
 * @file      pins.c
 *            
 * @ingroup   pinsdriver
 *            
 * @brief     This is the generated driver source file for PINS driver.
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

// Section: Includes
#include <xc.h>
#include <stddef.h>
#include "../pins.h"

// Section: File specific functions
static void (*IO_RE10_InterruptHandler)(void) = NULL;
static void (*IO_RE8_InterruptHandler)(void) = NULL;
static void (*IO_RE9_InterruptHandler)(void) = NULL;

// Section: Driver Interface Function Definitions
void PINS_Initialize(void)
{
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATA = 0x0000;
    LATB = 0x0000;
    LATC = 0x0000;
    LATD = 0x0000;
    LATE = 0x0000;

    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISA = 0x001F;
    TRISB = 0x0FFF;
    TRISC = 0xFFFF;
    TRISD = 0xDFFC;
    TRISE = 0xFF2F;


    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    CNPUA = 0x0000;
    CNPUB = 0x0000;
    CNPUC = 0x0000;
    CNPUD = 0x0000;
    CNPUE = 0x0700;
    CNPDA = 0x0000;
    CNPDB = 0x0000;
    CNPDC = 0x0000;
    CNPDD = 0x0000;
    CNPDE = 0x0000;


    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0000;
    ODCC = 0x0000;
    ODCD = 0x0000;
    ODCE = 0x0000;


    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    ANSELA = 0x001F;
    ANSELB = 0x039F;
    ANSELC = 0x00CF;
    ANSELD = 0x0C00;
    ANSELE = 0x000F;

    /****************************************************************************
     * Set the PPS
     ***************************************************************************/
     __builtin_write_RPCON(0x0000); // unlock PPS

        RPINR15bits.QEINDX1R = 0x003E; //RC14->QEI1:INDX1;
        RPINR14bits.QEIA1R = 0x003C; //RC12->QEI1:QEA1;
        RPINR14bits.QEIB1R = 0x003D; //RC13->QEI1:QEB1;
        RPINR18bits.U1RXR = 0x004E; //RD14->UART1:U1RX;
        RPOR22bits.RP77R = 0x0001;  //RD13->UART1:U1TX;

     __builtin_write_RPCON(0x0800); // lock PPS

    /*******************************************************************************
    * Interrupt On Change: any
    *******************************************************************************/
    CNEN0Ebits.CNEN0E10 = 1; //Pin : RE10; 
    CNEN1Ebits.CNEN1E10 = 1; //Pin : RE10; 
    /*******************************************************************************
    * Interrupt On Change: any
    *******************************************************************************/
    CNEN0Ebits.CNEN0E8 = 1; //Pin : RE8; 
    CNEN1Ebits.CNEN1E8 = 1; //Pin : RE8; 
    /*******************************************************************************
    * Interrupt On Change: any
    *******************************************************************************/
    CNEN0Ebits.CNEN0E9 = 1; //Pin : RE9; 
    CNEN1Ebits.CNEN1E9 = 1; //Pin : RE9; 

    /****************************************************************************
     * Interrupt On Change: flag
     ***************************************************************************/
    CNFEbits.CNFE10 = 0;    //Pin : IO_RE10
    CNFEbits.CNFE8 = 0;    //Pin : IO_RE8
    CNFEbits.CNFE9 = 0;    //Pin : IO_RE9

    /****************************************************************************
     * Interrupt On Change: config
     ***************************************************************************/
    CNCONEbits.CNSTYLE = 1; //Config for PORTE
    CNCONEbits.ON = 1; //Config for PORTE

    /* Initialize IOC Interrupt Handler*/
    IO_RE10_SetInterruptHandler(&IO_RE10_CallBack);
    IO_RE8_SetInterruptHandler(&IO_RE8_CallBack);
    IO_RE9_SetInterruptHandler(&IO_RE9_CallBack);

    /****************************************************************************
     * Interrupt On Change: Interrupt Enable
     ***************************************************************************/
    IFS4bits.CNEIF = 0; //Clear CNEI interrupt flag
    IEC4bits.CNEIE = 1; //Enable CNEI interrupt
}

void __attribute__ ((weak)) IO_RE10_CallBack(void)
{

}

void __attribute__ ((weak)) IO_RE8_CallBack(void)
{

}

void __attribute__ ((weak)) IO_RE9_CallBack(void)
{

}

void IO_RE10_SetInterruptHandler(void (* InterruptHandler)(void))
{ 
    IEC4bits.CNEIE = 0; //Disable CNEI interrupt
    IO_RE10_InterruptHandler = InterruptHandler; 
    IEC4bits.CNEIE = 1; //Enable CNEI interrupt
}

void IO_RE8_SetInterruptHandler(void (* InterruptHandler)(void))
{ 
    IEC4bits.CNEIE = 0; //Disable CNEI interrupt
    IO_RE8_InterruptHandler = InterruptHandler; 
    IEC4bits.CNEIE = 1; //Enable CNEI interrupt
}

void IO_RE9_SetInterruptHandler(void (* InterruptHandler)(void))
{ 
    IEC4bits.CNEIE = 0; //Disable CNEI interrupt
    IO_RE9_InterruptHandler = InterruptHandler; 
    IEC4bits.CNEIE = 1; //Enable CNEI interrupt
}

/* Interrupt service function for the CNEI interrupt. */
void __attribute__ (( interrupt, no_auto_psv )) _CNEInterrupt (void)
{
    if(CNFEbits.CNFE10 == 1)
    {
        if(IO_RE10_InterruptHandler != NULL) 
        { 
            IO_RE10_InterruptHandler(); 
        }
        
        CNFEbits.CNFE10 = 0;  //Clear flag for Pin - IO_RE10
    }
    
    if(CNFEbits.CNFE8 == 1)
    {
        if(IO_RE8_InterruptHandler != NULL) 
        { 
            IO_RE8_InterruptHandler(); 
        }
        
        CNFEbits.CNFE8 = 0;  //Clear flag for Pin - IO_RE8
    }
    
    if(CNFEbits.CNFE9 == 1)
    {
        if(IO_RE9_InterruptHandler != NULL) 
        { 
            IO_RE9_InterruptHandler(); 
        }
        
        CNFEbits.CNFE9 = 0;  //Clear flag for Pin - IO_RE9
    }
    
    // Clear the flag
    IFS4bits.CNEIF = 0;
}

