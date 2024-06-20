/**
 * SCCP4-TIMER Generated Driver Source File
 * 
 * @file      sccp4.c
 * 
 * @ingroup   timerdriver
 * 
 * @brief     This is the generated driver source file for SCCP4-TIMER driver
 *
 * @version   Firmware Driver Version 1.5.0
 *
 * @version   PLIB Version 1.5.0
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

#include <stddef.h> 
#include "../sccp4.h"
#include "../timer_interface.h"

// Section: Data Type Definitions

#define MASK_32_BIT_LOW 0x0000FFFFU
#define MASK_32_BIT_HIGH 0xFFFF0000U

// Section: File specific functions

static void (*SCCP4_TimeoutHandler)(void) = NULL;

// Section: Driver Interface

// Defines an object for TIMER_INTERFACE

const struct TIMER_INTERFACE Timer4 = {
    .Initialize     = &SCCP4_Timer_Initialize,
    .Deinitialize   = &SCCP4_Timer_Deinitialize,
    .Start          = &SCCP4_Timer_Start,
    .Stop           = &SCCP4_Timer_Stop,
    #if TIMER_PERIODCOUNTSET_API_SUPPORT
    .PeriodCountSet = &SCCP4_Timer_PeriodCountSet,
    #endif
    .PeriodSet      = &SCCP4_Timer_PeriodSet,
    .CounterGet     = &SCCP4_Timer_CounterGet,
    .PeriodGet	    = &SCCP4_Timer_PeriodGet,
    .InterruptPrioritySet = &SCCP4_Timer_InterruptPrioritySet,
    .TimeoutCallbackRegister = &SCCP4_Timer_TimeoutCallbackRegister,
    .Tasks          = NULL,
};

// Section: Driver Interface Function Definitions

void SCCP4_Timer_Initialize(void)
{
    // MOD ; CCSEL disabled; TMR32 16 Bit; TMRPS 1:64; CLKSEL FOSC/2; TMRSYNC disabled; CCPSLP disabled; CCPSIDL disabled; CCPON disabled; 
    CCP4CON1L = 0xC0; //The module is disabled, till other settings are configured
    //SYNC None; ALTSYNC disabled; ONESHOT disabled; TRIGEN disabled; IOPS Each Time Base Period Match; RTRGEN disabled; OPSRC Timer Interrupt Event; 
    CCP4CON1H = 0x0;
    //ASDG 0x0; SSDG disabled; ASDGM disabled; PWMRSEN disabled; 
    CCP4CON2L = 0x0;
    //ICSEL ; AUXOUT Special Event Trigger; ICGSM Level-Sensitive mode; OCAEN disabled; OENSYNC disabled; 
    CCP4CON2H = 0x10;
    //PSSACE Tri-state; POLACE disabled; OSCNT None; OETRIG disabled; 
    CCP4CON3H = 0x0;
    //ICOV disabled; ICDIS disabled; SCEVT disabled; ASEVT disabled; TRCLR disabled; TRSET disabled; ICGARM disabled; 
    CCP4STATL = 0x0;
    //TMRL 0x0000; 
    CCP4TMRL = 0x0;
    //TMRH 0x0000; 
    CCP4TMRH = 0x0;
    //PRL 0; 
    CCP4PRL = 0x0;
    //PRH 0; 
    CCP4PRH = 0x0;
    //CMPA 0; 
    CCP4RA = 0x0;
    //CMPB 0; 
    CCP4RB = 0x0;
    //BUFL 0x0000; 
    CCP4BUFL = 0x0;
    //BUFH 0x0000; 
    CCP4BUFH = 0x0;
    
    SCCP4_Timer_TimeoutCallbackRegister(&SCCP4_TimeoutCallback);

    IFS2bits.CCT4IF = 0;
    // Enabling SCCP4 interrupt
    IEC2bits.CCT4IE = 1;

    CCP4CON1Lbits.CCPON = 1; //Enable Module
}

void SCCP4_Timer_Deinitialize(void)
{
    CCP4CON1Lbits.CCPON = 0;
    
    IFS2bits.CCT4IF = 0;
    IEC2bits.CCT4IE = 0;
    
    CCP4CON1L = 0x0; 
    CCP4CON1H = 0x0; 
    CCP4CON2L = 0x0; 
    CCP4CON2H = 0x100; 
    CCP4CON3H = 0x0; 
    CCP4STATL = 0x0; 
    CCP4TMRL = 0x0; 
    CCP4TMRH = 0x0; 
    CCP4PRL = 0xFFFF; 
    CCP4PRH = 0xFFFF; 
    CCP4RA = 0x0; 
    CCP4RB = 0x0; 
    CCP4BUFL = 0x0; 
    CCP4BUFH = 0x0; 
}

void SCCP4_Timer_Start(void)
{
    IFS2bits.CCT4IF = 0;
    // Enable SCCP4 interrupt
    IEC2bits.CCT4IE = 1;
    
    CCP4CON1Lbits.CCPON = 1;
}

void SCCP4_Timer_Stop(void)
{
    CCP4CON1Lbits.CCPON = 0;
    
    IFS2bits.CCT4IF = 0;
    // Disable SCCP4 interrupt
    IEC2bits.CCT4IE = 0;
}

void SCCP4_Timer_PeriodSet(uint32_t count)
{
    if(count > 0xFFFFU)
    {
        CCP4PRL = (uint16_t)(count & MASK_32_BIT_LOW);
        CCP4PRH = (uint16_t)((count & MASK_32_BIT_HIGH) >> 16);
        CCP4CON1Lbits.T32 = 1;
    }
    else
    {
        CCP4PRL = (uint16_t)(count & MASK_32_BIT_LOW);
        CCP4CON1Lbits.T32 = 0;
    }
}

void SCCP4_Timer_InterruptPrioritySet(enum INTERRUPT_PRIORITY priority)
{
    IPC10bits.CCT4IP = priority;
}

void SCCP4_Timer_TimeoutCallbackRegister(void (*handler)(void))
{
    if(NULL != handler)
    {
        SCCP4_TimeoutHandler = handler;
    }
}

void SCCP4_TimeoutCallbackRegister(void* handler)
{
    if(NULL != handler)
    {
        SCCP4_TimeoutHandler = handler;
    }
}

void __attribute__ ((weak)) SCCP4_TimeoutCallback (void)
{ 

} 

void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCT4Interrupt (void)
{
    if(NULL != SCCP4_TimeoutHandler)
    {
        (*SCCP4_TimeoutHandler)();
    }
    IFS2bits.CCT4IF = 0;
}

void SCCP4_Timer_PeriodCountSet(size_t count)
{
    CCP4PRL = (uint16_t)(count & MASK_32_BIT_LOW);
    CCP4CON1Lbits.T32 = 0;
}

/**
 End of File
*/
