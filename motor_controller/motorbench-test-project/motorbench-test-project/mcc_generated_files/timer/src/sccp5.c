/**
 * SCCP5-TIMER Generated Driver Source File
 * 
 * @file      sccp5.c
 * 
 * @ingroup   timerdriver
 * 
 * @brief     This is the generated driver source file for SCCP5-TIMER driver
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
#include "../sccp5.h"
#include "../timer_interface.h"

// Section: Data Type Definitions

#define MASK_32_BIT_LOW 0x0000FFFFU
#define MASK_32_BIT_HIGH 0xFFFF0000U

// Section: File specific functions

static void (*SCCP5_TimeoutHandler)(void) = NULL;

// Section: Driver Interface

// Defines an object for TIMER_INTERFACE

const struct TIMER_INTERFACE Timer5 = {
    .Initialize     = &SCCP5_Timer_Initialize,
    .Deinitialize   = &SCCP5_Timer_Deinitialize,
    .Start          = &SCCP5_Timer_Start,
    .Stop           = &SCCP5_Timer_Stop,
    #if TIMER_PERIODCOUNTSET_API_SUPPORT
    .PeriodCountSet = &SCCP5_Timer_PeriodCountSet,
    #endif
    .PeriodSet      = &SCCP5_Timer_PeriodSet,
    .CounterGet     = &SCCP5_Timer_CounterGet,
    .PeriodGet	    = &SCCP5_Timer_PeriodGet,
    .InterruptPrioritySet = &SCCP5_Timer_InterruptPrioritySet,
    .TimeoutCallbackRegister = &SCCP5_Timer_TimeoutCallbackRegister,
    .Tasks          = NULL,
};

// Section: Driver Interface Function Definitions

void SCCP5_Timer_Initialize(void)
{
    // MOD ; CCSEL disabled; TMR32 16 Bit; TMRPS 1:1; CLKSEL FOSC/2; TMRSYNC disabled; CCPSLP disabled; CCPSIDL disabled; CCPON disabled; 
    CCP5CON1L = 0x0; //The module is disabled, till other settings are configured
    //SYNC None; ALTSYNC disabled; ONESHOT disabled; TRIGEN disabled; IOPS Each Time Base Period Match; RTRGEN disabled; OPSRC Timer Interrupt Event; 
    CCP5CON1H = 0x0;
    //ASDG 0x0; SSDG disabled; ASDGM disabled; PWMRSEN disabled; 
    CCP5CON2L = 0x0;
    //ICSEL ; AUXOUT Disabled; ICGSM Level-Sensitive mode; OCAEN disabled; OENSYNC disabled; 
    CCP5CON2H = 0x0;
    //PSSACE Tri-state; POLACE disabled; OSCNT None; OETRIG disabled; 
    CCP5CON3H = 0x0;
    //ICOV disabled; ICDIS disabled; SCEVT disabled; ASEVT disabled; TRCLR disabled; TRSET disabled; ICGARM disabled; 
    CCP5STATL = 0x0;
    //TMRL 0x0000; 
    CCP5TMRL = 0x0;
    //TMRH 0x0000; 
    CCP5TMRH = 0x0;
    //PRL 4999; 
    CCP5PRL = 0x1387;
    //PRH 0; 
    CCP5PRH = 0x0;
    //CMPA 0; 
    CCP5RA = 0x0;
    //CMPB 0; 
    CCP5RB = 0x0;
    //BUFL 0x0000; 
    CCP5BUFL = 0x0;
    //BUFH 0x0000; 
    CCP5BUFH = 0x0;
    
    SCCP5_Timer_TimeoutCallbackRegister(&SCCP5_TimeoutCallback);

    IFS2bits.CCT5IF = 0;
    // Enabling SCCP5 interrupt
    IEC2bits.CCT5IE = 1;

    CCP5CON1Lbits.CCPON = 1; //Enable Module
}

void SCCP5_Timer_Deinitialize(void)
{
    CCP5CON1Lbits.CCPON = 0;
    
    IFS2bits.CCT5IF = 0;
    IEC2bits.CCT5IE = 0;
    
    CCP5CON1L = 0x0; 
    CCP5CON1H = 0x0; 
    CCP5CON2L = 0x0; 
    CCP5CON2H = 0x100; 
    CCP5CON3H = 0x0; 
    CCP5STATL = 0x0; 
    CCP5TMRL = 0x0; 
    CCP5TMRH = 0x0; 
    CCP5PRL = 0xFFFF; 
    CCP5PRH = 0xFFFF; 
    CCP5RA = 0x0; 
    CCP5RB = 0x0; 
    CCP5BUFL = 0x0; 
    CCP5BUFH = 0x0; 
}

void SCCP5_Timer_Start(void)
{
    IFS2bits.CCT5IF = 0;
    // Enable SCCP5 interrupt
    IEC2bits.CCT5IE = 1;
    
    CCP5CON1Lbits.CCPON = 1;
}

void SCCP5_Timer_Stop(void)
{
    CCP5CON1Lbits.CCPON = 0;
    
    IFS2bits.CCT5IF = 0;
    // Disable SCCP5 interrupt
    IEC2bits.CCT5IE = 0;
}

void SCCP5_Timer_PeriodSet(uint32_t count)
{
    if(count > 0xFFFFU)
    {
        CCP5PRL = (uint16_t)(count & MASK_32_BIT_LOW);
        CCP5PRH = (uint16_t)((count & MASK_32_BIT_HIGH) >> 16);
        CCP5CON1Lbits.T32 = 1;
    }
    else
    {
        CCP5PRL = (uint16_t)(count & MASK_32_BIT_LOW);
        CCP5CON1Lbits.T32 = 0;
    }
}

void SCCP5_Timer_InterruptPrioritySet(enum INTERRUPT_PRIORITY priority)
{
    IPC11bits.CCT5IP = priority;
}

void SCCP5_Timer_TimeoutCallbackRegister(void (*handler)(void))
{
    if(NULL != handler)
    {
        SCCP5_TimeoutHandler = handler;
    }
}

void SCCP5_TimeoutCallbackRegister(void* handler)
{
    if(NULL != handler)
    {
        SCCP5_TimeoutHandler = handler;
    }
}

void __attribute__ ((weak)) SCCP5_TimeoutCallback (void)
{ 
    
} 

void __attribute__ ( ( interrupt, no_auto_psv ) ) _CCT5Interrupt (void)
{
    if(NULL != SCCP5_TimeoutHandler)
    {
        (*SCCP5_TimeoutHandler)();
    }
    IFS2bits.CCT5IF = 0;
}

void SCCP5_Timer_PeriodCountSet(size_t count)
{
    CCP5PRL = (uint16_t)(count & MASK_32_BIT_LOW);
    CCP5CON1Lbits.T32 = 0;
}

/**
 End of File
*/
