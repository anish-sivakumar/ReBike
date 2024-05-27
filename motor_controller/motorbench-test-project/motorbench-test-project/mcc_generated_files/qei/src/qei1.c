/**
 * QEI1 Generated Driver Source File 
 * 
 * @file      qei1.c
 *            
 * @ingroup   qeidriver
 *            
 * @brief     This is the generated driver source file for QEI1 driver
 *            
 * @version   Firmware Driver Version 1.2.2
 *
 * @version   PLIB Version 1.3.0
 *            
 * @skipline  Device : PIC24/dspIC/PIC32MM
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

#include "../qei1.h"


// Section: Driver Interface

const struct QEI_INTERFACE MCC_QEI = {
    .Initialize = &QEI1_Initialize,
    .Deinitialize = &QEI1_Deinitialize,
    .Enable = &QEI1_Enable,
    .Disable = &QEI1_Disable,
    .PositionCountRead = &QEI1_PositionCountRead,
    .PositionCount16bitRead = &QEI1_PositionCount16bitRead,
    .PositionCountWrite = &QEI1_PositionCountWrite,
    .ModuloRangeSet = &QEI1_ModuloRangeSet,
    .PhaseInputSwappedGet = &QEI1_PhaseInputSwappedGet,
    .PhaseInputSwappedSet = &QEI1_PhaseInputSwappedSet,
    .PositionCaptureEnable = &QEI1_PositionCaptureEnable,
    .PositionCaptureDisable = &QEI1_PositionCaptureDisable,
    .PositionCaptureGet = &QEI1_PositionCaptureGet,
    .PositionCapture16bitGet = &QEI1_PositionCapture16bitGet,
    .PositionCaptureSet = &QEI1_PositionCaptureSet,
    .CounterModeSet = &QEI1_CounterModeSet,
    .IMVGatedValueSet = &QEI1_IMVGatedValueSet
};

// Section: QEI1 Module APIs

void QEI1_Initialize(void)
{
    /* CCM Quadrature Encoder mode; GATEN disabled; CNTPOL Positive; INTDIV 1:1; IMV Index match occurs when QEBx = 0 and QEAx = 0; PIMOD Modulo Count mode; QEISIDL disabled; QEIEN disabled; */
    QEI1CON = 0x1800;
    /* QEAPOL disabled; QEBPOL disabled; IDXPOL disabled; HOMPOL disabled; SWPAB disabled; OUTFNC disabled; QFDIV 1:; FLTREN enabled; QCAPEN ; */
    QEI1IOC = 0x4000;
    /* HCAPEN ; */
    QEI1IOCH = 0x0;
    /* IDXIEN disabled; IDXIRQ No index event has occured; HOMIEN disabled; HOMIRQ No home event has occured; VELOVIEN disabled; VELOVIRQ No overflow has occured; PCIIEN disabled; PCIIRQ POSxCNT was not reinitialized; POSOVIEN disabled; POSOVIRQ No overflow has occured; PCLEQIEN disabled; PCLEQIRQ POSxCNT less than QEIxLEC; PCHEQIEN disabled; PCHEQIRQ POSxCNT less than QEIxGEC; */
    QEI1STAT = 0x0;
    /* POSCNTL 0x0; */
    POS1CNTL = 0x0;
    /* POSCNTH 0x0; */
    POS1CNTH = 0x0;
    /* POSHLDH 0x0; */
    POS1HLD = 0x0;
    /* VELCNTL 0x0; */
    VEL1CNT = 0x0;
    /* VELCNTH 0x0; */
    VEL1CNTH = 0x0;
    /* VELHLDH 0x0; */
    VEL1HLD = 0x0;
    /* INTTMRL 0x0; */
    INT1TMRL = 0x0;
    /* INTTMRH 0x0; */
    INT1TMRH = 0x0;
    /* INTHLDL 0x0; */
    INT1HLDL = 0x0;
    /* INTHLDH 0x0; */
    INT1HLDH = 0x0;
    /* INDXCNTL 0x0; */
    INDX1CNTL = 0x0;
    /* INDXCNTH 0x0; */
    INDX1CNTH = 0x0;
    /* INDXHLDH 0x0; */
    INDX1HLD = 0x0;
    /* QEIGECL 0x0; */
    QEI1GECL = 0x0;
    /* QEIGECH 0x0; */
    QEI1GECH = 0x0;
    /* QEIGECL 0x0; */
    QEI1ICL = 0x0;
    /* QEIGECH 0x0; */
    QEI1ICH = 0x0;
    /* QEILECL 0x0; */
    QEI1LECL = 0x0;
    /* QEILECH 0x0; */
    QEI1LECH = 0x0;
}

void QEI1_Deinitialize(void)
{
    QEI1CON = 0x0;
    QEI1IOC = 0x0;
    QEI1IOCH = 0x0;
    QEI1STAT = 0x0;
    POS1CNTL = 0x0;
    POS1CNTH = 0x0;
    POS1HLD = 0x0;
    VEL1CNT = 0x0;
    VEL1CNTH = 0x0;
    VEL1HLD = 0x0;
    INT1TMRL = 0x0;
    INT1TMRH = 0x0;
    INT1HLDL = 0x0;
    INT1HLDH = 0x0;
    INDX1CNTL = 0x0;
    INDX1CNTH = 0x0;
    INDX1HLD = 0x0;
    QEI1GECL = 0x0;
    QEI1GECH = 0x0;
    QEI1ICL = 0x0;
    QEI1ICH = 0x0;
    QEI1LECL = 0x0;
    QEI1LECH = 0x0;
}




