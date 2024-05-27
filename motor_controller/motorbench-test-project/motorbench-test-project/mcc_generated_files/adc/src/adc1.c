/**
 * ADC1 Generated Driver Source File
 * 
 * @file      adc1.c
 *            
 * @ingroup   adcdriver
 *            
 * @brief     This is the generated driver source file for ADC1 driver
 *            
 * @version   Firmware Driver Version 1.4.3
 *
 * @version   PLIB Version 2.3.0
 *           
 * @skipline  Device : dsPIC33CK256MP508
*/

/*
� [2024] Microchip Technology Inc. and its subsidiaries.

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
#include "../adc1.h"

// Section: File specific functions

static void (*ADC1_CommonHandler)(void) = NULL;
static void (*ADC1_ChannelHandler)(enum ADC_CHANNEL channel, uint16_t adcVal) = NULL;
static void (*ADC1_ComparatorHandler)(enum ADC_CMP comparator) = NULL;

// Section: File specific data type definitions

/**
 @ingroup  adcdriver
 @enum     ADC_PWM_TRIG_SRCS
 @brief    Defines the PWM ADC TRIGGER sources available for the module to use.
*/
enum ADC_PWM_TRIG_SRCS {
    PWM8_TRIGGER2 = 0x13, 
    PWM8_TRIGGER1 = 0x12, 
    PWM7_TRIGGER2 = 0x11, 
    PWM7_TRIGGER1 = 0x10, 
    PWM6_TRIGGER2 = 0xf, 
    PWM6_TRIGGER1 = 0xe, 
    PWM5_TRIGGER2 = 0xd, 
    PWM5_TRIGGER1 = 0xc, 
    PWM4_TRIGGER2 = 0xb, 
    PWM4_TRIGGER1 = 0xa, 
    PWM3_TRIGGER2 = 0x9, 
    PWM3_TRIGGER1 = 0x8, 
    PWM2_TRIGGER2 = 0x7, 
    PWM2_TRIGGER1 = 0x6, 
    PWM1_TRIGGER2 = 0x5, 
    PWM1_TRIGGER1 = 0x4, 
};

//Defines an object for ADC_MULTICORE.
const struct ADC_MULTICORE adc1Multicore = {
    .ChannelTasks                           = &ADC1_ChannelTasks, 
    .ComparatorTasks                        = NULL,
    .IndividualChannelInterruptEnable       = &ADC1_IndividualChannelInterruptEnable,
    .IndividualChannelInterruptDisable      = &ADC1_IndividualChannelInterruptDisable,
    .IndividualChannelInterruptFlagClear    = &ADC1_IndividualChannelInterruptFlagClear,
    .IndividualChannelInterruptPrioritySet  = &ADC1_IndividualChannelInterruptPrioritySet,
    .ChannelCallbackRegister                = &ADC1_ChannelCallbackRegister,
    .ComparatorCallbackRegister             = &ADC1_ComparatorCallbackRegister,
    .CorePowerEnable                        = &ADC1_CorePowerEnable,
    .SharedCorePowerEnable                  = &ADC1_SharedCorePowerEnable,
    .PWMTriggerSourceSet                    = &ADC1_PWMTriggerSourceSet
};

//Defines an object for ADC_INTERFACE.

const struct ADC_INTERFACE MCC_ADC = {
    .Initialize             = &ADC1_Initialize,
    .Deinitialize           = &ADC1_Deinitialize,
    .Enable                 = &ADC1_Enable,
    .Disable                = &ADC1_Disable,
    .SoftwareTriggerEnable  = &ADC1_SoftwareTriggerEnable,
    .SoftwareTriggerDisable = &ADC1_SoftwareTriggerDisable,
    .ChannelSelect          = &ADC1_ChannelSelect, 
    .ConversionResultGet    = &ADC1_ConversionResultGet,
    .IsConversionComplete   = &ADC1_IsConversionComplete,
    .ResolutionSet          = &ADC1_ResolutionSet,
    .InterruptEnable        = &ADC1_InterruptEnable,
    .InterruptDisable       = &ADC1_InterruptDisable,
    .InterruptFlagClear     = &ADC1_InterruptFlagClear,
    .InterruptPrioritySet   = &ADC1_InterruptPrioritySet,
    .CommonCallbackRegister = &ADC1_CommonCallbackRegister,
    .Tasks                  = &ADC1_Tasks,
    .adcMulticoreInterface = &adc1Multicore,
};

// Section: Driver Interface Function Definitions

void ADC1_Initialize (void)
{
    // ADSIDL disabled; ADON enabled; 
    ADCON1L = (uint16_t)0x8000 & (uint16_t)0x7FFF; //Disabling ADON bit
    // SHRRES 12-bit resolution; FORM Fractional; 
    ADCON1H = 0xE0;
    // SHRADCS 2; SHREISEL Early interrupt is generated 1 TADCORE clock prior to data being ready; PTGEN disabled; EIEN disabled; REFERCIE disabled; REFCIE disabled; 
    ADCON2L = 0x0;
    // SHRSAMC 13; 
    ADCON2H = 0xD;
    // CNVCHSEL AN0; SWCTRG disabled; SWLCTRG disabled; SHRSAMP disabled; SUSPCIE disabled; SUSPEND disabled; REFSEL disabled; 
    ADCON3L = 0x0;
    // C0EN enabled; C1EN enabled; SHREN enabled; CLKDIV 1; CLKSEL FOSC/2; 
    ADCON3H = (uint16_t)0x83 & (uint16_t)0xFF00; //Disabling C0EN, C1EN, C2EN, C3EN and SHREN bits
    // SAMC0EN enabled; SAMC1EN enabled; 
    ADCON4L = 0x3;
    // C0CHS AN0; C1CHS AN1; 
    ADCON4H = 0x0;
    // SIGN0 enabled; DIFF0 disabled; SIGN1 enabled; DIFF1 disabled; SIGN2 disabled; DIFF2 disabled; SIGN3 disabled; DIFF3 disabled; SIGN4 enabled; DIFF4 disabled; SIGN5 disabled; DIFF5 disabled; SIGN6 disabled; DIFF6 disabled; SIGN7 disabled; DIFF7 disabled; 
    ADMOD0L = 0x105;
    // SIGN8 disabled; DIFF8 disabled; SIGN9 disabled; DIFF9 disabled; SIGN10 enabled; DIFF10 disabled; SIGN11 enabled; DIFF11 disabled; SIGN12 disabled; DIFF12 disabled; SIGN13 disabled; DIFF13 disabled; SIGN14 disabled; DIFF14 disabled; SIGN15 enabled; DIFF15 disabled; 
    ADMOD0H = 0x4050;
    // SIGN16 disabled; DIFF16 disabled; SIGN17 disabled; DIFF17 disabled; SIGN18 disabled; DIFF18 disabled; SIGN19 disabled; DIFF19 disabled; SIGN20 disabled; DIFF20 disabled; SIGN21 disabled; DIFF21 disabled; SIGN22 disabled; DIFF22 disabled; SIGN23 disabled; DIFF23 disabled; 
    ADMOD1L = 0x0;
    // SIGN24 disabled; DIFF24 disabled; SIGN25 disabled; DIFF25 disabled; 
    ADMOD1H = 0x0;
    // IE0 disabled; IE1 disabled; IE2 disabled; IE3 disabled; IE4 disabled; IE5 disabled; IE6 disabled; IE7 disabled; IE8 disabled; IE9 disabled; IE10 disabled; IE11 disabled; IE12 disabled; IE13 disabled; IE14 disabled; IE15 disabled; 
    ADIEL = 0x0;
    // IE16 disabled; IE17 disabled; IE18 disabled; IE19 disabled; IE20 disabled; IE21 disabled; IE22 disabled; IE23 disabled; IE24 disabled; IE25 disabled; 
    ADIEH = 0x0;
    // 
    ADSTATL = 0x0;
    // 
    ADSTATH = 0x0;
    // CMPEN0 disabled; CMPEN1 disabled; CMPEN2 disabled; CMPEN3 disabled; CMPEN4 disabled; CMPEN5 disabled; CMPEN6 disabled; CMPEN7 disabled; CMPEN8 disabled; CMPEN9 disabled; CMPEN10 disabled; CMPEN11 disabled; CMPEN12 disabled; CMPEN13 disabled; CMPEN14 disabled; CMPEN15 disabled; 
    ADCMP0ENL = 0x0;
    // CMPEN0 disabled; CMPEN1 disabled; CMPEN2 disabled; CMPEN3 disabled; CMPEN4 disabled; CMPEN5 disabled; CMPEN6 disabled; CMPEN7 disabled; CMPEN8 disabled; CMPEN9 disabled; CMPEN10 disabled; CMPEN11 disabled; CMPEN12 disabled; CMPEN13 disabled; CMPEN14 disabled; CMPEN15 disabled; 
    ADCMP1ENL = 0x0;
    // CMPEN0 disabled; CMPEN1 disabled; CMPEN2 disabled; CMPEN3 disabled; CMPEN4 disabled; CMPEN5 disabled; CMPEN6 disabled; CMPEN7 disabled; CMPEN8 disabled; CMPEN9 disabled; CMPEN10 disabled; CMPEN11 disabled; CMPEN12 disabled; CMPEN13 disabled; CMPEN14 disabled; CMPEN15 disabled; 
    ADCMP2ENL = 0x0;
    // CMPEN0 disabled; CMPEN1 disabled; CMPEN2 disabled; CMPEN3 disabled; CMPEN4 disabled; CMPEN5 disabled; CMPEN6 disabled; CMPEN7 disabled; CMPEN8 disabled; CMPEN9 disabled; CMPEN10 disabled; CMPEN11 disabled; CMPEN12 disabled; CMPEN13 disabled; CMPEN14 disabled; CMPEN15 disabled; 
    ADCMP3ENL = 0x0;
    // CMPEN16 disabled; CMPEN17 disabled; CMPEN18 disabled; CMPEN19 disabled; CMPEN20 disabled; CMPEN21 disabled; CMPEN22 disabled; CMPEN23 disabled; CMPEN24 disabled; CMPEN25 disabled; 
    ADCMP0ENH = 0x0;
    // CMPEN16 disabled; CMPEN17 disabled; CMPEN18 disabled; CMPEN19 disabled; CMPEN20 disabled; CMPEN21 disabled; CMPEN22 disabled; CMPEN23 disabled; CMPEN24 disabled; CMPEN25 disabled; 
    ADCMP1ENH = 0x0;
    // CMPEN16 disabled; CMPEN17 disabled; CMPEN18 disabled; CMPEN19 disabled; CMPEN20 disabled; CMPEN21 disabled; CMPEN22 disabled; CMPEN23 disabled; CMPEN24 disabled; CMPEN25 disabled; 
    ADCMP2ENH = 0x0;
    // CMPEN16 disabled; CMPEN17 disabled; CMPEN18 disabled; CMPEN19 disabled; CMPEN20 disabled; CMPEN21 disabled; CMPEN22 disabled; CMPEN23 disabled; CMPEN24 disabled; CMPEN25 disabled; 
    ADCMP3ENH = 0x0;
    // CMPLO 0x0; 
    ADCMP0LO = 0x0;
    // CMPLO 0x0; 
    ADCMP1LO = 0x0;
    // CMPLO 0x0; 
    ADCMP2LO = 0x0;
    // CMPLO 0x0; 
    ADCMP3LO = 0x0;
    // CMPHI 0x0; 
    ADCMP0HI = 0x0;
    // CMPHI 0x0; 
    ADCMP1HI = 0x0;
    // CMPHI 0x0; 
    ADCMP2HI = 0x0;
    // CMPHI 0x0; 
    ADCMP3HI = 0x0;
    // 
    ADFL0DAT = 0x0;
    // 
    ADFL1DAT = 0x0;
    // 
    ADFL2DAT = 0x0;
    // 
    ADFL3DAT = 0x0;
    // FLCHSEL AN0; IE disabled; OVRSAM 4x; MODE Oversampling Mode; FLEN disabled; 
    ADFL0CON = 0x400;
    // FLCHSEL AN0; IE disabled; OVRSAM 4x; MODE Oversampling Mode; FLEN disabled; 
    ADFL1CON = 0x400;
    // FLCHSEL AN0; IE disabled; OVRSAM 4x; MODE Oversampling Mode; FLEN disabled; 
    ADFL2CON = 0x400;
    // FLCHSEL AN0; IE disabled; OVRSAM 4x; MODE Oversampling Mode; FLEN disabled; 
    ADFL3CON = 0x400;
    // LOLO disabled; LOHI disabled; HILO disabled; HIHI disabled; BTWN disabled; IE disabled; CMPEN disabled; 
    ADCMP0CON = 0x0;
    // LOLO disabled; LOHI disabled; HILO disabled; HIHI disabled; BTWN disabled; IE disabled; CMPEN disabled; 
    ADCMP1CON = 0x0;
    // LOLO disabled; LOHI disabled; HILO disabled; HIHI disabled; BTWN disabled; IE disabled; CMPEN disabled; 
    ADCMP2CON = 0x0;
    // LOLO disabled; LOHI disabled; HILO disabled; HIHI disabled; BTWN disabled; IE disabled; CMPEN disabled; 
    ADCMP3CON = 0x0;
    // LVLEN0 disabled; LVLEN1 disabled; LVLEN2 disabled; LVLEN3 disabled; LVLEN4 disabled; LVLEN5 disabled; LVLEN6 disabled; LVLEN7 disabled; LVLEN8 disabled; LVLEN9 disabled; LVLEN10 disabled; LVLEN11 disabled; LVLEN12 disabled; LVLEN13 disabled; LVLEN14 disabled; LVLEN15 disabled; 
    ADLVLTRGL = 0x0;
    // LVLEN16 disabled; LVLEN17 disabled; LVLEN18 disabled; LVLEN19 disabled; LVLEN20 disabled; LVLEN21 disabled; LVLEN22 disabled; LVLEN23 disabled; LVLEN24 disabled; LVLEN25 disabled; 
    ADLVLTRGH = 0x0;
    // SAMC 13; 
    ADCORE0L = 0xD;
    // SAMC 13; 
    ADCORE1L = 0xD;
    // ADCS 2; RES 12-bit resolution; EISEL Early interrupt is generated 1 TADCORE clock prior to data being ready; 
    ADCORE0H = 0x300;
    // ADCS 2; RES 12-bit resolution; EISEL Early interrupt is generated 1 TADCORE clock prior to data being ready; 
    ADCORE1H = 0x300;
    // EIEN0 disabled; EIEN1 disabled; EIEN2 disabled; EIEN3 disabled; EIEN4 disabled; EIEN5 disabled; EIEN6 disabled; EIEN7 disabled; EIEN8 disabled; EIEN9 disabled; EIEN10 disabled; EIEN11 disabled; EIEN12 disabled; EIEN13 disabled; EIEN14 disabled; EIEN15 disabled; 
    ADEIEL = 0x0;
    // EIEN16 disabled; EIEN17 disabled; EIEN18 disabled; EIEN19 disabled; EIEN20 disabled; EIEN21 disabled; EIEN22 disabled; EIEN23 disabled; EIEN24 disabled; EIEN25 disabled; 
    ADEIEH = 0x0;
    // EISTAT0 disabled; EISTAT1 disabled; EISTAT2 disabled; EISTAT3 disabled; EISTAT4 disabled; EISTAT5 disabled; EISTAT6 disabled; EISTAT7 disabled; EISTAT8 disabled; EISTAT9 disabled; EISTAT10 disabled; EISTAT11 disabled; EISTAT12 disabled; EISTAT13 disabled; EISTAT14 disabled; EISTAT15 disabled; 
    ADEISTATL = 0x0;
    // EISTAT16 disabled; EISTAT17 disabled; EISTAT18 disabled; EISTAT19 disabled; EISTAT20 disabled; EISTAT21 disabled; EISTAT22 disabled; EISTAT23 disabled; EISTAT24 disabled; EISTAT25 disabled; 
    ADEISTATH = 0x0;
    // C0CIE disabled; C1CIE disabled; SHRCIE disabled; WARMTIME 16 Source Clock Periods; 
    ADCON5H = (uint16_t)0x400 & (uint16_t)0xF0FF; //Disabling WARMTIME bit
    // 
    ADCBUF0 = 0x0;
    // 
    ADCBUF1 = 0x0;
    // 
    ADCBUF2 = 0x0;
    // 
    ADCBUF3 = 0x0;
    // 
    ADCBUF4 = 0x0;
    // 
    ADCBUF5 = 0x0;
    // 
    ADCBUF6 = 0x0;
    // 
    ADCBUF7 = 0x0;
    // 
    ADCBUF8 = 0x0;
    // 
    ADCBUF9 = 0x0;
    // 
    ADCBUF10 = 0x0;
    // 
    ADCBUF11 = 0x0;
    // 
    ADCBUF12 = 0x0;
    // 
    ADCBUF13 = 0x0;
    // 
    ADCBUF14 = 0x0;
    // 
    ADCBUF15 = 0x0;
    // 
    ADCBUF16 = 0x0;
    // 
    ADCBUF17 = 0x0;
    // 
    ADCBUF18 = 0x0;
    // 
    ADCBUF19 = 0x0;
    // 
    ADCBUF20 = 0x0;
    // 
    ADCBUF21 = 0x0;
    // 
    ADCBUF22 = 0x0;
    // 
    ADCBUF23 = 0x0;
    // 
    ADCBUF24 = 0x0;
    // 
    ADCBUF25 = 0x0;
        
    ADC1_CommonCallbackRegister(&ADC1_CommonCallback);
    ADC1_ChannelCallbackRegister(&ADC1_ChannelCallback);
    ADC1_ComparatorCallbackRegister(&ADC1_ComparatorCallback);
    

    // Setting WARMTIME bit
    ADCON5Hbits.WARMTIME = 0xF;
    // Enabling ADC Module
    ADCON1Lbits.ADON = 0x1;
    // Enabling Power for Core0
    ADC1_CorePowerEnable(ADC_CORE_0);
    // Enabling Power for Core1
    ADC1_CorePowerEnable(ADC_CORE_1);
    // Enabling Power for the Shared Core
    ADC1_SharedCorePowerEnable();

    //TRGSRC0 None; TRGSRC1 None; 
    ADTRIG0L = 0x0;
    //TRGSRC2 None; TRGSRC3 None; 
    ADTRIG0H = 0x0;
    //TRGSRC4 None; TRGSRC5 None; 
    ADTRIG1L = 0x0;
    //TRGSRC6 None; TRGSRC7 None; 
    ADTRIG1H = 0x0;
    //TRGSRC8 None; TRGSRC9 None; 
    ADTRIG2L = 0x0;
    //TRGSRC10 None; TRGSRC11 None; 
    ADTRIG2H = 0x0;
    //TRGSRC12 None; TRGSRC13 None; 
    ADTRIG3L = 0x0;
    //TRGSRC14 None; TRGSRC15 None; 
    ADTRIG3H = 0x0;
    //TRGSRC16 None; TRGSRC17 None; 
    ADTRIG4L = 0x0;
    //TRGSRC18 None; TRGSRC19 None; 
    ADTRIG4H = 0x0;
    //TRGSRC20 None; TRGSRC21 None; 
    ADTRIG5L = 0x0;
    //TRGSRC22 None; TRGSRC23 None; 
    ADTRIG5H = 0x0;
    //TRGSRC24 None; TRGSRC25 None; 
    ADTRIG6L = 0x0;
}

void ADC1_Deinitialize (void)
{
    ADCON1Lbits.ADON = 0;
    
    ADCON1L = 0x0;
    ADCON1H = 0x60;
    ADCON2L = 0x0;
    ADCON2H = 0x0;
    ADCON3L = 0x0;
    ADCON3H = 0x0;
    ADCON4L = 0x0;
    ADCON4H = 0x0;
    ADMOD0L = 0x0;
    ADMOD0H = 0x0;
    ADMOD1L = 0x0;
    ADMOD1H = 0x0;
    ADIEL = 0x0;
    ADIEH = 0x0;
    ADSTATL = 0x0;
    ADSTATH = 0x0;
    ADCMP0ENL = 0x0;
    ADCMP1ENL = 0x0;
    ADCMP2ENL = 0x0;
    ADCMP3ENL = 0x0;
    ADCMP0ENH = 0x0;
    ADCMP1ENH = 0x0;
    ADCMP2ENH = 0x0;
    ADCMP3ENH = 0x0;
    ADCMP0LO = 0x0;
    ADCMP1LO = 0x0;
    ADCMP2LO = 0x0;
    ADCMP3LO = 0x0;
    ADCMP0HI = 0x0;
    ADCMP1HI = 0x0;
    ADCMP2HI = 0x0;
    ADCMP3HI = 0x0;
    ADFL0DAT = 0x0;
    ADFL1DAT = 0x0;
    ADFL2DAT = 0x0;
    ADFL3DAT = 0x0;
    ADFL0CON = 0x0;
    ADFL1CON = 0x0;
    ADFL2CON = 0x0;
    ADFL3CON = 0x0;
    ADTRIG0L = 0x0;
    ADTRIG0H = 0x0;
    ADTRIG1L = 0x0;
    ADTRIG1H = 0x0;
    ADTRIG2L = 0x0;
    ADTRIG2H = 0x0;
    ADTRIG3L = 0x0;
    ADTRIG3H = 0x0;
    ADTRIG4L = 0x0;
    ADTRIG4H = 0x0;
    ADTRIG5L = 0x0;
    ADTRIG5H = 0x0;
    ADTRIG6L = 0x0;
    ADCMP0CON = 0x0;
    ADCMP1CON = 0x0;
    ADCMP2CON = 0x0;
    ADCMP3CON = 0x0;
    ADLVLTRGL = 0x0;
    ADLVLTRGH = 0x0;
    ADCORE0L = 0x0;
    ADCORE1L = 0x0;
    ADCORE0H = 0x300;
    ADCORE1H = 0x300;
    ADEIEL = 0x0;
    ADEIEH = 0x0;
    ADEISTATL = 0x0;
    ADEISTATH = 0x0;
    ADCON5L = 0x0;
    ADCON5H = 0x0;
    ADCBUF0 = 0x0;
    ADCBUF1 = 0x0;
    ADCBUF2 = 0x0;
    ADCBUF3 = 0x0;
    ADCBUF4 = 0x0;
    ADCBUF5 = 0x0;
    ADCBUF6 = 0x0;
    ADCBUF7 = 0x0;
    ADCBUF8 = 0x0;
    ADCBUF9 = 0x0;
    ADCBUF10 = 0x0;
    ADCBUF11 = 0x0;
    ADCBUF12 = 0x0;
    ADCBUF13 = 0x0;
    ADCBUF14 = 0x0;
    ADCBUF15 = 0x0;
    ADCBUF16 = 0x0;
    ADCBUF17 = 0x0;
    ADCBUF18 = 0x0;
    ADCBUF19 = 0x0;
    ADCBUF20 = 0x0;
    ADCBUF21 = 0x0;
    ADCBUF22 = 0x0;
    ADCBUF23 = 0x0;
    ADCBUF24 = 0x0;
    ADCBUF25 = 0x0;
}

void ADC1_CorePowerEnable(enum ADC_DEDICATED_CORE core) 
{
    switch(core)
    {
        case ADC_CORE_0:
            ADCON5Lbits.C0PWR = 1; 
            while(ADCON5Lbits.C0RDY == 0)
            {
            }
            ADCON3Hbits.C0EN = 1; 
            break;
        case ADC_CORE_1:
            ADCON5Lbits.C1PWR = 1; 
            while(ADCON5Lbits.C1RDY == 0)
            {
            }
            ADCON3Hbits.C1EN = 1; 
            break;
        default:    
            break;
    }
}


void ADC1_SharedCorePowerEnable (void) 
{
    ADCON5Lbits.SHRPWR = 1;   
    while(ADCON5Lbits.SHRRDY == 0)
    {
    }
    ADCON3Hbits.SHREN = 1;   
}

static uint16_t ADC1_TriggerSourceValueGet(enum ADC_PWM_INSTANCE pwmInstance, enum ADC_PWM_TRIGGERS triggerNumber)
{
    uint16_t adcTriggerSourceValue = 0x0U;
    switch(pwmInstance)
    {
        case 1:
                if(triggerNumber == ADC_PWM_TRIGGER_1)
                {
                    adcTriggerSourceValue = PWM1_TRIGGER1;
                }
                else if(triggerNumber == ADC_PWM_TRIGGER_2)
                {
                    adcTriggerSourceValue = PWM1_TRIGGER2;
                }
                break;
        case 2:
                if(triggerNumber == ADC_PWM_TRIGGER_1)
                {
                    adcTriggerSourceValue = PWM2_TRIGGER1;
                }
                else if(triggerNumber == ADC_PWM_TRIGGER_2)
                {
                    adcTriggerSourceValue = PWM2_TRIGGER2;
                }
                break;
        case 3:
                if(triggerNumber == ADC_PWM_TRIGGER_1)
                {
                    adcTriggerSourceValue = PWM3_TRIGGER1;
                }
                else if(triggerNumber == ADC_PWM_TRIGGER_2)
                {
                    adcTriggerSourceValue = PWM3_TRIGGER2;
                }
                break;
        case 4:
                if(triggerNumber == ADC_PWM_TRIGGER_1)
                {
                    adcTriggerSourceValue = PWM4_TRIGGER1;
                }
                else if(triggerNumber == ADC_PWM_TRIGGER_2)
                {
                    adcTriggerSourceValue = PWM4_TRIGGER2;
                }
                break;
        case 5:
                if(triggerNumber == ADC_PWM_TRIGGER_1)
                {
                    adcTriggerSourceValue = PWM5_TRIGGER1;
                }
                else if(triggerNumber == ADC_PWM_TRIGGER_2)
                {
                    adcTriggerSourceValue = PWM5_TRIGGER2;
                }
                break;
        case 6:
                if(triggerNumber == ADC_PWM_TRIGGER_1)
                {
                    adcTriggerSourceValue = PWM6_TRIGGER1;
                }
                else if(triggerNumber == ADC_PWM_TRIGGER_2)
                {
                    adcTriggerSourceValue = PWM6_TRIGGER2;
                }
                break;
        case 7:
                if(triggerNumber == ADC_PWM_TRIGGER_1)
                {
                    adcTriggerSourceValue = PWM7_TRIGGER1;
                }
                else if(triggerNumber == ADC_PWM_TRIGGER_2)
                {
                    adcTriggerSourceValue = PWM7_TRIGGER2;
                }
                break;
        case 8:
                if(triggerNumber == ADC_PWM_TRIGGER_1)
                {
                    adcTriggerSourceValue = PWM8_TRIGGER1;
                }
                else if(triggerNumber == ADC_PWM_TRIGGER_2)
                {
                    adcTriggerSourceValue = PWM8_TRIGGER2;
                }
                break;
         default:
                break;
    }
    return adcTriggerSourceValue;
}

void ADC1_PWMTriggerSourceSet(enum ADC_CHANNEL channel, enum ADC_PWM_INSTANCE pwmInstance, enum ADC_PWM_TRIGGERS triggerNumber)
{
    uint16_t adcTriggerValue;
    adcTriggerValue= ADC1_TriggerSourceValueGet(pwmInstance, triggerNumber);
    switch(channel)
    {
        case MCAF_ADC_DCLINK_CURRENT:
                ADTRIG1Lbits.TRGSRC4 = adcTriggerValue;
                break;
        case MCAF_ADC_PHASEC_CURRENT:
                ADTRIG2Hbits.TRGSRC10 = adcTriggerValue;
                break;
        case MCAF_ADC_POTENTIOMETER:
                ADTRIG2Hbits.TRGSRC11 = adcTriggerValue;
                break;
        case MCAF_ADC_BRIDGE_TEMPERATURE:
                ADTRIG3Lbits.TRGSRC12 = adcTriggerValue;
                break;
        case MCAF_ADC_DCLINK_VOLTAGE:
                ADTRIG3Hbits.TRGSRC15 = adcTriggerValue;
                break;
        case MCAF_ADC_PHASEA_VOLTAGE:
                ADTRIG4Lbits.TRGSRC17 = adcTriggerValue;
                break;
        case MCAF_ADC_PHASEC_VOLTAGE:
                ADTRIG5Hbits.TRGSRC22 = adcTriggerValue;
                break;
        case MCAF_ADC_PHASEB_VOLTAGE:
                ADTRIG5Hbits.TRGSRC23 = adcTriggerValue;
                break;
        case MCAF_ADC_PHASEA_CURRENT:
                ADTRIG0Lbits.TRGSRC0 = adcTriggerValue;
                break;
        case MCAF_ADC_PHASEB_CURRENT:
                ADTRIG0Lbits.TRGSRC1 = adcTriggerValue;
                break;
        default:
                break;
    }
}

void ADC1_CommonCallbackRegister(void(*callback)(void))
{
    if(NULL != callback)
    {
        ADC1_CommonHandler = callback;
    }
}

void __attribute__ ((weak)) ADC1_CommonCallback (void)
{ 

} 


void __attribute__ ( ( __interrupt__ , auto_psv, weak ) ) _ADCInterrupt ( void )
{
    uint16_t adcVal;
    if(NULL != ADC1_CommonHandler)
    {
        (*ADC1_CommonHandler)();
    }
    
    if(IFS5bits.ADCAN4IF == 1)
    {
        //Read the ADC value from the ADCBUF before clearing interrupt
        adcVal = ADCBUF4;
        if(NULL != ADC1_ChannelHandler)
        {
            (*ADC1_ChannelHandler)(MCAF_ADC_DCLINK_CURRENT, adcVal);
        }
        IFS5bits.ADCAN4IF = 0;
    }
    if(IFS6bits.ADCAN10IF == 1)
    {
        //Read the ADC value from the ADCBUF before clearing interrupt
        adcVal = ADCBUF10;
        if(NULL != ADC1_ChannelHandler)
        {
            (*ADC1_ChannelHandler)(MCAF_ADC_PHASEC_CURRENT, adcVal);
        }
        IFS6bits.ADCAN10IF = 0;
    }
    if(IFS6bits.ADCAN11IF == 1)
    {
        //Read the ADC value from the ADCBUF before clearing interrupt
        adcVal = ADCBUF11;
        if(NULL != ADC1_ChannelHandler)
        {
            (*ADC1_ChannelHandler)(MCAF_ADC_POTENTIOMETER, adcVal);
        }
        IFS6bits.ADCAN11IF = 0;
    }
    if(IFS6bits.ADCAN12IF == 1)
    {
        //Read the ADC value from the ADCBUF before clearing interrupt
        adcVal = ADCBUF12;
        if(NULL != ADC1_ChannelHandler)
        {
            (*ADC1_ChannelHandler)(MCAF_ADC_BRIDGE_TEMPERATURE, adcVal);
        }
        IFS6bits.ADCAN12IF = 0;
    }
    if(IFS6bits.ADCAN15IF == 1)
    {
        //Read the ADC value from the ADCBUF before clearing interrupt
        adcVal = ADCBUF15;
        if(NULL != ADC1_ChannelHandler)
        {
            (*ADC1_ChannelHandler)(MCAF_ADC_DCLINK_VOLTAGE, adcVal);
        }
        IFS6bits.ADCAN15IF = 0;
    }
    if(IFS6bits.ADCAN17IF == 1)
    {
        //Read the ADC value from the ADCBUF before clearing interrupt
        adcVal = ADCBUF17;
        if(NULL != ADC1_ChannelHandler)
        {
            (*ADC1_ChannelHandler)(MCAF_ADC_PHASEA_VOLTAGE, adcVal);
        }
        IFS6bits.ADCAN17IF = 0;
    }
    if(IFS7bits.ADCAN22IF == 1)
    {
        //Read the ADC value from the ADCBUF before clearing interrupt
        adcVal = ADCBUF22;
        if(NULL != ADC1_ChannelHandler)
        {
            (*ADC1_ChannelHandler)(MCAF_ADC_PHASEC_VOLTAGE, adcVal);
        }
        IFS7bits.ADCAN22IF = 0;
    }
    if(IFS7bits.ADCAN23IF == 1)
    {
        //Read the ADC value from the ADCBUF before clearing interrupt
        adcVal = ADCBUF23;
        if(NULL != ADC1_ChannelHandler)
        {
            (*ADC1_ChannelHandler)(MCAF_ADC_PHASEB_VOLTAGE, adcVal);
        }
        IFS7bits.ADCAN23IF = 0;
    }
    if(IFS5bits.ADCAN0IF == 1)
    {
        //Read the ADC value from the ADCBUF before clearing interrupt
        adcVal = ADCBUF0;
        if(NULL != ADC1_ChannelHandler)
        {
            (*ADC1_ChannelHandler)(MCAF_ADC_PHASEA_CURRENT, adcVal);
        }
        IFS5bits.ADCAN0IF = 0;
    }
    if(IFS5bits.ADCAN1IF == 1)
    {
        //Read the ADC value from the ADCBUF before clearing interrupt
        adcVal = ADCBUF1;
        if(NULL != ADC1_ChannelHandler)
        {
            (*ADC1_ChannelHandler)(MCAF_ADC_PHASEB_CURRENT, adcVal);
        }
        IFS5bits.ADCAN1IF = 0;
    }
        
    // clear the ADC1 interrupt flag
    IFS5bits.ADCIF = 0;
}

void __attribute__ ((weak)) ADC1_Tasks ( void )
{
    if(IFS5bits.ADCIF == 1)
    {
        if(NULL != ADC1_CommonHandler)
        {
            (*ADC1_CommonHandler)();
        }

        // clear the ADC1 interrupt flag
        IFS5bits.ADCIF = 0;
    }
}

void ADC1_ChannelCallbackRegister(void(*callback)(enum ADC_CHANNEL channel, uint16_t adcVal))
{
    if(NULL != callback)
    {
        ADC1_ChannelHandler = callback;
    }
}

void __attribute__ ((weak)) ADC1_ChannelCallback (enum ADC_CHANNEL channel, uint16_t adcVal)
{ 
    (void)channel;
    (void)adcVal;
} 


void __attribute__ ( ( __interrupt__ , auto_psv, weak ) ) _ADCAN4Interrupt ( void )
{
    uint16_t valMCAF_ADC_DCLINK_CURRENT;
    //Read the ADC value from the ADCBUF
    valMCAF_ADC_DCLINK_CURRENT = ADCBUF4;

    if(NULL != ADC1_ChannelHandler)
    {
        (*ADC1_ChannelHandler)(MCAF_ADC_DCLINK_CURRENT, valMCAF_ADC_DCLINK_CURRENT);
    }

    //clear the MCAF_ADC_DCLINK_CURRENT interrupt flag
    IFS5bits.ADCAN4IF = 0;
}

void __attribute__ ( ( __interrupt__ , auto_psv, weak ) ) _ADCAN10Interrupt ( void )
{
    uint16_t valMCAF_ADC_PHASEC_CURRENT;
    //Read the ADC value from the ADCBUF
    valMCAF_ADC_PHASEC_CURRENT = ADCBUF10;

    if(NULL != ADC1_ChannelHandler)
    {
        (*ADC1_ChannelHandler)(MCAF_ADC_PHASEC_CURRENT, valMCAF_ADC_PHASEC_CURRENT);
    }

    //clear the MCAF_ADC_PHASEC_CURRENT interrupt flag
    IFS6bits.ADCAN10IF = 0;
}

void __attribute__ ( ( __interrupt__ , auto_psv, weak ) ) _ADCAN11Interrupt ( void )
{
    uint16_t valMCAF_ADC_POTENTIOMETER;
    //Read the ADC value from the ADCBUF
    valMCAF_ADC_POTENTIOMETER = ADCBUF11;

    if(NULL != ADC1_ChannelHandler)
    {
        (*ADC1_ChannelHandler)(MCAF_ADC_POTENTIOMETER, valMCAF_ADC_POTENTIOMETER);
    }

    //clear the MCAF_ADC_POTENTIOMETER interrupt flag
    IFS6bits.ADCAN11IF = 0;
}

void __attribute__ ( ( __interrupt__ , auto_psv, weak ) ) _ADCAN12Interrupt ( void )
{
    uint16_t valMCAF_ADC_BRIDGE_TEMPERATURE;
    //Read the ADC value from the ADCBUF
    valMCAF_ADC_BRIDGE_TEMPERATURE = ADCBUF12;

    if(NULL != ADC1_ChannelHandler)
    {
        (*ADC1_ChannelHandler)(MCAF_ADC_BRIDGE_TEMPERATURE, valMCAF_ADC_BRIDGE_TEMPERATURE);
    }

    //clear the MCAF_ADC_BRIDGE_TEMPERATURE interrupt flag
    IFS6bits.ADCAN12IF = 0;
}

void __attribute__ ( ( __interrupt__ , auto_psv, weak ) ) _ADCAN15Interrupt ( void )
{
    uint16_t valMCAF_ADC_DCLINK_VOLTAGE;
    //Read the ADC value from the ADCBUF
    valMCAF_ADC_DCLINK_VOLTAGE = ADCBUF15;

    if(NULL != ADC1_ChannelHandler)
    {
        (*ADC1_ChannelHandler)(MCAF_ADC_DCLINK_VOLTAGE, valMCAF_ADC_DCLINK_VOLTAGE);
    }

    //clear the MCAF_ADC_DCLINK_VOLTAGE interrupt flag
    IFS6bits.ADCAN15IF = 0;
}

void __attribute__ ( ( __interrupt__ , auto_psv, weak ) ) _ADCAN17Interrupt ( void )
{
    uint16_t valMCAF_ADC_PHASEA_VOLTAGE;
    //Read the ADC value from the ADCBUF
    valMCAF_ADC_PHASEA_VOLTAGE = ADCBUF17;

    if(NULL != ADC1_ChannelHandler)
    {
        (*ADC1_ChannelHandler)(MCAF_ADC_PHASEA_VOLTAGE, valMCAF_ADC_PHASEA_VOLTAGE);
    }

    //clear the MCAF_ADC_PHASEA_VOLTAGE interrupt flag
    IFS6bits.ADCAN17IF = 0;
}

void __attribute__ ( ( __interrupt__ , auto_psv, weak ) ) _ADCAN22Interrupt ( void )
{
    uint16_t valMCAF_ADC_PHASEC_VOLTAGE;
    //Read the ADC value from the ADCBUF
    valMCAF_ADC_PHASEC_VOLTAGE = ADCBUF22;

    if(NULL != ADC1_ChannelHandler)
    {
        (*ADC1_ChannelHandler)(MCAF_ADC_PHASEC_VOLTAGE, valMCAF_ADC_PHASEC_VOLTAGE);
    }

    //clear the MCAF_ADC_PHASEC_VOLTAGE interrupt flag
    IFS7bits.ADCAN22IF = 0;
}

void __attribute__ ( ( __interrupt__ , auto_psv, weak ) ) _ADCAN23Interrupt ( void )
{
    uint16_t valMCAF_ADC_PHASEB_VOLTAGE;
    //Read the ADC value from the ADCBUF
    valMCAF_ADC_PHASEB_VOLTAGE = ADCBUF23;

    if(NULL != ADC1_ChannelHandler)
    {
        (*ADC1_ChannelHandler)(MCAF_ADC_PHASEB_VOLTAGE, valMCAF_ADC_PHASEB_VOLTAGE);
    }

    //clear the MCAF_ADC_PHASEB_VOLTAGE interrupt flag
    IFS7bits.ADCAN23IF = 0;
}


void __attribute__ ( ( __interrupt__ , auto_psv, weak ) ) _ADCAN0Interrupt ( void )
{
    uint16_t valMCAF_ADC_PHASEA_CURRENT;
    //Read the ADC value from the ADCBUF
    valMCAF_ADC_PHASEA_CURRENT = ADCBUF0;

    if(NULL != ADC1_ChannelHandler)
    {
        (*ADC1_ChannelHandler)(MCAF_ADC_PHASEA_CURRENT, valMCAF_ADC_PHASEA_CURRENT);
    }

    //clear the MCAF_ADC_PHASEA_CURRENT interrupt flag
    IFS5bits.ADCAN0IF = 0;
}

void __attribute__ ( ( __interrupt__ , auto_psv, weak ) ) _ADCAN1Interrupt ( void )
{
    uint16_t valMCAF_ADC_PHASEB_CURRENT;
    //Read the ADC value from the ADCBUF
    valMCAF_ADC_PHASEB_CURRENT = ADCBUF1;

    if(NULL != ADC1_ChannelHandler)
    {
        (*ADC1_ChannelHandler)(MCAF_ADC_PHASEB_CURRENT, valMCAF_ADC_PHASEB_CURRENT);
    }

    //clear the MCAF_ADC_PHASEB_CURRENT interrupt flag
    IFS5bits.ADCAN1IF = 0;
}


void __attribute__ ((weak)) ADC1_ChannelTasks (enum ADC_CHANNEL channel)
{
    uint16_t adcVal;
    
    switch(channel)
    {   
        case MCAF_ADC_DCLINK_CURRENT:
            if((bool)ADSTATLbits.AN4RDY == 1)
            {
                //Read the ADC value from the ADCBUF
                adcVal = ADCBUF4;

                if(NULL != ADC1_ChannelHandler)
                {
                    (*ADC1_ChannelHandler)(channel, adcVal);
                }
            }
            break;
        case MCAF_ADC_PHASEC_CURRENT:
            if((bool)ADSTATLbits.AN10RDY == 1)
            {
                //Read the ADC value from the ADCBUF
                adcVal = ADCBUF10;

                if(NULL != ADC1_ChannelHandler)
                {
                    (*ADC1_ChannelHandler)(channel, adcVal);
                }
            }
            break;
        case MCAF_ADC_POTENTIOMETER:
            if((bool)ADSTATLbits.AN11RDY == 1)
            {
                //Read the ADC value from the ADCBUF
                adcVal = ADCBUF11;

                if(NULL != ADC1_ChannelHandler)
                {
                    (*ADC1_ChannelHandler)(channel, adcVal);
                }
            }
            break;
        case MCAF_ADC_BRIDGE_TEMPERATURE:
            if((bool)ADSTATLbits.AN12RDY == 1)
            {
                //Read the ADC value from the ADCBUF
                adcVal = ADCBUF12;

                if(NULL != ADC1_ChannelHandler)
                {
                    (*ADC1_ChannelHandler)(channel, adcVal);
                }
            }
            break;
        case MCAF_ADC_DCLINK_VOLTAGE:
            if((bool)ADSTATLbits.AN15RDY == 1)
            {
                //Read the ADC value from the ADCBUF
                adcVal = ADCBUF15;

                if(NULL != ADC1_ChannelHandler)
                {
                    (*ADC1_ChannelHandler)(channel, adcVal);
                }
            }
            break;
        case MCAF_ADC_PHASEA_VOLTAGE:
            if((bool)ADSTATHbits.AN17RDY == 1)
            {
                //Read the ADC value from the ADCBUF
                adcVal = ADCBUF17;

                if(NULL != ADC1_ChannelHandler)
                {
                    (*ADC1_ChannelHandler)(channel, adcVal);
                }
            }
            break;
        case MCAF_ADC_PHASEC_VOLTAGE:
            if((bool)ADSTATHbits.AN22RDY == 1)
            {
                //Read the ADC value from the ADCBUF
                adcVal = ADCBUF22;

                if(NULL != ADC1_ChannelHandler)
                {
                    (*ADC1_ChannelHandler)(channel, adcVal);
                }
            }
            break;
        case MCAF_ADC_PHASEB_VOLTAGE:
            if((bool)ADSTATHbits.AN23RDY == 1)
            {
                //Read the ADC value from the ADCBUF
                adcVal = ADCBUF23;

                if(NULL != ADC1_ChannelHandler)
                {
                    (*ADC1_ChannelHandler)(channel, adcVal);
                }
            }
            break;
        case MCAF_ADC_PHASEA_CURRENT:
            if((bool)ADSTATLbits.AN0RDY == 1)
            {
                //Read the ADC value from the ADCBUF
                adcVal = ADCBUF0;

                if(NULL != ADC1_ChannelHandler)
                {
                    (*ADC1_ChannelHandler)(channel, adcVal);
                }
            }
            break;
        case MCAF_ADC_PHASEB_CURRENT:
            if((bool)ADSTATLbits.AN1RDY == 1)
            {
                //Read the ADC value from the ADCBUF
                adcVal = ADCBUF1;

                if(NULL != ADC1_ChannelHandler)
                {
                    (*ADC1_ChannelHandler)(channel, adcVal);
                }
            }
            break;
        default:
            break;
    }            
}

void ADC1_ComparatorCallbackRegister(void(*callback)(enum ADC_CMP comparator))
{
    if(NULL != callback)
    {
        ADC1_ComparatorHandler = callback;
    }
}

void __attribute__ ((weak)) ADC1_ComparatorCallback (enum ADC_CMP comparator)
{ 
    (void)comparator;
} 



