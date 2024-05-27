/**
 * PWM Generated Driver Header File 
 * 
 * @file      pwm.h
 * 
 * @ingroup   pwmhsdriver
 * 
 * @brief     This is the generated driver header file for the PWM driver
 *
 * @version   Firmware Driver Version 1.1.5
 *
 * @version   PLIB Version 2.2.0
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

#ifndef PWM_H
#define PWM_H

// Section: Included Files

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "pwm_hs_types.h"
#include "pwm_hs_interface.h"

// Section: Data Type Definitions


/**
 @ingroup  pwmhsdriver
 @brief    Structure object of type PWM_HS_INTERFACE with the 
           custom name given by the user in the Melody Driver User interface. 
           The default name e.g. PWM_HS can be changed by the 
           user in the PWM user interface. 
           This allows defining a structure with application specific name 
           using the 'Custom Name' field. Application specific name allows the 
           API Portability.
*/
extern const struct PWM_HS_INTERFACE MCC_PWM;

/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_Initialize API
 */
#define MCC_PWM_Initialize PWM_Initialize
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_Deinitialize API
 */
#define MCC_PWM_Deinitialize PWM_Deinitialize
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_Disable API
 */
#define MCC_PWM_Disable PWM_Disable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_Enable API
 */
#define MCC_PWM_Enable PWM_Enable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_MasterPeriodSet API
 */
#define MCC_PWM_MasterPeriodSet PWM_MasterPeriodSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_MasterDutyCycleSet API
 */
#define MCC_PWM_MasterDutyCycleSet PWM_MasterDutyCycleSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_MasterPhaseSet API
 */
#define MCC_PWM_MasterPhaseSet PWM_MasterPhaseSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_PeriodSet API
 */
#define MCC_PWM_PeriodSet PWM_PeriodSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_ModeSet API
 */
#define MCC_PWM_ModeSet PWM_ModeSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_DutyCycleSet API
 */
#define MCC_PWM_DutyCycleSet PWM_DutyCycleSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_PhaseSelect API
 */
#define MCC_PWM_PhaseSelect PWM_PhaseSelect
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_PhaseSet API
 */
#define MCC_PWM_PhaseSet PWM_PhaseSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_OverrideDataSet API
 */
#define MCC_PWM_OverrideDataSet PWM_OverrideDataSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_OverrideDataHighSet API
 */
#define MCC_PWM_OverrideDataHighSet PWM_OverrideDataHighSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_OverrideDataLowSet API
 */
#define MCC_PWM_OverrideDataLowSet PWM_OverrideDataLowSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_OverrideDataGet API
 */
#define MCC_PWM_OverrideDataGet PWM_OverrideDataGet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_OverrideHighEnable API
 */
#define MCC_PWM_OverrideHighEnable PWM_OverrideHighEnable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_OverrideLowEnable API
 */
#define MCC_PWM_OverrideLowEnable PWM_OverrideLowEnable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_OverrideHighDisable API
 */
#define MCC_PWM_OverrideHighDisable PWM_OverrideHighDisable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_OverrideLowDisable API
 */
#define MCC_PWM_OverrideLowDisable PWM_OverrideLowDisable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_DeadTimeLowSet API
 */
#define MCC_PWM_DeadTimeLowSet PWM_DeadTimeLowSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_DeadTimeHighSet API
 */
#define MCC_PWM_DeadTimeHighSet PWM_DeadTimeHighSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_DeadTimeSet API
 */
#define MCC_PWM_DeadTimeSet PWM_DeadTimeSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_TriggerCompareValueSet API
 */
#define MCC_PWM_TriggerCompareValueSet PWM_TriggerCompareValueSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_GeneratorInterruptEnable API
 */
#define MCC_PWM_GeneratorInterruptEnable PWM_GeneratorInterruptEnable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_GeneratorInterruptDisable API
 */
#define MCC_PWM_GeneratorInterruptDisable PWM_GeneratorInterruptDisable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_GeneratorEventStatusGet API
 */
#define MCC_PWM_GeneratorEventStatusGet PWM_GeneratorEventStatusGet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_GeneratorEventStatusClear API
 */
#define MCC_PWM_GeneratorEventStatusClear PWM_GeneratorEventStatusClear
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_GeneratorDisable API
 */
#define MCC_PWM_GeneratorDisable PWM_GeneratorDisable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_GeneratorEnable API
 */
#define MCC_PWM_GeneratorEnable PWM_GeneratorEnable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_TriggerACompareValueSet API
 */
#define MCC_PWM_TriggerACompareValueSet PWM_TriggerACompareValueSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_TriggerBCompareValueSet API
 */
#define MCC_PWM_TriggerBCompareValueSet PWM_TriggerBCompareValueSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_TriggerCCompareValueSet API
 */
#define MCC_PWM_TriggerCCompareValueSet PWM_TriggerCCompareValueSet
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_SoftwareUpdateRequest API
 */
#define MCC_PWM_SoftwareUpdateRequest PWM_SoftwareUpdateRequest
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_SoftwareUpdatePending API
 */
#define MCC_PWM_SoftwareUpdatePending PWM_SoftwareUpdatePending
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_FaultModeLatchClear API
 */
#define MCC_PWM_FaultModeLatchClear PWM_FaultModeLatchClear
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_Trigger1Enable API
 */
 #define MCC_PWM_Trigger1Enable PWM_Trigger1Enable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_Trigger1Disable API
 */
#define MCC_PWM_Trigger1Disable PWM_Trigger1Disable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_Trigger2Enable API
 */
 #define MCC_PWM_Trigger2Enable PWM_Trigger2Enable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_Trigger2Disable API
 */
#define MCC_PWM_Trigger2Disable PWM_Trigger2Disable
/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_GeneratorEOCEventCallbackRegister API
 */
#define MCC_PWM_GeneratorEOCEventCallbackRegister PWM_GeneratorEOCEventCallbackRegister

/**
 * @ingroup  pwmdriver
 * @brief    This macro defines the Custom Name for \ref PWM_GeneratorTasks API
 */
#define MCC_PWM_GeneratorTasks PWM_GeneratorTasks

// Section: PWM Module APIs

/**
 * @ingroup  pwmhsdriver
 * @brief    Initializes PWM module, using the given initialization data
 * @return   none  
 */
void PWM_Initialize(void);

/**
 * @ingroup  pwmhsdriver
 * @brief    Deinitializes the PWM to POR values
 * @return   none  
 */
void PWM_Deinitialize(void);

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function enables the specific PWM generator selected by the argument 
 *             PWM_GENERATOR.
 * @param[in]  genNum - PWM generator number  
 * @return     none  
 */
inline static void PWM_GeneratorEnable(enum PWM_GENERATOR genNum)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1CONLbits.ON = 1;              
                break;       
        case MOTOR1_PHASE_B:
                PG2CONLbits.ON = 1;              
                break;       
        case MOTOR1_PHASE_C:
                PG4CONLbits.ON = 1;              
                break;       
        default:break;    
    }     
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function disables the specific PWM generator selected by the argument 
 *             PWM_GENERATOR.
 * @param[in]  genNum - PWM generator number
 * @return     none  
 */
inline static void PWM_GeneratorDisable(enum PWM_GENERATOR genNum)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1CONLbits.ON = 0;              
                break;       
        case MOTOR1_PHASE_B:
                PG2CONLbits.ON = 0;              
                break;       
        case MOTOR1_PHASE_C:
                PG4CONLbits.ON = 0;              
                break;       
        default:break;    
    }    
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function sets the operating mode of specific PWM generator selected                  
 *             by the argument PWM_GENERATOR.
 * @param[in]  genNum - PWM generator number
 * @param[in]  mode - PWM operating mode
 * @return     none  
 */
inline static void PWM_ModeSet(enum PWM_GENERATOR genNum, enum PWM_MODES mode)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1CONLbits.MODSEL = mode; 
                if(mode == PWM_MODE_INDEPENDENT_EDGE_DUAL_OUTPUT)      
                {
                   PG1IOCONHbits.PMOD = 0x1;
                }       
                else
                {
                   PG1IOCONHbits.PMOD = 0x0;
                }
                break;       
        case MOTOR1_PHASE_B:
                PG2CONLbits.MODSEL = mode; 
                if(mode == PWM_MODE_INDEPENDENT_EDGE_DUAL_OUTPUT)      
                {
                   PG2IOCONHbits.PMOD = 0x1;
                }       
                else
                {
                   PG2IOCONHbits.PMOD = 0x0;
                }
                break;       
        case MOTOR1_PHASE_C:
                PG4CONLbits.MODSEL = mode; 
                if(mode == PWM_MODE_INDEPENDENT_EDGE_DUAL_OUTPUT)      
                {
                   PG4IOCONHbits.PMOD = 0x1;
                }       
                else
                {
                   PG4IOCONHbits.PMOD = 0x0;
                }
                break;       
        default:break;    
    }    
}

/**
 * @ingroup  pwmhsdriver
 * @brief    This inline function will enable all the generators of PWM module
 * @return   none  
 */
inline static void PWM_Enable(void)
{
    PG1CONLbits.ON = 1;              
    PG2CONLbits.ON = 1;              
    PG4CONLbits.ON = 1;              
}

/**
 * @ingroup  pwmhsdriver
 * @brief    This inline function will disable all the generators of PWM module
 * @return   none  
 */
inline static void PWM_Disable(void)
{
    PG1CONLbits.ON = 0;              
    PG2CONLbits.ON = 0;              
    PG4CONLbits.ON = 0;              
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function sets the period value in count for the Master Time Base generator
 * @param[in]  masterPeriod - Period value in count
 * @return     none  
 */
inline static void PWM_MasterPeriodSet(uint16_t masterPeriod)
{
    MPER = masterPeriod;
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function sets the PWM master duty cycle register
 * @param[in]  masterDutyCycle - Master Duty Cycle value
 * @return     none
 */
inline static void PWM_MasterDutyCycleSet(uint16_t masterDutyCycle)
{
    MDC = masterDutyCycle;
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function sets the phase value in count for the Master Time Base generator
 * @param[in]  masterPhase - Phase value in count
 * @return     none  
 */
inline static void PWM_MasterPhaseSet(uint16_t masterPhase)
{
    MPHASE = masterPhase;
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function sets the period value in count for the PWM generator specific Time Base.
 * @param[in]  genNum - PWM generator number
 * @param[in]  period - PWM generator period value in count
 * @return     none  
 */
inline static void PWM_PeriodSet(enum PWM_GENERATOR genNum,uint16_t period)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1PER = period;              
                break;       
        case MOTOR1_PHASE_B:
                PG2PER = period;              
                break;       
        case MOTOR1_PHASE_C:
                PG4PER = period;              
                break;       
        default:break;    
    }   
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function sets the PWM generator specific duty cycle register
 * @param[in]  genNum      - PWM generator number
 * @param[in]  dutyCycle   - PWM generator duty cycle
 * @return     none  
 */
inline static void PWM_DutyCycleSet(enum PWM_GENERATOR genNum,uint16_t dutyCycle)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1DC = dutyCycle;              
                break;       
        case MOTOR1_PHASE_B:
                PG2DC = dutyCycle;              
                break;       
        case MOTOR1_PHASE_C:
                PG4DC = dutyCycle;              
                break;       
        default:break;    
    }  
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function selects the PWM generator source for Phase
 * @param[in]  genNum - PWM generator number
 * @param[in]  source - PWM generator source select
 * @return     none  
 */
inline static void PWM_PhaseSelect(enum PWM_GENERATOR genNum,enum PWM_SOURCE_SELECT source)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1CONHbits.MPHSEL = source;              
                break;       
        case MOTOR1_PHASE_B:
                PG2CONHbits.MPHSEL = source;              
                break;       
        case MOTOR1_PHASE_C:
                PG4CONHbits.MPHSEL = source;              
                break;       
        default:break;    
    } 
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function sets the phase value in count for the PWM generator specific Time Base
 * @param[in]  genNum - PWM generator number
 * @param[in]  phase - PWM generator phase value in count
 * @return     none  
 */
inline static void PWM_PhaseSet(enum PWM_GENERATOR genNum,uint16_t phase)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1PHASE = phase;              
                break;       
        case MOTOR1_PHASE_B:
                PG2PHASE = phase;              
                break;       
        case MOTOR1_PHASE_C:
                PG4PHASE = phase;              
                break;       
        default:break;    
    } 
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function updates PWM override data bits with the requested value for a 
 *             specific PWM generator selected by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum          -   PWM generator number
 * @param[in]  overrideData    -   Override data  
 * @return     none  
 */
inline static void PWM_OverrideDataSet(enum PWM_GENERATOR genNum,uint16_t overrideData)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1IOCONLbits.OVRDAT = overrideData;              
                break;       
        case MOTOR1_PHASE_B:
                PG2IOCONLbits.OVRDAT = overrideData;              
                break;       
        case MOTOR1_PHASE_C:
                PG4IOCONLbits.OVRDAT = overrideData;              
                break;       
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function updates PWM override high data bit with the requested value for a 
 *             specific PWM generator selected by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum              - PWM generator number
 * @param[in]  overrideDataHigh    - Override data  
 * @return     none  
 */
inline static void PWM_OverrideDataHighSet(enum PWM_GENERATOR genNum,bool overrideDataHigh)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1IOCONLbits.OVRDAT = (PG1IOCONLbits.OVRDAT & 0x1) | ((uint8_t)overrideDataHigh << 0x1);
                break;
        case MOTOR1_PHASE_B:
                PG2IOCONLbits.OVRDAT = (PG2IOCONLbits.OVRDAT & 0x1) | ((uint8_t)overrideDataHigh << 0x1);
                break;
        case MOTOR1_PHASE_C:
                PG4IOCONLbits.OVRDAT = (PG4IOCONLbits.OVRDAT & 0x1) | ((uint8_t)overrideDataHigh << 0x1);
                break;
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function updates PWM override low data bit with the requested value for a 
 *             specific PWM generator selected by the argument \ref PWM_GENERATOR. 
 * @param[in]  genNum             - PWM generator number
 * @param[in]  overrideDataLow    - Override data  
 * @return     none  
 */
inline static void PWM_OverrideDataLowSet(enum PWM_GENERATOR genNum,bool overrideDataLow)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1IOCONLbits.OVRDAT = (PG1IOCONLbits.OVRDAT & 0x2) | overrideDataLow;
                break;  
        case MOTOR1_PHASE_B:
                PG2IOCONLbits.OVRDAT = (PG2IOCONLbits.OVRDAT & 0x2) | overrideDataLow;
                break;  
        case MOTOR1_PHASE_C:
                PG4IOCONLbits.OVRDAT = (PG4IOCONLbits.OVRDAT & 0x2) | overrideDataLow;
                break;  
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function gets PWM override value for the PWM Generator selected by the 
 *             argument \ref PWM_GENERATOR. 
 * @param[in]  genNum  -  PWM generator number
 * @return     Override data for the PWM Generator selected by the argument 
 *             PWM_GENERATOR.  
 */
inline static uint16_t PWM_OverrideDataGet(enum PWM_GENERATOR genNum)
{
    uint16_t overrideData = 0x0U;
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                overrideData = PG1IOCONLbits.OVRDAT;             
                break;
        case MOTOR1_PHASE_B:
                overrideData = PG2IOCONLbits.OVRDAT;             
                break;
        case MOTOR1_PHASE_C:
                overrideData = PG4IOCONLbits.OVRDAT;             
                break;
        default:break;    
    }
    return overrideData;
}

/**
 * @ingroup  pwmhsdriver
 * @brief    This inline function enables PWM override on PWMH output for specific PWM generator selected 
 *           by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum - PWM generator number  
 * @return   none  
 */
inline static void PWM_OverrideHighEnable(enum PWM_GENERATOR genNum)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1IOCONLbits.OVRENH = 1;              
                break;
        case MOTOR1_PHASE_B:
                PG2IOCONLbits.OVRENH = 1;              
                break;
        case MOTOR1_PHASE_C:
                PG4IOCONLbits.OVRENH = 1;              
                break;
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function enables PWM override on PWML output for specific PWM generator selected 
 *             by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum - PWM generator number
 * @return     none  
 */
inline static void PWM_OverrideLowEnable(enum PWM_GENERATOR genNum)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1IOCONLbits.OVRENL = 1;              
                break; 
        case MOTOR1_PHASE_B:
                PG2IOCONLbits.OVRENL = 1;              
                break; 
        case MOTOR1_PHASE_C:
                PG4IOCONLbits.OVRENL = 1;              
                break; 
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function disables PWM override on PWMH output for specific PWM generator selected 
 *             by the argument \ref PWM_GENERATOR.    
 * @param[in]  genNum - PWM generator number
 * @return     none  
 */
inline static void PWM_OverrideHighDisable(enum PWM_GENERATOR genNum)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1IOCONLbits.OVRENH = 0;              
                break;
        case MOTOR1_PHASE_B:
                PG2IOCONLbits.OVRENH = 0;              
                break;
        case MOTOR1_PHASE_C:
                PG4IOCONLbits.OVRENH = 0;              
                break;
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function disables PWM override on PWML output for specific PWM generator selected 
 *             by the argument \ref PWM_GENERATOR.    
 * @param[in]  genNum - PWM generator number 
 * @return     none  
 */
inline static void PWM_OverrideLowDisable(enum PWM_GENERATOR genNum)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1IOCONLbits.OVRENL = 0;              
                break;   
        case MOTOR1_PHASE_B:
                PG2IOCONLbits.OVRENL = 0;              
                break;   
        case MOTOR1_PHASE_C:
                PG4IOCONLbits.OVRENL = 0;              
                break;   
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function updates PWM Deadtime low register with the requested value for a 
 *             specific PWM generator selected by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum      - PWM generator number
 * @param[in]  deadtimeLow - Deadtime low value
 * @return     none  
 */
inline static void PWM_DeadTimeLowSet(enum PWM_GENERATOR genNum,uint16_t deadtimeLow)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1DTL = deadtimeLow;              
                break;       
        case MOTOR1_PHASE_B:
                PG2DTL = deadtimeLow;              
                break;       
        case MOTOR1_PHASE_C:
                PG4DTL = deadtimeLow;              
                break;       
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function updates PWM Deadtime high register with the requested value for a 
 *             specific PWM generator selected by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum          - PWM generator number
 * @param[in]  deadtimeHigh    - Deadtime high value
 * @return     none  
 */
inline static void PWM_DeadTimeHighSet(enum PWM_GENERATOR genNum,uint16_t deadtimeHigh)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1DTH = deadtimeHigh;              
                break;       
        case MOTOR1_PHASE_B:
                PG2DTH = deadtimeHigh;              
                break;       
        case MOTOR1_PHASE_C:
                PG4DTH = deadtimeHigh;              
                break;       
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function updates PWM Deadtime low and high register with the requested value for a 
 *             specific PWM generator selected by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum          - PWM generator number
 * @param[in]  deadtimeHigh    - Deadtime value
 * @return     none  
 */
inline static void PWM_DeadTimeSet(enum PWM_GENERATOR genNum,uint16_t deadtime)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1DTL = deadtime;
                PG1DTH = deadtime;                 
                break;       
        case MOTOR1_PHASE_B:
                PG2DTL = deadtime;
                PG2DTH = deadtime;                 
                break;       
        case MOTOR1_PHASE_C:
                PG4DTL = deadtime;
                PG4DTH = deadtime;                 
                break;       
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function sets the PWM trigger compare value in count for the PWM Generator 
 *             selected by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum          - PWM generator number
 * @param[in]  trigCompValue   - Trigger compare value in count
 * @return     none  
 */
inline static void PWM_TriggerCompareValueSet(enum PWM_GENERATOR genNum,uint16_t trigCompValue)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1TRIGA = trigCompValue;              
                break;      
        case MOTOR1_PHASE_B:
                PG2TRIGA = trigCompValue;              
                break;      
        case MOTOR1_PHASE_C:
                PG4TRIGA = trigCompValue;              
                break;      
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function enables interrupt requests for the PWM Generator selected by the 
 *             argument \ref PWM_GENERATOR.   
 * @param[in]  genNum - PWM generator number
 * @param[in]  interrupt - PWM generator interrupt source
 * @return     none  
 */
inline static void PWM_GeneratorInterruptEnable(enum PWM_GENERATOR genNum, enum PWM_GENERATOR_INTERRUPT interrupt)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        PG1EVTHbits.FLTIEN = true;               
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        PG1EVTHbits.CLIEN = true;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        PG1EVTHbits.FFIEN = true;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        PG1EVTHbits.SIEN = true;
                                        break;                                                        
                        default:break;  
                }              
                break;   
        case MOTOR1_PHASE_B:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        PG2EVTHbits.FLTIEN = true;               
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        PG2EVTHbits.CLIEN = true;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        PG2EVTHbits.FFIEN = true;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        PG2EVTHbits.SIEN = true;
                                        break;                                                        
                        default:break;  
                }              
                break;   
        case MOTOR1_PHASE_C:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        PG4EVTHbits.FLTIEN = true;               
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        PG4EVTHbits.CLIEN = true;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        PG4EVTHbits.FFIEN = true;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        PG4EVTHbits.SIEN = true;
                                        break;                                                        
                        default:break;  
                }              
                break;   
        default:break;  
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function disables interrupt requests for the PWM Generator selected by the 
 *             argument \ref PWM_GENERATOR.
 * @param[in]  genNum 	 - PWM generator number
 * @param[in]  interrupt - PWM generator interrupt source
 * @return     none  
 */
inline static void PWM_GeneratorInterruptDisable(enum PWM_GENERATOR genNum, enum PWM_GENERATOR_INTERRUPT interrupt)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        PG1EVTHbits.FLTIEN = false;               
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        PG1EVTHbits.CLIEN = false;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        PG1EVTHbits.FFIEN = false;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        PG1EVTHbits.SIEN = false;
                                        break;                                
                        default:break;  
                }              
                break;  
        case MOTOR1_PHASE_B:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        PG2EVTHbits.FLTIEN = false;               
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        PG2EVTHbits.CLIEN = false;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        PG2EVTHbits.FFIEN = false;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        PG2EVTHbits.SIEN = false;
                                        break;                                
                        default:break;  
                }              
                break;  
        case MOTOR1_PHASE_C:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        PG4EVTHbits.FLTIEN = false;               
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        PG4EVTHbits.CLIEN = false;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        PG4EVTHbits.FFIEN = false;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        PG4EVTHbits.SIEN = false;
                                        break;                                
                        default:break;  
                }              
                break;  
        default:break;  
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function clears the PWM interrupt status for the PWM Generator selected by the 
 *             argument \ref PWM_GENERATOR.   
 * @param[in]  genNum 	- PWM generator number
 * @param[in]  interrupt - PWM generator interrupt source
 * @return     none  
 */
inline static void PWM_GeneratorEventStatusClear(enum PWM_GENERATOR genNum, enum PWM_GENERATOR_INTERRUPT interrupt)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        PG1STATbits.FLTEVT = 0;                            
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        PG1STATbits.CLEVT = 0;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        PG1STATbits.FFEVT = 0;
                                        break;    
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        PG1STATbits.SEVT = 0;
                                        break;                            
                        default:break;  
                }              
                break; 
        case MOTOR1_PHASE_B:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        PG2STATbits.FLTEVT = 0;                            
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        PG2STATbits.CLEVT = 0;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        PG2STATbits.FFEVT = 0;
                                        break;    
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        PG2STATbits.SEVT = 0;
                                        break;                            
                        default:break;  
                }              
                break; 
        case MOTOR1_PHASE_C:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        PG4STATbits.FLTEVT = 0;                            
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        PG4STATbits.CLEVT = 0;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        PG4STATbits.FFEVT = 0;
                                        break;    
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        PG4STATbits.SEVT = 0;
                                        break;                            
                        default:break;  
                }              
                break; 
        default:break;  
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function gets the PWM interrupt status for the PWM Generator selected by the 
 *             argument \ref PWM_GENERATOR.   
 * @param[in]  genNum 	- PWM generator number
 * @param[in]  interrupt - PWM generator interrupt source
 * @return     true  - Interrupt is pending
 * @return     false - Interrupt is not pending
 */
inline static bool PWM_GeneratorEventStatusGet(enum PWM_GENERATOR genNum, enum PWM_GENERATOR_INTERRUPT interrupt)
{
    bool status = false;
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        status = PG1STATbits.FLTEVT;               
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        status = PG1STATbits.CLEVT;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        status = PG1STATbits.FFEVT;
                                        break;    
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        status = PG1STATbits.SEVT;
                                        break;                            
                        default:break;  
                }              
                break; 
        case MOTOR1_PHASE_B:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        status = PG2STATbits.FLTEVT;               
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        status = PG2STATbits.CLEVT;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        status = PG2STATbits.FFEVT;
                                        break;    
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        status = PG2STATbits.SEVT;
                                        break;                            
                        default:break;  
                }              
                break; 
        case MOTOR1_PHASE_C:
                switch(interrupt) { 
                        case PWM_GENERATOR_INTERRUPT_FAULT:
                                        status = PG4STATbits.FLTEVT;               
                                        break;       
                        case PWM_GENERATOR_INTERRUPT_CURRENT_LIMIT:
                                        status = PG4STATbits.CLEVT;
                                        break;
                        case PWM_GENERATOR_INTERRUPT_FEED_FORWARD:
                                        status = PG4STATbits.FFEVT;
                                        break;    
                        case PWM_GENERATOR_INTERRUPT_SYNC:
                                        status = PG4STATbits.SEVT;
                                        break;                            
                        default:break;  
                }              
                break; 
        default:break;  
    }
    return status;
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function requests to update the data registers for specific PWM generator 
 *             selected by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum - PWM generator number
 * @return     none  
 */
inline static void PWM_SoftwareUpdateRequest(enum PWM_GENERATOR genNum)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1STATbits.UPDREQ = 1;              
                break;       
        case MOTOR1_PHASE_B:
                PG2STATbits.UPDREQ = 1;              
                break;       
        case MOTOR1_PHASE_C:
                PG4STATbits.UPDREQ = 1;              
                break;       
        default:break;    
    }

}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function gets the status of the update request for specific PWM generator 
 *             selected by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum - PWM generator number
 * @return     true  - Software update is pending
 * @return     false - Software update is not pending 
 */
inline static bool PWM_SoftwareUpdatePending(enum PWM_GENERATOR genNum)
{
    bool status = false;
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                status = PG1STATbits.UPDATE;              
                break;       
        case MOTOR1_PHASE_B:
                status = PG2STATbits.UPDATE;              
                break;       
        case MOTOR1_PHASE_C:
                status = PG4STATbits.UPDATE;              
                break;       
        default:break;   
    }
    return status;
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function sets the Trigger A compare value in count for a specific PWM generator 
 *             selected by the argument \ref PWM_GENERATOR.  
 * @param[in]  genNum - PWM generator number
 * @param[in]  trigA  - Trigger A compare value in count
 * @return     none  
 */
inline static void PWM_TriggerACompareValueSet(enum PWM_GENERATOR genNum,uint16_t trigA)
{ 
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1TRIGA = trigA;              
                break;       
        case MOTOR1_PHASE_B:
                PG2TRIGA = trigA;              
                break;       
        case MOTOR1_PHASE_C:
                PG4TRIGA = trigA;              
                break;       
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function sets the Trigger B compare value in count for a specific PWM generator 
 *             selected by the argument \ref PWM_GENERATOR.   
 * @param[in]  genNum - PWM generator number
 * @param[in]  trigB  - Trigger B compare value in count
 * @return     none  
 */
inline static void PWM_TriggerBCompareValueSet(enum PWM_GENERATOR genNum,uint16_t trigB)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1TRIGB = trigB;              
                break;       
        case MOTOR1_PHASE_B:
                PG2TRIGB = trigB;              
                break;       
        case MOTOR1_PHASE_C:
                PG4TRIGB = trigB;              
                break;       
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function sets the Trigger C compare value in count for a specific PWM generator 
 *             selected by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum - PWM generator number
 * @param[in]  trigC  - Trigger C compare value in count
 * @return     none  
 */
inline static void PWM_TriggerCCompareValueSet(enum PWM_GENERATOR genNum,uint16_t trigC)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                PG1TRIGC = trigC;              
                break;       
        case MOTOR1_PHASE_B:
                PG2TRIGC = trigC;              
                break;       
        case MOTOR1_PHASE_C:
                PG4TRIGC = trigC;              
                break;       
        default:break;    
    }
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function enables ADC trigger 1 for the specific compare register 
 *             selected by the argument \ref PWM_GENERATOR.
 * @pre        Trigger value has to be set using \ref PWM_TriggerACompareValueSet, 
 *             \ref PWM_TriggerBCompareValueSet or \ref PWM_TriggerCCompareValueSet
 *             before calling this function.
 * @param[in]  genNum - PWM generator number
 * @param[in]  compareRegister - PWM generator number
 * @return     none  
 */
inline static void PWM_Trigger1Enable(enum PWM_GENERATOR genNum, enum PWM_TRIGGER_COMPARE compareRegister)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG1EVTLbits.ADTR1EN1 = 1;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG1EVTLbits.ADTR1EN2 = 1;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG1EVTLbits.ADTR1EN3 = 1;
                                        break;                           
                        default:break;  
                }              
                break;       
        case MOTOR1_PHASE_B:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG2EVTLbits.ADTR1EN1 = 1;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG2EVTLbits.ADTR1EN2 = 1;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG2EVTLbits.ADTR1EN3 = 1;
                                        break;                           
                        default:break;  
                }              
                break;       
        case MOTOR1_PHASE_C:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG4EVTLbits.ADTR1EN1 = 1;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG4EVTLbits.ADTR1EN2 = 1;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG4EVTLbits.ADTR1EN3 = 1;
                                        break;                           
                        default:break;  
                }              
                break;       
        default:break;    
    }

}  

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function disables ADC trigger 1 for the specific compare register 
 *             selected by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum - PWM generator number
 * @param[in]  compareRegister - PWM generator number
 * @return     none  
 */
inline static void PWM_Trigger1Disable(enum PWM_GENERATOR genNum, enum PWM_TRIGGER_COMPARE compareRegister)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG1EVTLbits.ADTR1EN1 = 0;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG1EVTLbits.ADTR1EN2 = 0;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG1EVTLbits.ADTR1EN3 = 0;
                                        break;                           
                        default:break;  
                }              
                break;       
        case MOTOR1_PHASE_B:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG2EVTLbits.ADTR1EN1 = 0;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG2EVTLbits.ADTR1EN2 = 0;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG2EVTLbits.ADTR1EN3 = 0;
                                        break;                           
                        default:break;  
                }              
                break;       
        case MOTOR1_PHASE_C:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG4EVTLbits.ADTR1EN1 = 0;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG4EVTLbits.ADTR1EN2 = 0;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG4EVTLbits.ADTR1EN3 = 0;
                                        break;                           
                        default:break;  
                }              
                break;       
        default:break;    
    }

}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function enables ADC trigger 2 for the specific compare register 
 *             selected by the argument \ref PWM_GENERATOR.
 * @pre        Trigger value has to be set using \ref PWM_TriggerACompareValueSet, 
 *             \ref PWM_TriggerBCompareValueSet or \ref PWM_TriggerCCompareValueSet
 *             before calling this function.
 * @param[in]  genNum - PWM generator number
 * @param[in]  compareRegister - PWM generator number
 * @return     none  
 */
inline static void PWM_Trigger2Enable(enum PWM_GENERATOR genNum, enum PWM_TRIGGER_COMPARE compareRegister)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG1EVTHbits.ADTR2EN1 = 1;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG1EVTHbits.ADTR2EN2 = 1;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG1EVTHbits.ADTR2EN3 = 1;
                                        break;                           
                        default:break;  
                }              
                break;       
        case MOTOR1_PHASE_B:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG2EVTHbits.ADTR2EN1 = 1;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG2EVTHbits.ADTR2EN2 = 1;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG2EVTHbits.ADTR2EN3 = 1;
                                        break;                           
                        default:break;  
                }              
                break;       
        case MOTOR1_PHASE_C:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG4EVTHbits.ADTR2EN1 = 1;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG4EVTHbits.ADTR2EN2 = 1;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG4EVTHbits.ADTR2EN3 = 1;
                                        break;                           
                        default:break;  
                }              
                break;       
        default:break;    
    }

}  

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function disables ADC trigger 2 for the specific compare register 
 *             selected by the argument \ref PWM_GENERATOR.
 * @param[in]  genNum - PWM generator number
 * @param[in]  compareRegister - PWM generator number
 * @return     none  
 */
inline static void PWM_Trigger2Disable(enum PWM_GENERATOR genNum, enum PWM_TRIGGER_COMPARE compareRegister)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG1EVTHbits.ADTR2EN1 = 0;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG1EVTHbits.ADTR2EN2 = 0;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG1EVTHbits.ADTR2EN3 = 0;
                                        break;                           
                        default:break;  
                }              
                break;       
        case MOTOR1_PHASE_B:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG2EVTHbits.ADTR2EN1 = 0;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG2EVTHbits.ADTR2EN2 = 0;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG2EVTHbits.ADTR2EN3 = 0;
                                        break;                           
                        default:break;  
                }              
                break;       
        case MOTOR1_PHASE_C:
                switch(compareRegister) { 
                        case PWM_TRIGGER_COMPARE_A:
                                        PG4EVTHbits.ADTR2EN1 = 0;               
                                        break;       
                        case PWM_TRIGGER_COMPARE_B:
                                        PG4EVTHbits.ADTR2EN2 = 0;
                                        break;
                        case PWM_TRIGGER_COMPARE_C:
                                        PG4EVTHbits.ADTR2EN3 = 0;
                                        break;                           
                        default:break;  
                }              
                break;       
        default:break;    
    }

}

/**
 * @ingroup    pwmhsdriver
 * @brief      This inline function clears the status of PWM latched fault mode for the PWM Generator 
 *             selected by the argument \ref PWM_GENERATOR.   
 * @param[in]  genNum - PWM generator number
 * @return     none  
 */
inline static void PWM_FaultModeLatchClear(enum PWM_GENERATOR genNum)
{
    switch(genNum) { 
        case MOTOR1_PHASE_A: 
                PG1FPCILbits.SWTERM = 1;
                break;   
        case MOTOR1_PHASE_B: 
                PG2FPCILbits.SWTERM = 1;
                break;   
        case MOTOR1_PHASE_C: 
                PG4FPCILbits.SWTERM = 1;
                break;   
        default:break;   
    }   
}

/**
 * @ingroup    pwmhsdriver
 * @brief      This function can be used to override default callback 
 *             \ref PWM_GeneratorEOCEventCallback and to define custom callback for 
 *             PWM EOCEvent event.
 * @param[in]  callback - Address of the callback function
 * @return     none  
 */
void PWM_GeneratorEOCEventCallbackRegister(void (*callback)(enum PWM_GENERATOR genNum));

/**
 * @ingroup    pwmhsdriver
 * @brief      This is the default callback with weak attribute. The user can 
 *             override and implement the default callback without weak attribute 
 *             or can register a custom callback function using PWM_EOCEventCallbackRegister.
 * @param[in]  genNum - PWM generator number
 * @return     none  
 */
void PWM_GeneratorEOCEventCallback(enum PWM_GENERATOR genNum);


/**
 * @ingroup    pwmhsdriver
 * @brief      This is a tasks function for PWM1
 * @param[in]  intGen - PWM generator number
 * @return     none  
 */
void PWM_GeneratorTasks(enum PWM_GENERATOR intGen);


#endif //PWM_H

