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
#include "mcc_generated_files/motorBench/mcaf_main.h"
#include "mcc_generated_files/system/system.h"
#include "mcc_generated_files/system/pins.h"
#include "mcc_generated_files/motorBench/hal/hardware_access_functions.h"
#include "mcc_generated_files/motorBench/util.h"


/*
    Main application
*/

/** Definitions */
/* The count corresponding to 60degree angle i.e, 32768/3  */
#define SECTOR_ANGLE    10922  

/*Offset between the zero crossing sector and the Phase A 
 * current zero crossing    */
#define OFFSET_CORRECTION   (-4000)

/*Hall angle correction divisor for correcting angle difference 
 *  between the sectors     */
#define HALL_CORRECTION_DIVISOR 3

/*Number of steps in which the Hall sensor angle correction is done
 i.e,2^HALL_CORRECTION_DIVISOR  */
#define HALL_CORRECTION_STEPS   8 


#define PWMFREQUENCY_HZ         20000
#define FOSC_OVER_2             100000000
#define TIMER1_PRESCALER        64

/**  SPEED MULTIPLIER CALCULATION = ((FOSC*60)/(TIMER_PRESCALER*6))
 * This is to calculate speed in electrical RPM  */
#define SPEED_MULTI     (unsigned long)((float)(FOSC_OVER_2/(float)(TIMER1_PRESCALER*6)))*(float)(60)    

/** PHASE INCREMENT MULTIPLIER = (FCY/(TIMER_PRESCALER*PWM_FREQUENCY))(65536/6)*/
#define PHASE_INC_MULTI    (unsigned long)((float)FOSC_OVER_2/((float)(TIMER1_PRESCALER)*(float)(PWMFREQUENCY_HZ))*(float)(65536/6))



// Hall signal variables
volatile int16_t thetaElectrical = 0;
typedef struct
{
    uint16_t timerValue;
    uint16_t sector;
    uint16_t speed;
    /* angle of estimation */
    int16_t theta;    

    int16_t period;
    int32_t periodStateVar;
    int16_t periodFilter;
    int16_t periodKFilter;
    int16_t phaseInc;
    int16_t correctHallTheta;
    int16_t thetaError;
    int16_t halfThetaCorrection;
    int16_t correctionFactor;
    int16_t correctionCounter;
} struct_HallData;
struct_HallData hallData;

// Hall interrupt declarations
void HallStateChange(void);
uint16_t HallValueRead(void);
void HallBasedEstimation(void);


volatile uint16_t TMR1_testing = 0;
int main(void)
{
    SYSTEM_Initialize();
    MCAF_MainInit();
    
    // Configure Hall ISRs
    IO_RE8_SetInterruptHandler(&HallStateChange);
    IO_RE9_SetInterruptHandler(&HallStateChange);
    IO_RE10_SetInterruptHandler(&HallStateChange);

    // hall data initialization
    hallData.periodKFilter = UTIL_MulQ15(0.001, 1);
    hallData.period = 0xFFF0; //65520
    
    while(1)
    {
        MCAF_MainLoop();
        //TMR1_testing = TMR1;
    }    
}

/******************************************************************************
 * Description: The calculation of six sector to equivalent angle in Q15 format.
 *              Here the angle is varying from -32768 to 32767. The 
 *****************************************************************************/
int16_t sectorToAngle[8] =  // 3, 2, 6, 4, 5, 1
{
    0,
    21845,      // sector-1 =
    -21864,     // sector-2 = (-32768+10922)
    -32768,     // sector-3
    0,          // sector-4
    10922,      // sector-5
    -10924,     //sector-6
    0
};

uint16_t HallValueRead(void) //defined in board_service.c in github sample project
{
    uint16_t buffer;
    uint16_t hallValue;

    buffer = PORTE; // using RE8, RE9, RE10 for Hall sensors. PORTE is 16 pins RE15..RE0
    buffer = buffer >> 8;
    hallValue = buffer & 0x0007; // 0b111 mask
    
    return hallValue;
}

void HallStateChange(void) 
{
    TMR1_testing = PORTEbits.RE8;
    
    
    hallData.timerValue = TMR1;
    
    
    TMR1 = 0;
    hallData.period = hallData.timerValue;    
    
    hallData.sector = HallValueRead();
    
    
    // Instead of making abrupt correction to the angle corresponding to hall sector, find the error and make gentle correction  
    hallData.thetaError = (sectorToAngle[hallData.sector] + OFFSET_CORRECTION) - hallData.theta;
    // Find the correction to be done in every step
    // If "hallThetaError" is 2000, and "hallCorrectionFactor" = (2000/8) = 250
    // So the error of 2000 will be corrected in 8 steps, with each step being 250
    hallData.correctionFactor = hallData.thetaError >> HALL_CORRECTION_DIVISOR;
    hallData.correctionCounter = HALL_CORRECTION_STEPS;
    
    //HallBasedEstimation(); 
    
}

/** Hall effect based speed and position estimation 
  this should be in main control ISR later    */
void HallBasedEstimation(void) 
{
    // 1. Set reference values
    // 2. Update PI controller loops
    
    // 3. Hall based position and speed calculations
    hallData.periodStateVar += (((long int)hallData.period - (long int)hallData.periodFilter)*(int)(hallData.periodKFilter));
    hallData.periodFilter = (int)(hallData.periodStateVar>>15);
    hallData.phaseInc = __builtin_divud((uint32_t)PHASE_INC_MULTI,(unsigned int)(hallData.periodFilter));
    hallData.speed = __builtin_divud((uint32_t)SPEED_MULTI,(unsigned int)(hallData.periodFilter));
    
    thetaElectrical = hallData.theta; //thetaElectrical used for FOC calcs
    hallData.theta = hallData.theta + hallData.phaseInc; 
    if(hallData.correctionCounter > 0)
    {
        hallData.theta = hallData.theta + hallData.correctionFactor;
        hallData.correctionCounter--;
    }
    
    // 4. Calculate sine and cosine theta
    // 5. Calculate inverse Park
    // 6. Calculate inverse Clarke - swapped input
    // 7. Calculate SVPWM sector and Duties
    // 8. Update PWM Duties
    
}