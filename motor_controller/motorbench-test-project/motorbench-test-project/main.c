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
    uint16_t sector; // sector of [3, 2, 6, 4, 5, 1] during 1 electrical cycle
    uint16_t speed; // rotor speed in RPM using filtered period 
    int16_t theta; // angle of estimation

    uint16_t period; // captures raw timer1 value 
    uint32_t periodStateVar; // intermediate result for filtered period calculation
    uint16_t periodFilter; // filtered period using moving average filtering method
    uint16_t periodKFilter; // period filter gain
    uint16_t phaseInc; // phase increment value for subsequent electrical angle
    //int16_t correctHallTheta; not used
    int16_t thetaError; // calculated error in electrical angle using OFFSET_CORRECTION
    // int16_t halfThetaCorrection; not used
    int16_t correctionFactor; // adjustment to electrical angle every ISR run
    int16_t correctionCounter; // counts the number of ISR runs to correct electrical angle over to avoid abrupt changes
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
    hallData.periodKFilter = Q15(0.01);//Q15(0.001);
    hallData.period = 0xFFF0; //65520
    
    while(1)
    {
        MCAF_MainLoop();
        HallBasedEstimation(); 
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
    uint16_t buffer; // stores PORTE bits
    uint16_t hallValue; // extracted hall sector 1-6

    buffer = PORTE; // using RE8, RE9, RE10 for Hall sensors. PORTE is 16 pins RE15..RE0
    buffer = buffer >> 8; // right shift to extract hall signals
    hallValue = buffer & 0x0007; // 0b111 mask
    
    return hallValue;
}

void HallStateChange(void) 
{
    hallData.period = TMR1; // timer1 count value
    TMR1 = 0; // reset timer 1 counter
    
    hallData.sector = HallValueRead();
    
    // Instead of making abrupt correction to the angle corresponding to hall sector, find the error and make gentle correction  
    hallData.thetaError = (sectorToAngle[hallData.sector] + OFFSET_CORRECTION) - hallData.theta;
    
    // Find the correction to be done in every step
    // If "hallThetaError" is 2000, and "hallCorrectionFactor" = (2000/8) = 250
    // So the error of 2000 will be corrected in 8 steps, with each step being 250
    hallData.correctionFactor = hallData.thetaError >> HALL_CORRECTION_DIVISOR;
    hallData.correctionCounter = HALL_CORRECTION_STEPS;
    
    // Run this in main ISR
    HallBasedEstimation(); 
    
}

/** Hall effect based speed and position estimation */
void HallBasedEstimation(void) 
{
    // 1. Set reference values (main ISR)
    // 2. Update PI controller loops (main ISR)
    
    // 3. Hall based position and speed calculations
    
    /* calculate filtered period = delta_period x filter_gain. 
        right shift result to remove extra 2^15 factor from fixed point multiplication*/
    hallData.periodStateVar += (((int32_t)hallData.period - (int32_t)hallData.periodFilter)*(int16_t)(hallData.periodKFilter));
    hallData.periodFilter = (int16_t)(hallData.periodStateVar>>15);
    
    
    hallData.phaseInc = __builtin_divud((uint32_t)PHASE_INC_MULTI,(uint16_t)(hallData.periodFilter));
    hallData.speed = __builtin_divud((uint32_t)SPEED_MULTI,(uint16_t)(hallData.periodFilter));
    
    thetaElectrical = hallData.theta; //thetaElectrical used for FOC calculations
    hallData.theta = hallData.theta + hallData.phaseInc;
    if(hallData.correctionCounter > 0)
    {
        hallData.theta = hallData.theta + hallData.correctionFactor;
        hallData.correctionCounter--;
    }
    
    // 4. Calculate sine and cosine theta (main ISR)
    // 5. Calculate inverse Park (main ISR)
    // 6. Calculate inverse Clarke - swapped input (main ISR)
    // 7. Calculate SVPWM sector and Duties (main ISR)
    // 8. Update PWM Duties (main ISR)
    
}