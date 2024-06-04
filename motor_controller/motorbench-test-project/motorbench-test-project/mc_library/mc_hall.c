/*
 * File:   mc_hall.c
 * Author: siani
 *
 * Created on June 3, 2024, 4:10 PM
 */

#include "../mcc_generated_files/motorBench/hal/hardware_access_functions.h"
#include "mc_hall.h"
#include "../mcc_generated_files/motorBench/util.h"

/**
 * LUT for the calculation of six sector to equivalent angle in Q15 format.
 *              Here the angle is varying from -32768 to 32767. 
 */
int16_t sectorToQ15Angle[8] =  // 3, 2, 6, 4, 5, 1
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
extern MC_HALL_DATA hall;

void MC_HALL_Init(MC_HALL_DATA *pHall){
   
    pHall->periodKFilter = Q15(0.01);
    pHall->period = 0xFFF0; //65520
}

void MC_HALL_ISR(void){
    MC_HALL_StateChange(&hall);
}

void MC_HALL_StateChange(MC_HALL_DATA *pHall)
{
    pHall->period = TMR1; // timer1 count value
    TMR1 = 0; // reset timer 1 counter
    
    pHall->sector = MC_HALL_ValueRead();
    
    // Instead of making abrupt correction to the angle corresponding to hall sector, find the error and make gentle correction  
    pHall->thetaError = (sectorToQ15Angle[pHall->sector] + OFFSET_CORRECTION) - pHall->theta;
    
    // Find the correction to be done in every step
    // If "hallThetaError" is 2000, and "hallCorrectionFactor" = (2000/8) = 250
    // So the error of 2000 will be corrected in 8 steps, with each step being 250
    pHall->correctionFactor = pHall->thetaError >> HALL_CORRECTION_DIVISOR;
    pHall->correctionCounter = HALL_CORRECTION_STEPS;
    
    // Run this in main ISR
    //HallBasedEstimation();  
}

uint16_t MC_HALL_ValueRead(void) 
{
    uint16_t buffer; // stores PORTE bits
    uint16_t hallValue; // extracted hall sector 1-6

    buffer = PORTE; // using RE8, RE9, RE10 for Hall sensors. PORTE is 16 pins RE15..RE0
    buffer = buffer >> 8; // right shift to extract hall signals
    hallValue = buffer & 0x0007; // 0b111 mask
    
    return hallValue;
}

// called by FOC ADC interrupt
int16_t MC_HALL_Estimate(MC_HALL_DATA *pHall) 
{
    int16_t thetaElectrical = 0;
    
    /* calculate filtered period = delta_period x filter_gain. 
        right shift result to remove extra 2^15 factor from fixed point multiplication*/
    pHall->periodStateVar += (((int32_t)pHall->period - (int32_t)pHall->periodFilter)*(int16_t)(pHall->periodKFilter));
    pHall->periodFilter = (int16_t)(pHall->periodStateVar>>15);
    
    pHall->phaseInc = __builtin_divud((uint32_t)PHASE_INC_MULTI,(uint16_t)(pHall->periodFilter));
    pHall->speed = __builtin_divud((uint32_t)SPEED_MULTI,(uint16_t)(pHall->periodFilter));
    
    thetaElectrical = pHall->theta; //thetaElectrical used for FOC calculations
    pHall->theta = pHall->theta + pHall->phaseInc;
    if(pHall->correctionCounter > 0)
    {
        pHall->theta = pHall->theta + pHall->correctionFactor;
        pHall->correctionCounter--;
    }
    
    return thetaElectrical;
    
}

