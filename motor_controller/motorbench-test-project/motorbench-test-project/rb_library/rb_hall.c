/*
 * File:   rb_hall.c
 * Author: siani
 *
 * Created on June 3, 2024, 4:10 PM
 */

#include "rb_hall.h"
#include "rb_isr.h"

#include "../mcc_generated_files/motorBench/hal/hardware_access_functions.h"
#include "../mcc_generated_files/motorBench/util.h"

/** Global instance of the hall sensor variables */
volatile RB_HALL_DATA hall;


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


void RB_HALL_Init(void){
   
    hall.periodKFilter = Q15(0.005); //Q15(0.01);
    //hall.period = 0xFFF0; //65520
    RB_HALL_Reset();
    
    // Configure ISRs
    IO_RE8_SetInterruptHandler(&RB_HALL_ISR);
    IO_RE9_SetInterruptHandler(&RB_HALL_ISR);
    IO_RE10_SetInterruptHandler(&RB_HALL_ISR);
}

void RB_HALL_Reset(void)
{
    hall.timedOut = true;
    hall.minSpeedReached = false;
    hall.speed = 0;
    hall.period = 0xffff;
    hall.periodStateVar = 0xffffffff;
    hall.periodFilter = 0xffff;
    hall.phaseInc = 0;
    
}


void RB_HALL_StateChange(void)
{
    uint16_t tmr1_tmp = TMR1; // store timer1 count value
    TMR1 = 0; // reset timer 1 counter
    
    // Check if our TMR1 measurement was valid
    if(hall.timedOut){
        hall.timedOut = false;
    }else{
        hall.minSpeedReached = true;
        hall.period = tmr1_tmp;
    }

    hall.sector = RB_HALL_ValueRead();
    
    // Instead of making abrupt correction to the angle corresponding to hall sector, find the error and make gentle correction  
    hall.thetaError = (sectorToQ15Angle[hall.sector] + OFFSET_CORRECTION) - hall.theta;
    
    // Find the correction to be done in every step
    // If "hallThetaError" is 2000, and "hallCorrectionFactor" = (2000/8) = 250
    // So the error of 2000 will be corrected in 8 steps, with each step being 250
    hall.correctionFactor = hall.thetaError >> HALL_CORRECTION_DIVISOR;
    hall.correctionCounter = HALL_CORRECTION_STEPS;
    
    // Run this in main ISR
    //HallBasedEstimation();  
}

uint16_t RB_HALL_ValueRead(void) 
{
    uint16_t buffer; // stores PORTE bits
    uint16_t hallValue; // extracted hall sector 1-6

    buffer = PORTE; // using RE8, RE9, RE10 for Hall sensors. PORTE is 16 pins RE15..RE0
    buffer = buffer >> 8; // right shift to extract hall signals
    hallValue = buffer & 0x0007; // 0b111 mask
    
    return hallValue;
}

// called by FOC ADC interrupt
int16_t RB_HALL_Estimate(void) 
{
    if (hall.minSpeedReached){

        /* calculate filtered period = delta_period x filter_gain. 
            right shift result to remove extra 2^15 factor from fixed point multiplication*/
        hall.periodStateVar += (((int32_t)hall.period - (int32_t)hall.periodFilter)*(int16_t)(hall.periodKFilter));
        hall.periodFilter = (int16_t)(hall.periodStateVar>>15);

        hall.phaseInc = __builtin_divud((uint32_t)PHASE_INC_MULTI,(uint16_t)(hall.periodFilter));
        hall.speed = __builtin_divud((uint32_t)SPEED_MULTI,(uint16_t)(hall.periodFilter));
        hall.theta = hall.theta + hall.phaseInc;

    }
    
    int16_t thetaElectrical = hall.theta; //thetaElectrical used for FOC calculations
        if(hall.correctionCounter > 0)
        {
            hall.theta = hall.theta + hall.correctionFactor;
            hall.correctionCounter--;
        }
    return thetaElectrical;
}