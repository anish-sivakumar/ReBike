/*
 * File:   rb_hall.c
 * Author: siani
 *
 * Created on June 3, 2024, 4:10 PM
 */

#include "rb_hall.h"
#include "rb_isr.h"

#include "timer/sccp4.h"
#include "hal/hardware_access_functions.h"
#include "motorBench/util.h"


/**
 * LUT for the calculation of six sector to equivalent angle in Q15 format.
 *              Here the angle is varying from -32768 to 32767 => -pi t +pi
 */
int16_t sectorToQ15Angle[8] =  // 3, 2, 6, 4, 5, 1 is the fwd order from hall signals 
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


void RB_HALL_Init(RB_HALL_DATA *phall){
   
    phall->periodKFilter = Q15(0.001); //Q15(0.002); //Q15(0.01);
    phall->startupCounter = 0;
    phall->sector = RB_HALL_ValueRead();
    
    //hall.period = 0xFFF0; //65520
    RB_HALL_Reset(phall);
    
    // Configure ISRs
    IO_RE8_SetInterruptHandler(&RB_HALL_ISR);
    IO_RE9_SetInterruptHandler(&RB_HALL_ISR);
    IO_RE10_SetInterruptHandler(&RB_HALL_ISR);
    
    SCCP4_Timer_Stop();
    SCCP4_Timer_PeriodSet(RB_HALL_TMR4_PERIOD);
    SCCP4_Timer_TimeoutCallbackRegister(&RB_HALL_TIMEOUT_ISR);
    SCCP4_Timer_Start();
}

uint16_t RB_HALL_NextSector(uint16_t prev){
    uint16_t next;
    
    // Switching order of hall signals is 3,2,6,4,5,1
    switch (prev)
    {
        case 3:
            next = 2;
            break;
        case 2:
            next = 6;
            break;
        case 6:
            next = 4;
            break;
        case 4:
            next = 5;
            break;
        case 5:
            next = 1;
            break;
        case 1:
            next = 3;
            break;
        default: // Should never happen
            next = 0;
            break;
    }
    return next;
}

void RB_HALL_Reset(RB_HALL_DATA *phall)
{
    phall->startupCounter = 0;
    phall->minSpeedReached = false;
    phall->speed = 0;
    phall->period = 0xffff;
    phall->periodStateVar = 0xffffffff;
    phall->periodFilter = 0xffff;
    phall->phaseInc = 0;
    
            
}


void RB_HALL_StateChange(RB_HALL_DATA *phall)
{
    // store timer value
    uint16_t tmr_tmp = (uint16_t)SCCP4_Timer_Counter16BitGet(); 
    uint16_t sector_tmp = RB_HALL_ValueRead();
    
    // set period at some point during startup
    if (phall->startupCounter == 30)
    {
        phall->period = tmr_tmp;     
    }
    
    uint16_t min_acceptable_tmr = __builtin_mulus(phall->period, Q15(0.5)) >> 15; 
    uint32_t max_acceptable_tmr = phall->period + min_acceptable_tmr;
    
    /* Sector 5 is being missed. It's possible it's noisy.
     *  this makes the timer value really low (~80), and period very low
     *  which causes period filter to oscillate, and theta to be messed up,
     *  which makes controls bad. 
     * 
     * if hall is noisy, we can check if timer value is reasonable before assinging sector
     * --> we checked on the scope, and hall signal gets more noisy as speed increases
     * --> need to debounce to make sure the interrupted state change is a real one
     */
    
    // Some noise is causing the hall ISR to run more often that it should. 
    // Only run the state change routine if we actually saw a change in the hall sector.
    // TODO: try to debouce this better. predicting what sector to expect next did not work for debouncing.
    if (sector_tmp == RB_HALL_NextSector(phall->sector)) {
//    if (sector_tmp != phall->sector) {
        // reset timer - only want to this when we have a state change we trust
        SCCP4_Timer_Stop();
        CCP4TMRL = 0;
        SCCP4_Timer_Start();
        
        phall->sector = sector_tmp;
        
        // only use timer count as period, if it's reasonable
        // if it's not, period will remain last reasonable value
        if ((tmr_tmp > min_acceptable_tmr) && (tmr_tmp < max_acceptable_tmr))
        {
            phall->period = tmr_tmp;
        }
        
        // count up 78 hall states, and check that timer doesn't overflow during that time
        if(phall->startupCounter < 78){
            phall->startupCounter++;
        }else{ // startupCounter will remain expired, and reset on timer overflow
            phall->minSpeedReached = true;
        }
        
        // Instead of making abrupt correction to the angle corresponding to hall sector, find the error and make gentle correction  
        phall->thetaError = (sectorToQ15Angle[phall->sector] + OFFSET_CORRECTION) - phall->theta;

        // Find the correction to be done in every step
        // If "hallThetaError" is 2000, and "hallCorrectionFactor" = (2000/8) = 250
        // So the error of 2000 will be corrected in 8 steps, with each step being 250
        phall->correctionFactor = phall->thetaError >> HALL_CORRECTION_DIVISOR;
        phall->correctionCounter = HALL_CORRECTION_STEPS;
    }
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


void RB_HALL_Estimate(RB_HALL_DATA *phall) 
{    
    /* calculate filtered period = delta_period x filter_gain. 
        right shift result to remove extra 2^15 factor from fixed point multiplication*/
    phall->periodStateVar += (((int32_t)phall->period - (int32_t)phall->periodFilter)*(int16_t)(phall->periodKFilter));
    phall->periodFilter = (int16_t)(phall->periodStateVar>>15);

    phall->phaseInc = __builtin_divud((uint32_t)PHASE_INC_MULTI,(uint16_t)(phall->periodFilter));
    phall->speed = __builtin_divud((uint32_t)SPEED_MULTI,(uint16_t)(phall->periodFilter));
    phall->theta = phall->theta + phall->phaseInc;


    if(phall->correctionCounter > 0)
    {
        phall->theta = phall->theta + phall->correctionFactor;
        phall->correctionCounter--;
    }
}