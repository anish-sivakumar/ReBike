/* 
 * File:   mc_hall.h
 * Author: siani
 *
 * Created on June 3, 2024, 4:10 PM
 */

#ifndef RB_HALL_H
#define	RB_HALL_H

#ifdef	__cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>

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
    
    bool minSpeedReached;
    bool timedOut;
    
} RB_HALL_DATA;

/** Function Declarations */

/**
 * ISR for hall signals
 */
void RB_HALL_ISR(void);


/**
 * initializes hall object
 * @param hall
 */    
void RB_HALL_Init(RB_HALL_DATA *pHall);

/**
 * Invalidates the hall data structure
 */
void RB_HALL_InvalidateData(void);

/**
 * ISR for hall state change
 */
void RB_HALL_StateChange(RB_HALL_DATA *pHall);


/**
 * Reads PORTE Hall sensor bits
 *
 */
uint16_t RB_HALL_ValueRead(void);


/**
 * Estimates electrical angle and rotor speed using hall bits
 *
 */
int16_t RB_HALL_Estimate(RB_HALL_DATA *pHall);


 
#ifdef	__cplusplus
}
#endif

#endif	/* RB_HALL_H */

