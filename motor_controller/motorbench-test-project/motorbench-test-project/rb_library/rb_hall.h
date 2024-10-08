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
#include "rb_foc_params.h"

/** Definitions */
/* The count corresponding to 60degree angle i.e, 32768/3  */
#define SECTOR_ANGLE    10922  

/*Offset between the zero crossing sector and the Phase A 
 * current zero crossing    */
#define OFFSET_CORRECTION   -10000 // (-4000)

/*Hall angle correction divisor for correcting angle difference 
 *  between the sectors     */
#define HALL_CORRECTION_DIVISOR 5 //old:3

/*Number of steps in which the Hall sensor angle correction is done
 i.e,2^HALL_CORRECTION_DIVISOR  */
#define HALL_CORRECTION_STEPS   32 //old:8, 
/* this is only effective in between hall sector changes, which depends on speed
 * if hall sector changes every 5ms, and theta can be corrected every 50us,
 *      we can correct over 100 steps
 */


#define PWM_FREQ_HZ         20000
#define TIMER4_FREQ_HZ          100000000
#define TIMER4_PRESCALER        64
// Corresponds to a timer period of roughly 40ms
#define RB_HALL_TMR4_PERIOD     0xFFFF
    

/**  SPEED MULTIPLIER CALCULATION - 
 * Rotor speed in RPM = [fraction of revolution b/w hall changes] / [time b/w hall changes]
 * = { [(1/(POLEPAIRS*6)] / [TMR1/(FOSC_OVER_2/TIMER1_PRESCALER)] } * 60s/min
 * Hence, SPEED_MULTI = ((FOSC_OVER_2*60)/(TIMER_PRESCALER*6*POLEPAIRS))
 */
#define SPEED_MULTI     (unsigned long)((float)(TIMER4_FREQ_HZ/(float)(TIMER4_PRESCALER*6*RB_MOTOR_POLEPAIRS)))*(float)(60)    

/** PHASE INCREMENT MULTIPLIER - Amount phase increases in 20kHz ISR step
 * = [increase in phase per hall change] * [ISR step time / hall change time]
 * = [360/6 degrees] * [hall change frequency / ISR frequency]
 * = [65536/6] * [((FOSC_OVER_2/TIMER_PRESCALER)/TMR1) / (PWM_FREQUENCY)]
 * HENCE, PHASE_INC_MULTI = (FOSC_OVER_2/(TIMER_PRESCALER*PWM_FREQUENCY))(65536/6)*/
#define PHASE_INC_MULTI    (unsigned long)((float)TIMER4_FREQ_HZ/((float)(TIMER4_PRESCALER)*(float)(PWM_FREQ_HZ))*(float)(65536/6))

typedef struct
{
    uint16_t sector; // sector of [3, 2, 6, 4, 5, 1] during 1 electrical cycle
    uint16_t speed; // rotor speed in RPM using filtered period 
    int16_t theta; // angle of estimation: Q15 -32768 to 32767 => -pi t +pi

    uint16_t period; // captures raw timer4 value 
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
    uint16_t startupCounter; // ensures there is no timer timeout during 78 hall states (half revolution)

    
} RB_HALL_DATA;

/** Function Declarations */

/**
 * initializes hall object
 * @param phall
 */    
void RB_HALL_Init(RB_HALL_DATA *phall);

/**
 * Uses the previous hall sector to predict the next sector.
 * @param prev: previous hall sector
 * @return: next hall sector
 */
uint16_t RB_HALL_NextSector(uint16_t prev);
/**
 * Invalidates the hall data structure
 * @param phall
 */
void RB_HALL_Reset(RB_HALL_DATA *phall);

/**
 * ISR for hall state change
 * @param phall
 */
void RB_HALL_StateChange(RB_HALL_DATA *phall);


/**
 * Reads PORTE Hall sensor bits
 *
 */
uint16_t RB_HALL_ValueRead(void);


/**
 * Estimates electrical angle and rotor speed using hall bits
 * @param phall
 */
void RB_HALL_Estimate(RB_HALL_DATA *phall);


 
#ifdef	__cplusplus
}
#endif

#endif	/* RB_HALL_H */

