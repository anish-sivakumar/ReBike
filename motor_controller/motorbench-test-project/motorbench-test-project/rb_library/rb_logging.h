/* 
 * File:   rb_logging.h
 * Author: chygg
 *
 * Created on July 14, 2024, 7:20 PM
 */

#ifndef RB_LOGGING_H
#define	RB_LOGGING_H

#include <stdint.h>

// Dividing by 2048 = 11 bit right shift
#define RB_LOGGING_DIVISOR 11

typedef struct tagRB_LOGGING_SUMS{
  int32_t vDC; // DC voltage
  int32_t iDC; // DC current
  int32_t iqRef; // q-axis current reference
  int32_t iqFdb; // q-axis current 
  int32_t power; // calculated power
  uint32_t speed; // motor speed [RPM]
  uint32_t bridgeTemp; // mosfet bridge temperature
  
  int64_t iA; // phase A current
  int64_t iB; // phase B current
  int64_t vA; // phase A voltage
  int64_t vB; // phase B voltage
}RB_LOGGING_SUMS;

typedef struct tagRB_LOGGING_DATA{
  int16_t vDC; // DC voltage
  int16_t iDC; // DC current
  int16_t iqRef; // q-axis current reference
  int16_t iqFdb; // q-axis current 
  int16_t power; // calculated power
  uint16_t speed; // motor speed [RPM]
  uint16_t bridgeTemp; // mosfet bridge temperature
  
  int16_t iA; // phase A current
  int16_t iB; // phase B current
  int16_t vA; // phase A voltage
  int16_t vB; // phase B voltage
}RB_LOGGING_AVERAGES;

/**
 * Sum values we intend to send over CAN for logging purposes
 * @param data
 * @param sums
 */
void RB_Logging_SumStepISR(RB_LOGGING_SUMS* sums, 
        int16_t vDC, int16_t iDC, int16_t iqRef, int16_t iqFdb, int16_t power, 
        uint16_t speed, uint16_t bridgeTemp, int16_t iA, int16_t iB,
        int16_t vA, int16_t vB);


/**
 * Divide summed values by number of samples to compute average
 * @param averages
 * @param sums
 */
void RB_Logging_Average(RB_LOGGING_AVERAGES* averages, const RB_LOGGING_SUMS* sums);


/**
 * Set all sums to zero
 * @param sums
 */
void RB_Logging_SumReset(RB_LOGGING_SUMS* sums);

#endif	/* RB_LOGGING_H */

