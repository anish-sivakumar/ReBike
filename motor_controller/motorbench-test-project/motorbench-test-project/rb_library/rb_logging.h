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
  int32_t iA; // phase A current
  int32_t iB; // phase B current
  int32_t vA; // phase A voltage
  int32_t vB; // phase B voltage
  uint32_t speed; // motor speed [RPM]
  int32_t iqRef; // q-axis current reference
  int32_t iqFdb; // q-axis current 
  uint32_t temp_fet; // mosfet bridge temperature
  int32_t power; // calculated power
}RB_LOGGING_SUMS;

typedef struct tagRB_LOGGING_AVGS{
  int16_t vDC; // DC voltage
  int16_t iDC; // DC current
  int16_t iA; // phase A current
  int16_t iB; // phase B current
  int16_t vA; // phase A voltage
  int16_t vB; // phase B voltage
  uint16_t speed; // motor speed [RPM]
  int16_t iqRef; // q-axis current reference
  int16_t iqFdb; // q-axis current 
  uint16_t temp_fet; // mosfet bridge temperature
  int16_t power; // calculated power
}RB_LOGGING_AVGS;

RB_Logging_SumStep(RB_LOGGING_SUMS* sums);
RB_Logging_Averaging(RB_LOGGING_AVGS* averages, const RB_LOGGING_SUMS* sums);
RB_Logging_SumReset(RB_LOGGING_SUMS* sums);

#endif	/* RB_LOGGING_H */

