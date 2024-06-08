/* 
 * File:   rb_measure.h
 * Author: siani
 *
 * Created on June 8, 2024, 12:05 PM
 */

#include <stdint.h>
#include <stdbool.h>
#include "../motorBench/library/mc-library/motor_control_types.h"

#ifndef RB_MEASURE_H
#define	RB_MEASURE_H

#ifdef	__cplusplus
extern "C" {
#endif

/*
 * Current compensation gains
 *
 * Ia = KAA*Ia_measured + KAB*Ib_measured
 * Ib = KBA*Ia_measured + KBB*Ib_measured
 *
 * Ic = KCC*Ic_measured (if applicable)
 *
 */
/* normalized A->A current gain */
#define C_KAA                         16384      // Q14(  1.00000) =   +1.00000             =   +1.00000             + 0.0000%   
/* normalized B->A current gain */
#define C_KAB                             0      // Q14(  0.00000) =   +0.00000             =   +0.00000             + 0.0000%   
/* normalized A->B current gain */
#define C_KBA                             0      // Q14(  0.00000) =   +0.00000             =   +0.00000             + 0.0000%   
/* normalized B->B current gain */
#define C_KBB                         16384      // Q14(  1.00000) =   +1.00000             =   +1.00000             + 0.0000%
/* normalized C->C current gain */
#define C_KCC                         16384      // Q14(  1.00000) =   +1.00000             =   +1.00000             + 0.0000%
/* scaling for Idc */
#define IDC_SCALING_FACTOR             16384      // Q14(  1.00000) =   +1.00000             =   +1.00000             + 0.0000%

#define CURRENT_OFFSET_COUNT_BITS   (int16_t)   10
#define CURRENT_OFFSET_COUNT_MAX    (int16_t)(1 << CURRENT_OFFSET_COUNT_BITS)
    
/* scaling for Vdc */
#define VDC_SCALING_FACTOR              1024      // Q10(  1.00000) =   +1.00000             =   +1.00000             + 0.0000%
#define VDC_SCALING_FACTOR_Q              10
  
    
/* Bridge Temp parameters  
 * 
 * 
 * 
 */
    
/* temperature gain */
#define RB_BRIDGE_TEMPERATURE_GAIN        33000      // Q16(  0.50354) = +503.54004 m           = +503.54000 m           + 0.0000%
#define RB_BRIDGE_TEMPERATURE_OFFSET            5000 //            temperature offset
/* Pole of bridge temperature low-pass filter */
#define RB_BRIDGE_TEMPERATURE_FILTER_GAIN        328      // Q16(  0.00500) = +100.09766 rad/s       = +100.00000 rad/s       + 0.0977%
/* Maximum temperature slew rate */
#define RB_BRIDGE_TEMPERATURE_SLEW_RATE       1311      // Q15(  0.04001) =   +4.00085 C/s         =   +4.00000 C/s         + 0.0214%
/* Pole of voltage loop low-pass filter */
#define RB_FILTER_COEFF_VQ                 1638      // Q16(  0.02499) = +499.87793 rad/s       = +500.00000 rad/s       - 0.0244%
      
    
    
    
/**
 * Compensation for current measurements.
 * 
 * These are applied as follows:
 * If Ia[0] and Ib[0] are the original ADC measurements, we calculate:
 * (note: sign is correct since our current sensing gain into the ADC is negative)
 * 
 * Ia[1] = -(Ia[0] - offseta)
 * Ib[1] = -(Ib[0] - offsetb)
 * 
 * Ia[final] = qKaa * Ia[1] + qKab * Ib[1]
 * Ib[final] = qKba * Ia[1] + qKbb * Ib[1]
 * 
 * The cross-terms are important to correct cross-coupling of the ADC measurements.
 */
typedef struct
{
    int16_t                 rawIa;      /** raw phase A current from ADC */ 
    int16_t                 qKaa;        /** Q15 phase A gain */
    int16_t                 qKab;        /** Q15 cross-coupling gain B->A */
    int32_t                 sumIa;       /* Accumulation of Ia to calculate offset*/
    int16_t                 offsetIa;     /** phase A offset */
    
    int16_t                 rawIb;      /** raw phase B current from ADC */
    int16_t                 qKba;        /** Q15 cross-coupling gain A->B */
    int16_t                 qKbb;        /** Q15 phase B gain */
    int32_t                 sumIb;       /* Accumulation of Ib to calculate offset*/
    int16_t                 offsetIb;     /** phase B offset */
    
    int16_t                 rawIdc;      /** raw DC current from ADC */
    int16_t                 qKidc;       /** Q15 DC link gain */
    int32_t                 sumIdc;        /* Accumulation of Idc to calculate offset*/
    int16_t                 offsetIdc;   /** DC link offset */
    
    int16_t                 calibrationCounter;  /** counted number of samples used to calc offset */
    int16_t                 calibrationComplete; /** 1 when offset values have been calculated */
    
} RB_MEASURE_CURRENT_T;

void RB_ADCCompensationInit(RB_MEASURE_CURRENT_T *pcal);

void RB_ADCRead(RB_MEASURE_CURRENT_T *pcal, MC_ABC_T *piabc, int16_t *pvDC);

inline static int16_t RB_ADCProcess(int16_t measurement, int16_t offset)
{
    
    // inverted ADC and offset
    {
        return offset - measurement;
    }
    
    // scale 
    
}

#ifdef	__cplusplus
}
#endif

#endif	/* RB_MEASURE_H */

