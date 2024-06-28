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
  
/** Different possible faults*/
typedef enum
{
    RBFAULT_NOFAULT = 0,
    RBFAULT_BRIDGE_OVERTEMP,
    RBFAULT_MOTOR_OVERTEMP,   
    RBFAULT_PHASE_OVERCURRENT,
    RBFAULT_DCBUS_OVERCURRENT,
    RBFAULT_DCBUS_OVERVOLTAGE   
} RB_FAULTS;


/** structure to store fault data */
typedef struct
{
    bool isFault;
    uint16_t faultType;
} RB_FAULT_DATA;
    
/**
 * Calibration/Compensation parameters for ADC current measurements.
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
    
    int16_t                 calibCounter;  /** counted number of samples used to calc offset */
    bool                    done; /** true when offset values have been calculated */
    
} RB_MEASURE_CURRENT_T;

/**
 * Initializes ADC scaling constants and offsets
 * @param pcalib
 */
void RB_ADCCalibrationInit(RB_MEASURE_CURRENT_T *pcalib);


/**
 * Measures average offset in ADC current reading at zero current flow
 * over multiple ISR steps. Then, saves the offsets.
 * @param pcalib
 */
void RB_ADCCalibrationStepISR(RB_MEASURE_CURRENT_T *pcalib);


/**
 * Reads raw ADC values and applies compensation
 * @param pcalib
 * @param piabc
 * @param pvDC
 */
void RB_ADCReadStepISR(RB_MEASURE_CURRENT_T *pcalib, MC_ABC_T *piabc, int16_t *pvDC);


/**
 * Apply current offset and scaling
 * @param measurement
 * @param offset
 * @param gain
 * @return 
 */
inline static int16_t RB_ADCCompensate(int16_t measurement, int16_t offset, int16_t gain)
{
    int16_t temp;
    
    /* inverted ADC and offset
     * -(measurement - offset) = offset - measurement 
     */
    
    temp = offset - measurement;
    
    /* Deciding not to scale by motorBench gain because it is confusing
     * Iabc will range from [-32768, +32768]  
     * Max value corresponds to 21.83 Apeak */
    //temp = (__builtin_mulss(temp, gain)) >> 15;
    
    return temp;
}

/**
 * 16 bit implementation of low pass filter
 * y[n] += (x[n]-y[n-1])*coeff
 *
 * @param pfilterx16 LPF state
 * @param input 1.15 fixed point
 * @return Output of low pass filter
 * 
 * coefficient for LPF given by f3db*Ts*2*pi, 
 * where f3db is cutoff frequency and Ts is sampling time
 */
inline static int16_t RB_LPF(int16_t input, int16_t prevOutput, int16_t coeff)
{
    int32_t stateVar;
    
    stateVar = (prevOutput << 15); // load 16-bit int into 32-bit int
    stateVar += __builtin_mulss(input, coeff);
    stateVar -= __builtin_mulss(prevOutput, coeff);
    
    return (int16_t)(stateVar >> 15); // remove extra 2^15 factor
    
}

void RB_FaultInit(RB_FAULT_DATA *state);

bool phaseCurrentFault(MC_ABC_T *piabc);

bool bridgeTempFault(void);

void RB_FaultCheck(RB_FAULT_DATA *pstate, MC_ABC_T *piabc);



#ifdef	__cplusplus
}
#endif

#endif	/* RB_MEASURE_H */

