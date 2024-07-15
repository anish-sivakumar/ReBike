/* 
 * File:   rb_control.h
 * Author: siani
 *
 * Created on June 5, 2024, 2:28 PM
 */

#ifndef MC_CONTROL_H
#define	MC_CONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif
    
// these includes are OK/low-level enough, or Ours
#include <stdint.h>
#include <stdbool.h>
    
#include "rb_foc_params.h"
#include "rb_measure.h"
  
typedef struct
{
    /* increment for reference value */
    int16_t   inc;
    
    /* Target value*/
    int16_t  target;
    
    /* difference between target and reference */
    int16_t   diff;
    
    /* The rate limiting will be executed only every rampCount*/
    int16_t   rampCount;  
    
} RB_RATELIMIT;

    
 /**
  * Motor Parameters from vendor
  */   
 typedef struct tagMotorParams
 {
     int16_t    rs;                     /** Stator resistance */
     int16_t    ke;                     /** Back-EMF constant */
 } RB_MOTOR_PARAMS_T;
   
 
 typedef struct
 {
    bool    saturated;
    //testing
 } RB_CURRENT_SAT;
 
 
/**
 * Motor state data
 */
typedef struct tagPMSM
{
    RB_MOTOR_PARAMS_T           motorParams; /** Kv and Rs*/  
    
    /* Current loop command */
    //MC_DQ_T                     idqCmdRaw;  /** Input command for the current loops, prior to rate limiting */
    MC_DQ_T                     idqRef;     /** Input command for the current loops */
    
    /* Current feedback path */
    MC_ABC_T                    iabc;       /** phase current measurements - 
                                             * 12-bit ADC resolution: 2^12 = 21.83A */
    MC_ALPHABETA_T              ialphabeta; /** stationary (alphabeta) frame current measurements */
    int16_t                     i0;         /** zero-sequence current = (Ia + Ib + Ic)/3 */
    MC_DQ_T                     idqFdb;        /** rotating (dq) frame current measurements */
    
    /* Current controllers */
    MC_PISTATE_T                idCtrl;  /** controller state for the D axis */
    MC_PISTATE_T                iqCtrl;  /** controller state for the Q axis */
    RB_RATELIMIT                iqRateLim; /** rate limits Iq reference value */
    RB_CURRENT_SAT              iqSat; /* Q-axis current saturation detection */
    
    /* Current loop forward path */
    MC_DQ_T                     vdqCmd;     /** desired dq-frame voltage, output of current loop */
    MC_ALPHABETA_T              valphabetaCmd; /** desired alphabeta-frame voltage */
    MC_ABC_T                    vabcCmd;       /** desired phase voltage */
    MC_DUTYCYCLEOUT_T           pwmDutyCycle;   /** PWM count */
    MC_SINCOS_T                 sincosTheta;     /** sine and cosine of electrical angle */
  
    /** current calibration parameters */
    RB_MEASURE_CURRENT_T currentCalib;
        
    /** measured DC link voltage via ADC. Voltage divider ratio 1:21.6 feeds the ADC
     *  hence 40V DC Bus --> ((40V/21.6)/3.3V) * 2^15 = 18,388 in Q15
     */ 
    int16_t vDC;
    
    /** phase voltage measurements - at a voltage scaling ratio of 1:21.6 */
    MC_ABC_T vabc;       
    
    /** measured DC link current - from 12bit ADC, scaled to 21.83A
            ex: 96 reading in iDC, (96/2^12) * 21.83A = 0.5A*/
    int16_t iDC;
    
    /** measured MOSFET bridge temp - 3V3 ref and 10mV/10degC linear slope
     (0.15259*3.3V) / 10e-3V/C = 50degC*/
    uint16_t bridgeTemp;
    
    /** calculated power based on q-axis current*/
    int16_t torque;
    
    /** speed converted to radians per second */
    uint16_t omega;
    
    /** calculated power based on torque and speed */
    int16_t power;
    
} RB_MOTOR_DATA;


/**
 * Initialize PI controller parameters
 * @param pPMSM
 */
void RB_InitControlParameters(RB_MOTOR_DATA *pPMSM);

/**
 * Initialize PI controller and ADC compensation parameters
 * @param pPMSM
 * @return 
 */
bool RB_FocInit(RB_MOTOR_DATA *pPMSM);

/**
 * Initialize controller state
 * @param pmotor motor data
 */
inline static void RB_InitControlLoopState(RB_MOTOR_DATA *pPMSM)
{
    // D and Q axis PI controller state variables
    pPMSM->idCtrl.integrator = 0;
    pPMSM->iqCtrl.integrator = 0;
    pPMSM->vdqCmd.d = 0;
    pPMSM->vdqCmd.q = 0;
    pPMSM->idqRef.d = 0;
    pPMSM->idqRef.q = 0;
}

/**
 * Calculate current reference values from pot input
 * @param potVal
 * @param pidqRef
 */
void RB_SetCurrentReference(int16_t throttleCmd, MC_DQ_T *pidqRef, 
        RB_RATELIMIT *rateLim, bool stopped);


/**
 * Detects PI Controller saturation and adjusts reference value
 * @param piqRef
 * @param iqFdb
 * @param vqCmd
 */
bool RB_PISaturationDetect(int16_t *piqRef, int16_t iqFdb, int16_t vqCmd, int16_t speed);



#ifdef	__cplusplus
}
#endif

#endif	/* MC_CONTROL_H */