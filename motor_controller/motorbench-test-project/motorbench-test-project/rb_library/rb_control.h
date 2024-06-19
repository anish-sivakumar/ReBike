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
#include "../motorBench/library/mc-library/motor_control_types.h"
#include "rb_foc_params.h"
#include "rb_measure.h"
    
// try to avoid using motorBench files
#include "../motorBench/units.h"
#include "../motorBench/foc_types.h"
#include "../motorBench/deadtimecomp_types.h"
#include "../motorBench/flux_control_types.h"
#include "../motorBench/filter_types.h"
#include "../motorbench/hal/hardware_access_functions_types.h"
#include "../motorbench/system_state.h"
#include "../motorbench/filter.h" 
//#include "../motorbench/current_measure.h"
//#include "../motorbench/foc.h"

    
 /**
  * Motor Parameters from vendor
  */   
 typedef struct tagMotorParams
 {
     int16_t    rs;                     /** Stator resistance */
     int16_t    ke;                     /** Back-EMF constant */
 } RB_MOTOR_PARAMS_T;
 
 /**
  * Interface for Hall-effect sensor based position and speed calculations
  */
 typedef struct tagHallEstimator
 {
     int16_t    thetaElectrical;         /** estimated rotor angle (electrical) */
     int16_t    omegaElectrical;        /** estimated rotor velocity (electrical) */
 } RB_HALL_ESTIMATOR_T;
     

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
    MC_ABC_T                    iabc;       /** phase current measurements */
    MC_ALPHABETA_T              ialphabeta; /** stationary (alphabeta) frame current measurements */
    int16_t                     i0;         /** zero-sequence current = (Ia + Ib + Ic)/3 */
    MC_DQ_T                     idqFdb;        /** rotating (dq) frame current measurements */
    
    /* Current controllers */
    MC_PISTATE_T                idCtrl;  /** controller state for the D axis */
    MC_PISTATE_T                iqCtrl;  /** controller state for the Q axis */
     
    /** Output limit for each axis of the current loops, normalized to DC link voltage,
     *  line-to-neutral, so that 0.57735 = 1/sqrt(3) = full line-to-line voltage */
    MC_DQ_T                     idqCtrlOutLimit; 
    int16_t                     dynLimit;  /** dynamic current limit */
    int16_t                     iqCmdLimit;  /** maximum output current amplitude, q-axis */
    
    /* Current loop forward path */
    MC_DQ_T                     vdqCmd;     /** desired dq-frame voltage, output of current loop */
    MC_ALPHABETA_T              valphabetaCmd; /** desired alphabeta-frame voltage */
    MC_ABC_T                    vabcCmd;       /** desired phase voltage */
    int16_t                     rVdc;       /** reciprocal of DC link voltage */
    MC_ABC_T                    dabc;       /** after ZSM + clip */
    MC_ABC_T                    pwmDutycycle;   /** PWM count */
    int16_t                     thetaElectrical;  /** electrical angle */
    int16_t                     omegaElectrical;  /** electrical frequency */
    MC_SINCOS_T                 sincosTheta;     /** sine and cosine of electrical angle */
    
    /** Safety Related */
    MCAF_BRIDGE_TEMPERATURE bridgeTemperature;  /** bridge temperature */
    MCAF_FAULT_DETECT_T faultDetect;     /** fault detect state */
  
    /** current calibration parameters */
    RB_MEASURE_CURRENT_T currentCalib;
        
    /** measured DC link voltage */
    int16_t vDC;
    
    /** measured DC link current*/
    int16_t iDC;
    
} RB_MOTOR_DATA;


void RB_InitControlParameters(RB_MOTOR_DATA *pPMSM);

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
void RB_SetCurrentReference(uint16_t potVal, MC_DQ_T *pidqRef);

/**
 * Step PI controller 
 * @param idqFdb
 * @param idqRef
 * @param pidCtrl
 * @param pvdqCmd
 */
void RB_ControlStepISR(MC_DQ_T idqFdb, MC_DQ_T idqRef, MC_PISTATE_T *pidCtrl, 
        MC_DQ_T *pvdqCmd);

#ifdef	__cplusplus
}
#endif

#endif	/* MC_CONTROL_H */