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
    /* Current loop command */
    MC_DQ_T                     idqCmdRaw;  /** Input command for the current loops, prior to rate limiting */
//    MC_DQ_T                     idqCmdPerturbed; /** Input command for the current loops, prior to rate limiting */
    MC_DQ_T                     idqCmd;     /** Input command for the current loops */

    /* Current feedback path */
    MC_ABC_T                    iabc;       /** phase current measurements */
    MC_ALPHABETA_T              ialphabeta; /** stationary (alphabeta) frame current measurements */
    int16_t                     i0;         /** zero-sequence current = (Ia + Ib + Ic)/3 */
    int16_t                     idq;        /** rotating (dq) frame current measurements */

    /* Current controllers */
    MC_PISTATE_T                idCtrl;  /** controller state for the D axis */
    MC_PISTATE_T                iqCtrl;  /** controller state for the Q axis */
     
    /** Output limit for each axis of the current loops, normalized to DC link voltage,
     *  line-to-neutral, so that 0.57735 = 1/sqrt(3) = full line-to-line voltage */
    MC_DQ_T                     idqCtrlOutLimit; 
    
    int16_t                     dynLimit;  /** dynamic current limit */
    int16_t                     iqCmdLimit;  /** maximum output current amplitude, q-axis */
    
//    MCAF_STANDARD_INPUT_SIGNALS_T   standardInputs;  /** standard input signals */
    
    RB_MOTOR_PARAMS_T           motorParams; /** Kv and Rs*/                       
//    MCAF_MOTOR_PARAMETERS_T       motorParameters; /** motor parameters */
//    MCAF_BACKEMF_CALCULATION_T    backEMF;         /** quantities for estimated back-emf calculation */

    /* Current loop forward path */
    MC_DQ_T                     vdqCmd;     /** desired dq-frame voltage, output of current loop */
    MC_DQ_T                     vdq;        /** desired dq-frame voltage */
    MC_ALPHABETA_T              valphabeta; /** desired alphabeta-frame voltage */
    MC_ALPHABETA_T              valphabetaPerturbed; /** desired alphabeta-frame voltage, after perturbation */
    MC_ABC_T                    vabc;       /** desired phase voltage */
    int16_t                     rVdc;       /** reciprocal of DC link voltage */
    MC_ABC_T                    dabcRaw;    /** scaled duty cycle, per-unit, before dead-time compensation */
    //MCAF_DEAD_TIME_COMPENSATION deadTimeCompensation; /** dead-time compensation state */
    MC_ABC_T                    dabcUnshifted;  /** scaled duty cycle, per-unit, prior to ZSM and clipping */
    MC_ABC_T                    dabc;       /** after ZSM + clip */
    //MCAF_U_DUTYCYCLE_ALPHABETA dalphabetaOut[3]; /** convert dabc back to alpha-beta frame for estimators, with history */
    //MCAF_U_VOLTAGE_ALPHABETA valphabetaOut; /** value of applied alpha-beta voltage, including deadtime compensation */
    MC_ABC_T                    pwmDutycycle;   /** PWM count */
    //MCAF_FLUX_CONTROL_STATE_T fluxControl; /** flux-control state */
    MCAF_FILTER_LOW_PASS_S16_T  vqFiltered; /** filtered q-axis voltage, for feedback purposes */
    //MCAF_CURRENT_MEASUREMENT currentMeasure; /** current measure state */

    /* Angle and speed, including estimators */
    int16_t                     thetaElectrical;  /** electrical angle */
    int16_t                     omegaElectrical;  /** electrical frequency */
    MC_SINCOS_T                 sincos;     /** sine and cosine of electrical angle */

    RB_HALL_ESTIMATOR_T         estimator;  /** position and velocity estimator state */

    /* Velocity loop */
//    MCAF_U_VELOCITY_ELEC        omegaCmd;   /** input command for the velocity loop */
//    MCAF_PISTATE_T      omegaCtrl;  /** controller state for the velocity loop */
//    MCAF_VELOCITY_CONTROL_DATA velocityControl; /** Control inputs for the velocity loop */
//    MCAF_U_CURRENT    iqTorqueCmd;  /** output of the velocity loop */
    uint16_t                controlFlags; /** MCAF_CTRL_FLAGS bitfields */
    uint16_t                stateFlags;   /** MCAF_STATE_FLAGS bitfields */    
    HAL_ADC_SELECT_T        adcSelect;    /** which channel we are scanning */
    int16_t                 potInput; /** potentiometer input */
    
    MCAF_BRIDGE_TEMPERATURE bridgeTemperature;  /** bridge temperature */
    
    /* open-loop to closed-loop transition */
    //MCAF_MOTOR_STARTUP_DATA startup;  /** State variables for the startup code */
    
    MCAF_FSM_STATE      state;           /** motor control state machine state */
    
    /** counter for subsampling (e.g. executing something every N counts */
    uint16_t subsampleCounter;            
    
    /** user interface data exchanged via main thread and ISR */
//    volatile MCAF_UI_DATA ui;
    
    MCAF_FAULT_DETECT_T faultDetect;     /** fault detect state */
    MCAF_RECOVERY_DATA_T recovery; /** recovery status */
    MCAF_MONITOR_DATA_T monitor;   /** monitor data */

    /** test harness state, used by ISR, may be shared with main thread in future */
//    volatile MCAF_MOTOR_TEST_MANAGER testing;
    
    MCAF_SYSTEM_DATA *psys;       /** pointer to shared system state */
    MCAF_SAT_DETECT_T sat;        /** saturation detection */
//    MCAF_STOPPING_STATE stopping; /** Stopping timer state */

    /** current calibration parameters */
    RB_MEASURE_CURRENT_T currentCal;
        
    /** initialization - current offsets */
    MCAF_MOTOR_INITIALIZATION initialization;  

//    /** miscellaneous configurable parameters */
//    struct tagConfig {
//        /** number of ISR cycles to delay forward-path voltages
//         *  to match the delay of the current feedback signals
//         */
//        uint16_t deadTimeCompensationVoltageDelay;  
//    } config;
    
    /** MCAPI related shared data */
//    volatile MCAPI_MOTOR_DATA apiData;
    /** MCAPI related feedback data in MCAF that is 
     * published to the application through MCAPI */
    //MCAPI_FEEDBACK_SIGNALS apiFeedback;
    
    /** measured DC link voltage */
    int16_t vDC;
    
    /** measured DC link current*/
    int16_t iDC;
    
    /** measured absolute voltage reference */
    uint16_t vAbsRef;

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
    pPMSM->idqCmdRaw.d = 0;
    pPMSM->idqCmdRaw.q = 0;
}

#ifdef	__cplusplus
}
#endif

#endif	/* MC_CONTROL_H */