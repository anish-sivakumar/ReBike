/**
 * stall_detect.c
 *
 * Module to detect when motor is stall
 *
 * Component: supervisory
 */

/* *********************************************************************
 *
 * Motor Control Application Framework
 * R7/RC37 (commit 116330, build on 2023 Feb 09)
 *
 * (c) 2017 - 2023 Microchip Technology Inc. and its subsidiaries. You may use
 * this software and any derivatives exclusively with Microchip products.
 *
 * This software and any accompanying information is for suggestion only.
 * It does not modify Microchip's standard warranty for its products.
 * You agree that you are solely responsible for testing the software and
 * determining its suitability.  Microchip has no obligation to modify,
 * test, certify, or support the software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH
 * MICROCHIP PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY
 * APPLICATION.
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL,
 * PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF
 * ANY KIND WHATSOEVER RELATED TO THE USE OF THIS SOFTWARE, THE
 * motorBench(R) DEVELOPMENT SUITE TOOL, PARAMETERS AND GENERATED CODE,
 * HOWEVER CAUSED, BY END USERS, WHETHER MICROCHIP'S CUSTOMERS OR
 * CUSTOMER'S CUSTOMERS, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES OR THE DAMAGES ARE FORESEEABLE. TO THE
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
 * CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
 * OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
 * SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF
 * THESE TERMS.
 *
 * *****************************************************************************/

#include "stall_detect.h"
#include "parameters/fault_detect_params.h"
#include "parameters/options.h"
#include "parameters/operating_params.h"
#include "util.h"
#include "filter_types.h"
#include "filter.h"

/**
 * Initializes parameters related to variance detector
 *
 * @param pvariancex16 variance detect state
 */
void MCAF_VarianceDetectx16Init(MCAF_VARIANCE_DETECT_T *pvariancex16);

/**
 * Computes variance of Ed and Eq signal and indicate stall
 * when high frequency content of Ed and Eq exceeds DC content
 *
 * Summary : 1st of 2 stage implementation of variance detect
 *
 * @param pvariancex16 variance detect state
 * @param input signals including Ed and Eq
 */
void MCAF_VarianceDetectx16(MCAF_VARIANCE_DETECT_T *pvariancex16,
        const MCAF_STALL_DETECT_INPUT_T *pinputs);

/**
 * Computes variance of Ed and Eq signal and indicate stall
 * when high frequency content of Ed and Eq exceeds DC content
 *
 * Summary : 2nd of 2 stage implementation of variance detect
 *
 * @param pvariancex16 variance detect state
 * @return whether high frequency content in Es is higher than DC content (true = detected)
*/
bool MCAF_VarianceDetectx16PostDecimation(MCAF_VARIANCE_DETECT_T *pvariancex16);

/**
 * Initializes parameters related to torque angle based stall detection
 *
 * @param ptorqueangle torque angle detect state
 */
void MCAF_TorqueAngleDetectInit(MCAF_TORQUE_ANGLE_DETECT_T *ptorqueAngle);

/**
 * Computes torque angle of motor and indicates stall
 * when torque angle falls below computed threshold based on speed
 *
 * Summary : Torque angle based stall detect
 *
 * @param ptorqueangle torque angle detect state
 * @param pinputs input signals including Ed and Eq
 * @return whether (Vs^2)*(stall_const)^2 is greater than (Es^2) (true = detected)
*/
bool MCAF_TorqueAngleDetect(MCAF_TORQUE_ANGLE_DETECT_T *ptorqueAngle,
        const MCAF_STALL_DETECT_INPUT_T *pinputs);

/**
 * Initializes parameters related to overcurrent detect
 *
 * @param povercurrent overcurrent detect state
 */
void MCAF_OvercurrentDetectInit(MCAF_OVERCURRENT_SW_DETECT_T *povercurrent);

/**
 * Computes Is (Id^2 + Iq^2) and indicates stall when Is exceeds threshold
 *
 * Summary : Over current stall detect
 *
 * @param povercurrent overcurrent detect state
 * @param pidq current vector Idq
 * @return whether (Is^2) is greater than threshold (true = detected)
 */
bool MCAF_OvercurrentDetect(MCAF_OVERCURRENT_SW_DETECT_T *povercurrent, const MC_DQ_T *pidq);

/**
 * Initializes parameters related to low speed detect
 *
 * @param plowspeed low speed detect state
 */
void MCAF_LowSpeedDetectInit(MCAF_LOW_SPEED_DETECT_T *plowSpeed);

/**
 * Monitors estimated velocity and indicates stall when velocity falls below threshold
 *
 * @param plowspeed low speed detect state
 * @param omega estimated velocity by estimator
 * @return whether omega_electrical is below threshold (true = detected)
 */
bool MCAF_LowSpeedDetect(MCAF_LOW_SPEED_DETECT_T *plowSpeed, int16_t omega);

/**
 * Initializes parameters related to negative Ed based stall detect
 *
 * @param Negative Ed stall detect state
 */
void MCAF_NegativeEdDetectInit(MCAF_NEGATIVE_ED_DETECT_T *pEdState);

/**
 * Monitors Ed and saturation state. It will indicates stall
 * when Ed goes negative and there is current saturation
 *
 * Summary : Negative Ed stall detect
 *
 * @param pedstate Negative Ed stall detect state
 * @param pinputs input signals including estimated d-axis back-EMF
 * @param currsat true = sat current, false = no saturation on current
 * @return whether Ed is below threshold during current saturation (true = detected)
 */
bool MCAF_NegativeEdDetect(MCAF_NEGATIVE_ED_DETECT_T *pedState,
        const MCAF_STALL_DETECT_INPUT_T *pinputs,
        bool currsat);

/**
 * Initializes various parameters associated with stall detect
 *
 * @param pstalldetect stall detect state
 */

void MCAF_StallDetectInit(MCAF_STALL_DETECT_T *pstallDetect)
{

    MCAF_VarianceDetectx16Init(&pstallDetect->varianceDetect);
    MCAF_OvercurrentDetectInit(&pstallDetect->overcurrentDetect);
    MCAF_LowSpeedDetectInit(&pstallDetect->lowSpeedDetect);
    MCAF_NegativeEdDetectInit(&pstallDetect->negativeEdDetect);
    MCAF_TorqueAngleDetectInit(&pstallDetect->torqueAngleDetect);
    pstallDetect->stallDetectFlag = 0;
    pstallDetect->stallDetectFlagMask = ~MCAF_TORQUE_ANGLE_STALL_DETECT;    //Do not detect torque angle stall
    pstallDetect->decimationTimer = 0;
    pstallDetect->decimationTimerThreshold = DECIMATION_FACTOR;
}


void MCAF_VarianceDetectx16Reset(MCAF_VARIANCE_DETECT_T *pvariancex16)
{
        /* Initialize state variables for 16 bit implementation of LPF and HPF */
    MCAF_LpfFilterInitx16(&pvariancex16->lpf1EsSqr);
    MCAF_LpfFilterInitx16(&pvariancex16->lpf2EsSqr);

    MCAF_LpfFilterInitx16(&pvariancex16->lpf1HpEsSqr);
    MCAF_LpfFilterInitx16(&pvariancex16->lpf2HpEsSqr);

    MCAF_HpfFilterInitx16(&pvariancex16->hpfEd);
    MCAF_HpfFilterInitx16(&pvariancex16->hpfEq);
    
    /* Timer counts for variance detect */
    pvariancex16->timer = 0;
}

void MCAF_VarianceDetectx16Init(MCAF_VARIANCE_DETECT_T *pvariancex16)
{
    /* Initialize a1 filter constant for 16 bit implementation of LPF */
    pvariancex16->lpf1EsSqr.coeff = DECIMATION_FILTER_LPF1_VARIANCE_DETECT_COEFFA1;
    pvariancex16->lpf2EsSqr.coeff = DECIMATION_FILTER_LPF2_VARIANCE_DETECT_COEFFA1;

    pvariancex16->lpf1HpEsSqr.coeff = DECIMATION_FILTER_LPF1_VARIANCE_DETECT_COEFFA1;
    pvariancex16->lpf2HpEsSqr.coeff = DECIMATION_FILTER_LPF2_VARIANCE_DETECT_COEFFA1;

    /* Initialize filter constant for 16 bit implementation of HPF */
    pvariancex16->hpfEd.coeff = FILTER_HPF_VARIANCE_DETECT;
    pvariancex16->hpfEq.coeff = FILTER_HPF_VARIANCE_DETECT;

    /* Initialize timer for variance detect */
    pvariancex16->timerThreshold = TIMER_COUNTS_VARIANCE_DETECT;
    /* Timer counts for variance detect */
    MCAF_VarianceDetectx16Reset(pvariancex16);
}

void MCAF_OvercurrentDetectReset(MCAF_OVERCURRENT_SW_DETECT_T *povercurrent)
{
    /* Initialize state variables for LPF */
    MCAF_LpfFilterInitx16(&povercurrent->lpfIs);

    povercurrent->timer = 0;
}


void MCAF_OvercurrentDetectInit(MCAF_OVERCURRENT_SW_DETECT_T *povercurrent)
{
    /* Initialize a1 filter constant for 16 bit implementation of LPF */
    povercurrent->lpfIs.coeff = FILTER_LPF_OVERCURRENT_DETECT;

    povercurrent->overcurrentThreshold = THRESHOLD_OVERCURRENT_STALL_DETECT;
    povercurrent->timerThreshold = TIMER_COUNTS_OVERCURRENT_DETECT;

    MCAF_OvercurrentDetectReset(povercurrent);
}

void MCAF_NegativeEdDetectReset(MCAF_NEGATIVE_ED_DETECT_T *pEdState)
{
    pEdState->timerActive = 0;
    pEdState->timerInactive = 0;
}


void MCAF_NegativeEdDetectInit(MCAF_NEGATIVE_ED_DETECT_T *pEdState)
{
    pEdState->timeActiveThreshold = ACTIVE_TIMER_THRESHOLD_NEGATIVE_ED_DETECT;

    pEdState->timerInactiveThreshold = INACTIVE_TIMER_THRESHOLD_NEGATIVE_ED_DETECT;

    pEdState->edThreshold = THRESHOLD_ED_STALL_DETECT;
    
    MCAF_NegativeEdDetectReset(pEdState);
}

void MCAF_TorqueAngleDetectReset(MCAF_TORQUE_ANGLE_DETECT_T *pTorqueAngle)
{
    pTorqueAngle->esSqr = 0;
    pTorqueAngle->vsSqr = 0;
    pTorqueAngle->timerActive = 0;
    pTorqueAngle->timerInActive = 0;
}

void MCAF_TorqueAngleDetectInit(MCAF_TORQUE_ANGLE_DETECT_T *pTorqueAngle)
{
    pTorqueAngle->k = STALL_DETECT_TORQUE_ANGLE_K;
    pTorqueAngle->polycoef[0] = STALL_DETECT_TORQUE_ANGLE_COEFF0;
    pTorqueAngle->polycoef[1] = STALL_DETECT_TORQUE_ANGLE_COEFF1;
    pTorqueAngle->polycoef[2] = STALL_DETECT_TORQUE_ANGLE_COEFF2;
    pTorqueAngle->velocityThreshold = STALL_DETECT_TORQUE_ANGLE_VELOCITY_THRESHOLD;
    
    pTorqueAngle->timerActiveThreshold = ACTIVE_TIMER_THRESHOLD_TORQUE_ANGLE_DETECT;

    pTorqueAngle->timerInActiveThreshold = INACTIVE_TIMER_THRESHOLD_TORQUE_ANGLE_DETECT;
    
    MCAF_TorqueAngleDetectReset(pTorqueAngle);

}

void MCAF_LowSpeedDetectReset(MCAF_LOW_SPEED_DETECT_T *plowspeed)
{
    plowspeed->timerActive = 0;
    plowspeed->timerInActive = 0;
}

void MCAF_LowSpeedDetectInit(MCAF_LOW_SPEED_DETECT_T *plowspeed)
{
    /* Speed threshold for low speed */
    plowspeed->lowSpeedThreshold = THRESHOLD_UNDERSPEED_STALL_DETECT;
    /* Timer counts for low speed detect */
    plowspeed->timerActiveThreshold = ACTIVE_TIMER_THRESHOLD_LOW_SPEED_DETECT;

    /* Timer counts for low speed detect */
    plowspeed->timerInActiveThreshold = INACTIVE_TIMER_THRESHOLD_LOW_SPEED_DETECT;
    
    MCAF_LowSpeedDetectReset(plowspeed);

}

void MCAF_StallDetectReset(MCAF_STALL_DETECT_T *pstallDetect)
{
    MCAF_VarianceDetectx16Reset(&pstallDetect->varianceDetect);
    MCAF_OvercurrentDetectReset(&pstallDetect->overcurrentDetect);
    MCAF_NegativeEdDetectReset(&pstallDetect->negativeEdDetect);
    MCAF_TorqueAngleDetectReset(&pstallDetect->torqueAngleDetect);
    MCAF_LowSpeedDetectReset(&pstallDetect->lowSpeedDetect);
    pstallDetect->stallDetectFlag = 0;    
}

/**
 * Internal function for updating stall detect condition bit flags.
 * Updates mask bits of the stallDetectFlag field, based on a condition.
 * 
 * @param pstallDetect stall detect state
 * @param condition true to set the mask bits, false to clear the mask bits
 * @param mask bits that should be set or cleared
 */
inline static void updateFlags(MCAF_STALL_DETECT_T *pstallDetect, bool condition, uint16_t mask)
{
    if (condition)
    {
        pstallDetect->stallDetectFlag = UTIL_SetBits(pstallDetect->stallDetectFlag, mask);
    }
    else
    {
        pstallDetect->stallDetectFlag = UTIL_ClearBits(pstallDetect->stallDetectFlag, mask);
    }    
}

void MCAF_StallDetect(MCAF_STALL_DETECT_T *pstallDetect, const MCAF_MOTOR_DATA *pmotor)
{
    if (MCAF_OvercurrentDetectEnabled())
    {
        updateFlags(pstallDetect, 
                  MCAF_OvercurrentDetect(&pstallDetect->overcurrentDetect, &pmotor->idq),
                  MCAF_OVERCURRENT_STALL_DETECT);
    }
    
    if (MCAF_StartupGetStatus(&pmotor->startup) == MSST_COMPLETE)
    {
    
        if (MCAF_NegativeEdDetectEnabled())
        {
            updateFlags(pstallDetect,
                      MCAF_NegativeEdDetect(&pstallDetect->negativeEdDetect,
                                            &pstallDetect->inputs,
                                            pmotor->sat.state == MCAF_SAT_CURRENT),
                      MCAF_NEGATIVE_ED_STALL_DETECT);
        }
        
        if (MCAF_VarianceDetectEnabled())
        {
            MCAF_VarianceDetectx16(&pstallDetect->varianceDetect, 
                                   &pstallDetect->inputs);
        }
        
        if (pmotor->state == MCSM_RUNNING)
        {
            if (MCAF_LowSpeedDetectEnabled())
            {
                updateFlags(pstallDetect,
                            MCAF_LowSpeedDetect(&pstallDetect->lowSpeedDetect, pmotor->omegaElectrical),
                            MCAF_LOW_SPEED_STALL_DETECT);
            }
            if (MCAF_TorqueAngleDetectEnabled())
            {
                updateFlags(pstallDetect,
                            MCAF_TorqueAngleDetect(&pstallDetect->torqueAngleDetect, 
                                                   &pstallDetect->inputs),
                            MCAF_TORQUE_ANGLE_STALL_DETECT);
            }
        }
        if (MCAF_VarianceDetectEnabled() &&
                ++pstallDetect->decimationTimer >= pstallDetect->decimationTimerThreshold)
        {
            pstallDetect->decimationTimer = 0;
            updateFlags(pstallDetect,
                        MCAF_VarianceDetectx16PostDecimation(&pstallDetect->varianceDetect),
                        MCAF_LOSS_OF_LOCK_STALL_DETECT);
        }
    }
}

inline void MCAF_VarianceDetectx16(MCAF_VARIANCE_DETECT_T *pvariancex16,
        const MCAF_STALL_DETECT_INPUT_T *pinputs)
{
    /* HPF for Esd */
    const int16_t esdHpf = MCAF_HpfFilterx16(&pvariancex16->hpfEd, pinputs->esdq.d);
    /* HPF for Esq */
    const int16_t esqHpf = MCAF_HpfFilterx16(&pvariancex16->hpfEq, pinputs->esdq.q);
    /* Esd(HPF)^2 + Esq(HPF)^2 */
    const int16_t esHpf = UTIL_SignedSqr(esdHpf) + UTIL_SignedSqr(esqHpf);
    /* 1st stage for LPF for Es(HPF) */
    MCAF_LpfFilterx16(&pvariancex16->lpf1HpEsSqr, esHpf);
    /* Esd^2 + Esq^2 */
    const int16_t esSqr = UTIL_SignedSqr(pinputs->esdq.d) + UTIL_SignedSqr(pinputs->esdq.q);
    /* 1st stage LPF filter for Es */
    MCAF_LpfFilterx16(&pvariancex16->lpf1EsSqr, esSqr);
}

inline bool MCAF_VarianceDetectx16PostDecimation(MCAF_VARIANCE_DETECT_T *pvariancex16)
{
   bool varianceDetect = false;
   /* 2nd stage filter for Es(HPF) */
   MCAF_LpfFilterx16(&pvariancex16->lpf2HpEsSqr, pvariancex16->lpf1HpEsSqr.output);
   /* 2nd stage filter for Es */
   MCAF_LpfFilterx16(&pvariancex16->lpf2EsSqr, pvariancex16->lpf1EsSqr.output);
   /* esLPF2 divide by 16 */
   pvariancex16->esLp = (pvariancex16->lpf2EsSqr.output >> 4);

   /* Set the flag and start the timer when Es(HPF-LPF) > Es(LPF) */
    if (pvariancex16->lpf2HpEsSqr.output > pvariancex16->esLp)
    {
        pvariancex16->timer++;
        if (pvariancex16->timer >= pvariancex16->timerThreshold)
        {
            /* Detects loss of lock when the timer exceeds the time counts */
            varianceDetect = true;
            pvariancex16->timer = 0;
        }
    }
    else
    {
        pvariancex16->timer = 0;
    }
    return varianceDetect;
}

/**
 * Evaluate a quadratic polynomial using Horner's rule and Q15 math.
 * @param x input variable
 * @param pcoef polynomial coefficients in order of increasing degree
 */
inline static int16_t MCAF_TorqueAngleEvaluateQuadratic(
    int16_t x,
    const int16_t *pcoef)
{
    int16_t y;
    y = UTIL_MulQ15(pcoef[2], x);
    y += *++pcoef;
    y = UTIL_MulQ15(y, x);
    y += *--pcoef;
    return y;
}

inline bool MCAF_TorqueAngleDetect(MCAF_TORQUE_ANGLE_DETECT_T *ptorqueangle,
                                    const MCAF_STALL_DETECT_INPUT_T *pinputs)
{
    bool detectedNow = false;
    bool stall_detect = false;
    const int16_t absElectricalFrequency = UTIL_Abs16Approx(pinputs->omegaElectrical);
    
    /* Torque angle detection is needed only when velocity is below a threshold */
    if (absElectricalFrequency < ptorqueangle->velocityThreshold)
    {

        /* if rotor is stop,most of Vs voltage should be added on R and L */
        /* and the Es may become very small,and if Es < Vs*Gain,means rotor stall */
        /* Es^2 = Ed^2 + Eq^2 */
        ptorqueangle->esSqr = UTIL_SignedSqr(pinputs->esdqFiltered.d) +
                              UTIL_SignedSqr(pinputs->esdqFiltered.q);
        /* Vs^2 = Valpha^2 + Vbeta^2 */
        ptorqueangle->vsSqr = UTIL_SignedSqr(pinputs->valphabeta.alpha) +
                              UTIL_SignedSqr(pinputs->valphabeta.beta);

        ptorqueangle->g = MCAF_TorqueAngleEvaluateQuadratic(
                            absElectricalFrequency,
                            ptorqueangle->polycoef);
        ptorqueangle->gEsSqr = UTIL_MulQ15(ptorqueangle->g, ptorqueangle->esSqr);
        ptorqueangle->kx = UTIL_MulQ15(ptorqueangle->k, absElectricalFrequency);
        ptorqueangle->kxVsSqr = UTIL_MulQ15(ptorqueangle->kx, ptorqueangle->vsSqr);
        
        detectedNow = ptorqueangle->gEsSqr < ptorqueangle->kxVsSqr;
    }
    
    if (detectedNow)
    {
        ptorqueangle->quotient = __builtin_divf(ptorqueangle->gEsSqr, ptorqueangle->kxVsSqr);
        ptorqueangle->timerInActive = 1;
        
        ptorqueangle->timerActive++;

        /* check if the timer active exceed the threshold */
        if (ptorqueangle->timerActive > ptorqueangle->timerActiveThreshold)
        {
            /* set the fault state */
            stall_detect = true;
            
            ptorqueangle->timerActive = 0;
            ptorqueangle->timerInActive = 0;
        }
    }
    else
    {
        ptorqueangle->quotient = 0x7fff;
        if (ptorqueangle->timerActive > 0)
        {
            ptorqueangle->timerActive++;
            
            ptorqueangle->timerInActive++;
            /* if timer inactive exceeded the threshold */
            if (ptorqueangle->timerInActive > ptorqueangle->timerInActiveThreshold)
            {
                /* reset both timers */
                ptorqueangle->timerActive = 0;
                ptorqueangle->timerInActive = 0;
            }
        }
    }
    return stall_detect;

}

inline bool MCAF_OvercurrentDetect(MCAF_OVERCURRENT_SW_DETECT_T *povercurrent, const MC_DQ_T *pidq)
{
    bool overCurrentDetect = false;
    povercurrent->is = UTIL_SignedSqr(pidq->d) + UTIL_SignedSqr(pidq->q);
    MCAF_LpfFilterx16(&povercurrent->lpfIs, povercurrent->is);

    if (povercurrent->lpfIs.output > UTIL_SignedSqr(povercurrent->overcurrentThreshold))
    {
        /* start timer for over current when Is exceeds the threshold */
        povercurrent->timer++;
        if (povercurrent->timer >= povercurrent->timerThreshold)
        {
            overCurrentDetect = true;    /* set the global flag when timer exceeds threshold */
            povercurrent->timer = 0;
        }
    }
    else
    {
        povercurrent->timer = 0;
    }
    return overCurrentDetect;
}


inline bool MCAF_NegativeEdDetect(MCAF_NEGATIVE_ED_DETECT_T *pedstate,
         const MCAF_STALL_DETECT_INPUT_T *pinputs,
         bool currsat)
{
    bool negativeEdDetect = false;
    
    if ((currsat == true) && (pinputs->esdq.d < pedstate->edThreshold))
    {
        /* re-initialize inactive timer */
        /* initialize it with 1 to indicate it was started */
        pedstate->timerInactive = 1;
        
        pedstate->timerActive++;

        /* check if the timer active exceed the threshold */
        if (pedstate->timerActive > pedstate->timeActiveThreshold)
        {
            /* set the fault state */
            negativeEdDetect = true;
            pedstate->timerActive = 0;
            pedstate->timerInactive = 0;
        }
    }
    else
    {
        if (pedstate->timerActive > 0)
        {
            pedstate->timerActive++;
            
            pedstate->timerInactive++;
            if (pedstate->timerInactive > pedstate->timerInactiveThreshold)
            {
                /* reset both timers */
                pedstate->timerActive = 0;
                pedstate->timerInactive = 0;
            }
        }
    }

    return negativeEdDetect;
 }

inline bool MCAF_LowSpeedDetect(MCAF_LOW_SPEED_DETECT_T *plowspeed, int16_t omega)
{
    bool lowSpeedDetect = false;
    /* initialize under speed as false, only a fault detect condition can change this state *
     * if the under speed threshold was crossed */
    if (UTIL_Abs16(omega) < plowspeed->lowSpeedThreshold)
    {
        /* re-initialize inactive timer */
        /* initialize it with 1 to indicate it was started */
        plowspeed->timerInActive = 1;
        
        plowspeed->timerActive++;

        /* check if the timer active exceed the threshold */
        if (plowspeed->timerActive > plowspeed->timerActiveThreshold)
        {
            /* set the fault state */
            lowSpeedDetect = true;
            plowspeed->timerActive = 0;
            plowspeed->timerInActive = 0;
        }
    }
    else
    {
        if (plowspeed->timerActive > 0)
        {
            plowspeed->timerActive++;
            
            plowspeed->timerInActive++;
            /* if timer inactive exceeded the threshold */
            if (plowspeed->timerInActive > plowspeed->timerInActiveThreshold)
            {
                /* reset both timers */
                plowspeed->timerActive = 0;
                plowspeed->timerInActive = 0;
            }
        }
    }

   return lowSpeedDetect;
}
