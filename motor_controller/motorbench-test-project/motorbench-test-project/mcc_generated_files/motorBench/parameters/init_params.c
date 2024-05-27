/**
 * init_params.c
 * 
 * Initialization of motor parameters
 *
 * Component: FOC
 */ /*
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
 *
 ******************************************************************************/

#include "../system_state.h"
#include "foc_params.h"
#include "adc_params.h"
#include "outerloop_params.h"
#include "filter.h"

void MCAF_InitControlParameters_Motor1(MCAF_MOTOR_DATA *pmotor)
{
    /* ============= PI D Term =============== */
    pmotor->idCtrl.kp = DKP;
    pmotor->idCtrl.ki = DKI;
    pmotor->idCtrl.nkp = DKNP;
    pmotor->idCtrl.nki = DKNI;
    pmotor->idCtrl.kc = DKC;
    pmotor->idCtrl.outMax = 0;
    pmotor->idCtrl.outMin = 0;
    pmotor->idqCtrlOutLimit.d = MCAF_CURRENT_CTRL_D_OUT_LIMIT;

    /* ============= PI Q Term =============== */
    pmotor->iqCtrl.kp = QKP;
    pmotor->iqCtrl.ki = QKI;
    pmotor->iqCtrl.nkp = QKNP;
    pmotor->iqCtrl.nki = QKNI;
    pmotor->iqCtrl.kc = QKC;
    pmotor->iqCtrl.outMax = 0;
    pmotor->iqCtrl.outMin = 0;
    pmotor->idqCtrlOutLimit.q = MCAF_CURRENT_CTRL_Q_OUT_LIMIT;

    /* ============= PI W Term =============== */
    if (MCAF_OuterLoopType() == MCAF_OLT_VOLTAGE)
    {
        pmotor->omegaCtrl.kp = MCAF_CONTROL_GAIN_KVP;
        pmotor->omegaCtrl.ki = MCAF_CONTROL_GAIN_KVI;
        pmotor->omegaCtrl.nkp = 15 - MCAF_CONTROL_GAIN_KVP_Q;
        pmotor->omegaCtrl.nki = 15 - MCAF_CONTROL_GAIN_KVI_Q;
        pmotor->velocityControl.velocityCmdGain = MCAF_VELOCITY_TO_VOLTAGE_GAIN;
    }
    else
    {
        pmotor->omegaCtrl.kp = WKP;
        pmotor->omegaCtrl.ki = WKI;
        pmotor->omegaCtrl.nkp = WKNP;
        pmotor->omegaCtrl.nki = WKNI;
        pmotor->velocityControl.velocityCmdGain = INT16_MAX;
    }
    pmotor->omegaCtrl.kc = WKC;
    pmotor->omegaCtrl.outMax = MCAF_VELOCITY_CTRL_IQ_OUT_LIMIT;
    pmotor->omegaCtrl.outMin = -MCAF_VELOCITY_CTRL_IQ_OUT_LIMIT;
    
    pmotor->bridgeTemperature.gain   = MCAF_BRIDGE_TEMPERATURE_GAIN;
    pmotor->bridgeTemperature.offset = MCAF_BRIDGE_TEMPERATURE_OFFSET;
    pmotor->bridgeTemperature.filter.gain = MCAF_BRIDGE_TEMPERATURE_FILTER_GAIN;
    pmotor->bridgeTemperature.filter.state.x32 = 0;
    pmotor->bridgeTemperature.filter.output = 0;
    pmotor->bridgeTemperature.filter.slewRate = MCAF_BRIDGE_TEMPERATURE_SLEW_RATE;

    MCAF_FilterLowPassS16Init(&pmotor->vqFiltered, MCAF_FILTER_COEFF_VQ);
}

