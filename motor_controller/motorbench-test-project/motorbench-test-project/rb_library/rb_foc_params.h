/* 
 * File:   rb_foc_params.h
 * Author: siani
 *
 * Created on June 5, 2024, 4:52 PM
 */

#ifndef RB_FOC_PARAMS_H
#define	RB_FOC_PARAMS_H

#ifdef	__cplusplus
extern "C" {
#endif


// Motor Parameters from vendor
#define     RB_MOTOR_RS               0.165   // phase resistance in ohms
#define     RB_MOTOR_KE               8.2 // motor constant in units RPM/V, assuming Line-Line RMS voltage
#define     RB_MOTOR_POLEPAIRS        26 // 52 poles total
#define     RB_MOTOR_HYS_LOSS         0.715 // Hysteresis Losses (N-m)
#define     RB_MOTOR_EDDIE_LOSS       0.009 // Eddie Losses (N-m / rad/sec)

// D-axis current control loop coefficients in Q15 format
#define     RB_DCURRENT_KP              Q15(0.5) // Q15(0.05) or Q15(0.2543) 
#define     RB_DCURRENT_KI              Q15(0.006) //Q15(0.003) or Q15(0.0188)
#define     RB_DCURRENT_KC              Q15(0.999) // Q15(0.999)
#define     RB_DVOLTAGE_OUTMAX          0x7FFF      //32767
#define     RB_DVOLTAGE_OUTMIN          0    
    
// Q-axis current control loop coefficients in Q15 format
#define     RB_QCURRENT_KP          Q15(0.5) //Q15(0.05)
#define     RB_QCURRENT_KI          Q15(0.006) // Q15(0.003)
#define     RB_QCURRENT_KC          Q15(0.999)
#define     RB_QVOLTAGE_OUTMAX      0x7FFF      //32767
#define     RB_QVOLTAGE_OUTMIN      0  

// Q-axis current ramp rate limiting
#define     RB_QCURRENT_MAX          2000
#define     RB_QRAMP_INCREMENT       Q15(0.0003) // ammount iq is incremented per ISR step 0.00003
#define     RB_QRAMP_COUNT           3 // new iq ref is calculated every [REF_RAMP_COUNT] times            
    
// Safety Fault related
#define     RB_PHASECURRENT_MAX     21000  // 21228 is 14.14peak = 10A RMS from user guide
    

#ifdef	__cplusplus
}
#endif

#endif	/* RB_FOC_PARAMS_H */

