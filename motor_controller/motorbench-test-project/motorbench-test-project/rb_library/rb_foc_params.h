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
#define     RB_RS               0
#define     RB_KE               0

// D-axis current control loop coefficients in Q15 format
#define     RB_DCURRENT_KP              Q15(0.05) // initial values from ref project 
#define     RB_DCURRENT_KI              Q15(0.003)
#define     RB_DCURRENT_KC              Q15(0.999)
#define     RB_DVOLTAGE_OUTMAX          0x7FFF      //32767
#define     RB_DVOLTAGE_OUTMIN          0    
    
// D-axis current control loop coefficients in Q15 format
#define     RB_QCURRENT_KP          Q15(0.05) // initial values from ref project 
#define     RB_QCURRENT_KI          Q15(0.003)
#define     RB_QCURRENT_KC          Q15(0.999)
#define     RB_QVOLTAGE_OUTMAX      0x7FFF      //32767
#define     RB_QVOLTAGE_OUTMIN      0  
    
// speed control loop coefficients in Q15 format - Do later
    

    

#ifdef	__cplusplus
}
#endif

#endif	/* RB_FOC_PARAMS_H */

