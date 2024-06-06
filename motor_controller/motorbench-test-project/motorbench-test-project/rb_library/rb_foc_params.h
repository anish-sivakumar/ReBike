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
#define     RB_DKP              0 // get starting values from github project
#define     RB_DKI              0
#define     RB_DKC              0
#define     RB_DOUTMAX          0
#define     RB_DOUTMIN          0    
    
// D-axis current control loop coefficients in Q15 format
#define     RB_QKP          0 // get starting values from github project
#define     RB_QKI          0
#define     RB_QKC          0 
#define     RB_QOUTMAX      0
#define     RB_QOUTMIN      0  
    
// speed control loop coefficients in Q15 format - Do later
    
// Bridge Temp parameters
    
/* temperature gain */
#define RB_BRIDGE_TEMPERATURE_GAIN        33000      // Q16(  0.50354) = +503.54004 m           = +503.54000 m           + 0.0000%
#define RB_BRIDGE_TEMPERATURE_OFFSET            5000 //            temperature offset
/* Pole of bridge temperature low-pass filter */
#define RB_BRIDGE_TEMPERATURE_FILTER_GAIN        328      // Q16(  0.00500) = +100.09766 rad/s       = +100.00000 rad/s       + 0.0977%
/* Maximum temperature slew rate */
#define RB_BRIDGE_TEMPERATURE_SLEW_RATE       1311      // Q15(  0.04001) =   +4.00085 C/s         =   +4.00000 C/s         + 0.0214%
/* Pole of voltage loop low-pass filter */
#define RB_FILTER_COEFF_VQ                 1638      // Q16(  0.02499) = +499.87793 rad/s       = +500.00000 rad/s       - 0.0244%
  
    

#ifdef	__cplusplus
}
#endif

#endif	/* RB_FOC_PARAMS_H */

