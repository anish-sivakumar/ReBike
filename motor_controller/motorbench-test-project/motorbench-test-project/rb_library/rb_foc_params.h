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
    

    

#ifdef	__cplusplus
}
#endif

#endif	/* RB_FOC_PARAMS_H */

