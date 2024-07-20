#include "rb_logging.h"
#include "motorBench/math_asm.h"
#include "math.h"

void RB_Logging_SumStepISR(RB_LOGGING_SUMS* sums, 
        int16_t vDC, int16_t iDC, int16_t iqRef, int16_t iqFdb, int16_t power, 
        uint16_t speed, uint16_t bridgeTemp, int16_t iA, int16_t iB,
        int16_t vA, int16_t vB) 
{
    // summed values
    sums->vDC += vDC;
    sums->iDC += iDC; 
    sums->iqRef += -iqRef; // applying switch in direction from controls
    sums->iqFdb += -iqFdb; 
    sums->power += power; 
    sums->speed += speed; 
    sums->bridgeTemp += bridgeTemp; 
  
    // summed squared values
    // mulss returns 32-bit int
    sums->iA += (int64_t)__builtin_mulss(iA, iA); 
    sums->iAB += (int64_t)__builtin_mulss(iA-iB, iA-iB); 
    sums->vA += (int64_t)__builtin_mulss(vA, vA); 
    sums->vAB += (int64_t)__builtin_mulss(vA-vB, vA-vB);
}

void RB_Logging_Average(RB_LOGGING_AVERAGES* averages, const RB_LOGGING_SUMS* sums){
    // regular means of summed values
    averages->vDC       = (int16_t) (sums->vDC      >> RB_LOGGING_DIVISOR);
    averages->iDC       = (int16_t) (sums->iDC      >> RB_LOGGING_DIVISOR); 
    averages->iqRef     = (int16_t) (sums->iqRef    >> RB_LOGGING_DIVISOR); 
    averages->iqFdb     = (int16_t) (sums->iqFdb    >> RB_LOGGING_DIVISOR); 
    averages->power     = (int16_t) (sums->power    >> RB_LOGGING_DIVISOR); 
    averages->speed     = (uint16_t)(sums->speed    >> RB_LOGGING_DIVISOR); 
    averages->bridgeTemp  = (uint16_t)(sums->bridgeTemp >> RB_LOGGING_DIVISOR); 
    
    /* root of mean of squared values. 
     * right shift brings back to 32-bit values from squaring 
     * trying square-root with floats now
     */
    averages->iA        = (int16_t) sqrtf((float)(sums->iA >> RB_LOGGING_DIVISOR));
    averages->iAB        = (int16_t) sqrtf((float)(sums->iAB >> RB_LOGGING_DIVISOR)); 
    averages->vA        = (int16_t) sqrtf((float)(sums->vA >> RB_LOGGING_DIVISOR)); 
    averages->vAB        = (int16_t) sqrtf((float)(sums->vAB >> RB_LOGGING_DIVISOR));
}

void RB_Logging_SumReset(RB_LOGGING_SUMS* sums){
    sums->vDC = 0;
    sums->iDC = 0; 
    sums->iqRef = 0; 
    sums->iqFdb = 0; 
    sums->power = 0; 
    sums->speed = 0; 
    sums->bridgeTemp = 0; 
    sums->iA = 0; 
    sums->iAB = 0; 
    sums->vA = 0; 
    sums->vAB = 0; 
}
