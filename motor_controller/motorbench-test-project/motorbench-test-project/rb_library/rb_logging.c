#include "rb_logging.h"

RB_Logging_SumStep(RB_LOGGING_SUMS* sums) {
  sums->vDC;
  sums->iDC; 
  sums->iA; 
  sums->iB; 
  sums->vA; 
  sums->vB; 
  sums->speed; 
  sums->iqRef; 
  sums->iqFdb; 
  sums->temp_fet; 
  sums->power; 
}

RB_Logging_Averaging(RB_LOGGING_AVGS* averages, const RB_LOGGING_SUMS* sums){
    averages->vDC       = (int16_t) (sums->vDC      >> RB_LOGGING_DIVISOR);
    averages->iDC       = (int16_t) (sums->iDC      >> RB_LOGGING_DIVISOR); 
    averages->iA        = (int16_t) (sums->iA       >> RB_LOGGING_DIVISOR); 
    averages->iB        = (int16_t) (sums->iB       >> RB_LOGGING_DIVISOR); 
    averages->vA        = (int16_t) (sums->vA       >> RB_LOGGING_DIVISOR); 
    averages->vB        = (int16_t) (sums->vB       >> RB_LOGGING_DIVISOR); 
    averages->speed     = (uint16_t)(sums->speed    >> RB_LOGGING_DIVISOR); 
    averages->iqRef     = (int16_t) (sums->iqRef    >> RB_LOGGING_DIVISOR); 
    averages->iqFdb     = (int16_t) (sums->iqFdb    >> RB_LOGGING_DIVISOR); 
    averages->temp_fet  = (uint16_t)(sums->temp_fet >> RB_LOGGING_DIVISOR); 
    averages->power     = (int16_t) (sums->power    >> RB_LOGGING_DIVISOR); 
}

RB_Logging_SumReset(RB_LOGGING_SUMS* sums){
  sums->vDC = 0;
  sums->iDC = 0; 
  sums->iA = 0; 
  sums->iB = 0; 
  sums->vA = 0; 
  sums->vB = 0; 
  sums->speed = 0; 
  sums->iqRef = 0; 
  sums->iqFdb = 0; 
  sums->temp_fet = 0; 
  sums->power = 0; 
}
