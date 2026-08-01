#include "EncFaults.h"

void InitializeEncFaults(EncFaults *e){
  e->directionErrorThreshCount = 5;
  e->directionErrorRPMThresh_firstOn = 30;
  e->directionErrorRPMThresh_belowOff = 10;
  e->encoderCheckingOn=0; 
}

void ResetEncFaults(EncFaults *e){
    e->directionErrorCount=0;
    e->directionErrFlag=0;
    e->transitionErrFlag=0;
    e->indexErrFlag=0;
    e->EncoderErrFlag = 0;
}


void EnableEncoderFltChking(EncFaults *e){
  e->encoderCheckingOn = 0;
  e->encoderChkBool= 1;
}
void startEncoderChecking(EncFaults *e,uint16_t absRPM){
  if ( e->encoderChkBool){
    if (e->encoderCheckingOn==0){
      if (absRPM > e->directionErrorRPMThresh_firstOn){
         e->encoderCheckingOn=1;
         ResetEncFaults(e);
      }
    }
  }
}

void stopEncoderChecking(EncFaults *e,uint16_t absRPM){
    if(e->encoderCheckingOn==1){
      if (absRPM < e->directionErrorRPMThresh_belowOff){
         e->encoderCheckingOn=0;
         e->encoderChkBool = 0;
      }
    }
}
  