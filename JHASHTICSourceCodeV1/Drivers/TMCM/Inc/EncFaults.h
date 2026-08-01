
#ifndef ENCFAULTS_H_
#define ENCFAULTS_H_

#include "stdint.h"


typedef struct {
  uint8_t directionErrorCount;
  uint8_t directionErrorThreshCount;
  uint8_t directionErrFlag;
  uint8_t transitionErrFlag;
  uint8_t indexErrFlag;
  uint8_t EncoderErrFlag;
  uint8_t directionErrorRPMThresh_firstOn;
  uint8_t directionErrorRPMThresh_belowOff;
  uint8_t encoderCheckingOn;
  uint8_t encoderChkBool;
}EncFaults;

extern EncFaults encFlts;

void InitializeEncFaults(EncFaults *e);
void EnableEncoderFltChking(EncFaults *e);
void ResetEncFaults(EncFaults *e);
void startEncoderChecking(EncFaults *e,uint16_t absRPM);
void stopEncoderChecking(EncFaults *e,uint16_t absRPM);

#endif