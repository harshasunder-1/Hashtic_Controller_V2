#ifndef FRICTION_START_H_
#define FRICTION_START_H_

#include "stdint.h"


#define FRICTION_START_IQ 100  // for LCC, 100
#define FRICTION_START_CYCLES 5000 //each cycle is 100us, we want about 200ms, so 200*10, 2000

typedef struct {
  uint8_t oneTime;
  int16_t addIq;
  float addIqF;
  uint16_t usCounter;
  uint16_t usCounterThreshold;
  int16_t IqRef_initial;
  int16_t IqRef_withFF;
  
}frictionStart;


void InitFrictionStart(frictionStart *f,uint16_t usThresh,uint16_t addIq);
void FrictionStart_Begin(frictionStart *f);

extern frictionStart fs;








#endif
