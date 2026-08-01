#include "FrictionStart.h"

void InitFrictionStart(frictionStart *f,uint16_t usThresh,uint16_t addIq){
  f->addIq = addIq;
  f->addIqF = addIq;
  f->usCounterThreshold = usThresh; 
}

void FrictionStart_Begin(frictionStart *f){
  f->oneTime = 1;
  f->usCounter = 0;
}
