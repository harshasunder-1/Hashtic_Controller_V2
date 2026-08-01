
#ifndef CONTROLFUNCTIONS_H_
#define CONTROLFUNCTIONS_H_

#include "stdint.h"
#include "StateMachine.h"

typedef struct {
  int16_t targetRPM;
  uint16_t rampUpTime;
  uint16_t steadyStateTime;
  uint16_t changeRPMTime;
  uint8_t start;
  uint8_t stop;
  uint8_t changeRPM;
}laptopControl;


typedef struct {
  uint16_t idx;
  int16_t target;
  int16_t direction;
  uint16_t stopIdx;
   
  uint8_t start;
  uint8_t stop;
  uint8_t increment_signal;
  uint8_t timerOn_bool;
}laptopContinuousControl;

#define ONE_SHOT_REGEN_FIXED_IQ 500


void applyRegen(int currentDirection);
void laptopRun(laptopControl *lc,systemState *ss);
void laptopStop(laptopControl *lc,systemState *ss);
void laptopChangeRPM(laptopControl *lc);


void laptopCC_on(laptopContinuousControl *lcc);
void laptopCC_increment(laptopContinuousControl *lcc);
void laptopCC_stop(laptopContinuousControl *lcc);

#endif
