#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "stdint.h"
#include "main.h" //only for custom faults - remove
#include "state_machine.h"

typedef enum {
  CC_IDLE,
  CC_RUNNING,
  CC_ERROR,
}cc_state_t;


typedef enum {
  CC_RAMPOFF,
  CC_RAMPUP,
  CC_RAMPDOWN,
}cc_rampState;

typedef enum {
  CC, //continuous control
  SS, //single shot
  LCC, //laptop continous 
  LSS, //laptop single shot
  NO_RUN, 
}runType;

typedef struct{

  runType runType;
  int8_t direction;
  int16_t targetRPM;
   
  uint16_t indexID;
  cc_state_t cc_state ; //continuous control state
  cc_rampState cc_ramp ;
  int16_t currentRPM;
  uint16_t currentAbsRpm;
  uint16_t iqRef;
  uint8_t brakeState;
  float travelledDist;

  int16_t phaseVoltageRMS;
  int16_t phaseCurrentRMS;
  
  int16_t DC_VOLTAGE;
  State_t MCSDK_STATE;

  uint8_t MCSDK_PreFault;
  uint8_t MCSDK_CurrentFault;
  CustomErrors CustomFaults;
  
}systemState;


typedef struct {
  uint8_t timerOnBool;
  uint8_t PCM_timer_thresh;
  volatile uint8_t PCM_timer;
}continousControlTimer;

typedef struct{
  uint8_t brakeOneTime;
  uint16_t thresholdBrake1;
  uint16_t thresholdBrake2;
  uint8_t brakeCounter;
}brakeCtrl;



uint8_t CheckPodState(systemState *ss);
void updateTMCMState(systemState *ss);

#endif