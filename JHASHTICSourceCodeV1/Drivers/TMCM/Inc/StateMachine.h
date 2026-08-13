#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

#include "stdint.h"
#include "main.h" //only for custom faults - remove
#include "state_machine.h"

typedef enum {
  CC_IDLE,
  CC_RUNNING_CL,
  CC_RUNNING_OL,
  CC_ERROR,
  CC_FINISH,
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
  int8_t podDirection;
  int16_t targetRPM;
  uint16_t brakeDistance;
  int8_t motorDirection; 
   
  uint16_t indexID;
  cc_state_t cc_state ; //continuous control state
  cc_rampState cc_ramp ;
  int16_t currentRPM;
  uint16_t currentAbsRpm;
  uint16_t iqRef;
  int8_t brakeState;
  uint8_t engageBrake;
  uint8_t CL_DeltaRPMThreshold;

  float travelledDist;

  int16_t phaseVoltageRMS;
  int16_t phaseCurrentRMS;
  
  int16_t DC_VOLTAGE;
  State_t MCSDK_STATE;

  uint8_t MCSDK_PreFault;
  uint8_t MCSDK_CurrentFault;
  CustomErrors CustomFaults;
  uint8_t neverStarting;
  
}systemState;


typedef struct {
  uint8_t timerOnBool;
  uint8_t PCM_timer_thresh;
  volatile uint8_t PCM_timer;
}continousControlTimer;

typedef struct{
  uint8_t brakeOneTime;
  uint8_t brakeCounter;
  uint32_t brakeTime1;
}brakeCtrl;



uint8_t CheckPodState(systemState *ss);
void updateTMCMState(systemState *ss);
uint8_t TMCM_SpeedLoop_TurnOff(void);
uint16_t Calculate_CLDeltaRPMThreshold(uint16_t targetRpm);
#endif
