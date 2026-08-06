#include "Ramp.h"

RampingOperation RampOp;
RampState rampstate;

extern MCI_Handle_t * pMCI[0];
extern int16_t hTargetSpeedUserDefined;


void StartRamps(void){
  RampOp.steadyStateElapsedTime = 0;
  RampOp.startSteadyStateTimerBool = 0;
  RampOp.overallTimer = 0;
  RampOp.start_overallTimerBool = 1;
  rampstate = RAMPING_UP;
}

void ResetRamp(void){
  RampOp.steadyStateTime_s = 0; 
  RampOp.overallTimer = 0;
  RampOp.start_overallTimerBool = 0;
  rampstate = MOTOR_OFF;
}

void Ramping(systemState *ss){

  if (rampstate == RAMPING_UP){
  	RampOp.IsRampUpCompleted = MCI_RampCompleted(pMCI[M1]);
 	if(RampOp.IsRampUpCompleted == 1){ // if ramp up complete and speed has reached target rpm, go into steady state
          if(ss->currentAbsRpm >= ss->targetRPM){
              rampstate = STEADY_STATE;
              RampOp.steadyStateElapsedTime = 0;
              RampOp.startSteadyStateTimerBool = 1;
          }
      }    
  }
  
  if (rampstate == STEADY_STATE){
    if(RampOp.steadyStateElapsedTime >= RampOp.steadyStateTime_s){
      // REGEN 
      applyRegen(ss->direction);
      rampstate = RAMPING_DOWN;
      RampOp.startSteadyStateTimerBool =0; //stop steady state timer
      RampOp.steadyStateElapsedTime = 0;
      }
    }
  
  if (rampstate == RAMPING_DOWN){
    RampOp.IsTargetRampDownCompleted = MCI_RampCompleted(pMCI[M1]);
    if(RampOp.IsTargetRampDownCompleted == 1 && (ss->currentAbsRpm <= RAMPDOWN_TURNOFF_SPEED)){     
      ss->targetRPM=0;
      TMCM_SpeedLoop_TurnOff(); 
      RampTurnOff();
      hTargetSpeedUserDefined = 0;
    }
  } 
}
 
void RampTurnOff(void){
  rampstate = MOTOR_OFF;
  RampOp.start_overallTimerBool = 0;
  RampOp.overallTimer = 0;
  RampOp.steadyStateElapsedTime = 0;
  RampOp.startSteadyStateTimerBool = 0;
}
void OverallTimerTurnOff(systemState *ss){
  // Turn Off the mtor if the entire running time exceeds the actual runtime specified for operation
  if(RampOp.overallTimer >= TOTAL_RUNTIME_SEC){
      ss->targetRPM=0;
      TMCM_SpeedLoop_TurnOff(); 
      RampTurnOff();
      hTargetSpeedUserDefined=0;
    } 
}