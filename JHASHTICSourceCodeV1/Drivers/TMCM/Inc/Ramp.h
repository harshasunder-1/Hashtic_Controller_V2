#ifndef RAMP_H_
#define RAMP_H_

#include "stm32g4xx_hal.h"

#include "mc_interface.h"
#include "main.h"
#include "mc_api.h"
#include "ControlFns.h"
#include "StateMachine.h"

#define TOTAL_RUNTIME_SEC 120 // number that will turnoff the motor, whatever the rampup state. HAve it incase we wnat to use it for quick changes and debugging
#define STEADYSTATE_RUNTIME 120  //how long we want the pod to run at setRPM. can be Zero.CANT be -ve
#define RAMPDOWN_TURNOFF_SPEED 100 // RPM where we turn off the motor.
#define RAMPDOWN_TIME 6000 // CANT BE -ve. HAS TO BE SET CORRECTLY , so that from setRPM to 100 doesnt require large braking, set in ms!
#define START_TIME_THRESHOLD_S 5 
#define RPM_CLOSEDLOOP_THRESHOLD 100

typedef struct{
  uint8_t overallTimer;
  uint8_t start_overallTimerBool;
  uint16_t rampUpTime;
  uint8_t IsRampUpCompleted;
  uint8_t startSteadyStateTimerBool;
  float steadyStateTime_s;
  uint8_t steadyStateElapsedTime;
  uint8_t IsTargetRampDownCompleted;  
} RampingOperation;


typedef enum 
{
  RAMPING_UP,
  STEADY_STATE,
  RAMPING_DOWN, 
  MOTOR_OFF
} RampState;

void ResetRamp(void);
void Ramping(systemState *ss);
void StartRamps(void);
void RampTurnOff(void);
void OverallTimerTurnOff(systemState *ss);
#endif
