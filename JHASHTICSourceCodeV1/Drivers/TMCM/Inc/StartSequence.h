#ifndef START_SEQ_H_
#define START_SEQ_H_

#include "StateMachine.h"


//ON GROUND(800,2000,2400,5000,5,50,600)
//LIFTED (500,800,1000,1000,5,50,200)

#define T_RAMP_DONE 500 //time to ramp up the Torque
#define T_STALL_CHECK 800 // time before which we should see some movement
#define T_HANDOVER_MAX 1000 // time in which if we dont see required speed we turn off

#define START_TORQUE_IQ 1000//command to start the torque
#define RPM_STALL_MIN 5 // RPM below which we consider it a stall
#define RPM_HANDOVER 50 
#define RPM_STEADYSTATE 200

enum StartState { IDLE_WAITING, START_SEQUENCE, WAIT_PCM };
typedef enum { NO_ERROR, START_STALL,SPEED_HANDOVER_TIMEOUT, PCM_HANDOVER_TIMEOUT }ErrorState;

typedef struct{
  uint8_t currentState;
  ErrorState errorState;
  
  uint16_t globalStartSeqTimer;
  
  uint8_t PCM_startCommand;
 
}startSeq;

extern startSeq ssq;

void initializeStartSeqParams( startSeq *ssq_);
ErrorState ExecStartSeq(startSeq *ssq_,systemState *ss);

void resetStartSeqParams( startSeq *ssq);
void resetStartSeqErrorState( startSeq *ssq);


#endif