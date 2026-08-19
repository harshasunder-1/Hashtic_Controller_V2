#include "stdint.h"
#include "StartSequence.h"
#include "motorcontrol.h"
#include "Configuration.h"

extern FOCVars_t FOCVars[1];

void initializeStartSeqParams( startSeq *ssq_){
  ssq_->currentState = IDLE_WAITING;  
  ssq_->errorState = NO_ERROR;
  
  ssq_->globalStartSeqTimer = 0;
  ssq_->PCM_startCommand = 0;
}

void resetStartSeqParams( startSeq *ssq_){
  ssq_->currentState = IDLE_WAITING;  
  ssq_->globalStartSeqTimer = 0;
  ssq_->PCM_startCommand = 0;
}

void resetStartSeqErrorState( startSeq *ssq_){
  ssq_->errorState = NO_ERROR;
}

int16_t currentTorque;
ErrorState ExecStartSeq(startSeq *ssq_,systemState *ss){
  
  switch (ssq_->currentState) {
    
    case IDLE_WAITING:
        if (ssq_->PCM_startCommand == 1) { 
            //MC_ProgramTorqueRampMotor1(0, 0); // asynchronous
            //Whenever we apply either a torque ramp or speed ramp, inside the calcCurrRef function we will 
            // see signForCW and apply the correct Iq. But here we dont need to do anything.
            int16_t startingTorque = START_TORQUE_IQ * ss->motorDirection;
            MC_ProgramTorqueRampMotor1(startingTorque,T_RAMP_DONE);
            MC_StartMotor1();
            ssq_->globalStartSeqTimer = 0;
            ssq_->currentState = START_SEQUENCE;
        }
        break;

    case START_SEQUENCE:
      
        // 1. SUCCESS CONDITION (With Noise Debouncing)
        if (ss->currentAbsRpm >= RPM_HANDOVER) {

          // Bumpless transfer with clamping
            PID_SetUpperOutputLimit(&PIDSpeedHandle_M1, IQMAX);
            PID_SetLowerOutputLimit(&PIDSpeedHandle_M1, -IQMAX);
            
            uint16_t kiDivisor= PID_GetKIDivisor(&PIDSpeedHandle_M1);
            
            //Here current torque is electrical torque with a sign, but the the integral term in the PI loop needs to be 'logical' torque.
            //ie: it has to be +ve current for +ve speed. multiplying like below changes electrical torque back to logical torque
            //if needed because the winding is opposite.
            currentTorque = FOCVars[0].Iqdref.q; // this has a -ve sign for FWD motion if winding is -1
            int32_t integralTerm = 0 ;
            integralTerm = kiDivisor * currentTorque * c.signForCWRotation; 
            PID_SetIntegralTerm(&PIDSpeedHandle_M1,integralTerm);
            
           // MC_StopMotor1(); // uncomment and test only the starting torque
            
            MC_ProgramSpeedRampMotor1(ss->targetRPM /6 * ss->motorDirection, 8000 );
            //sendPCM_MSG(currentRPM, currentPosition);
            ssq_->currentState = TRANSITION_DONE;
            break;
            }

        // 2. EARLY STALL CHECK
        if (ssq_->globalStartSeqTimer >= T_STALL_CHECK) {
            if (ss->currentAbsRpm < RPM_STALL_MIN) {
                ssq_->errorState = START_STALL;
                MC_StopMotor1();
                ssq_->currentState = IDLE_WAITING; 
                break;
            }
        }

        // 3. FINAL TIMEOUT
        if (ssq_->globalStartSeqTimer >= T_HANDOVER_MAX) {
            ssq_->errorState = SPEED_HANDOVER_TIMEOUT;
            MC_StopMotor1();
            ssq_->currentState = IDLE_WAITING;
            break;
        }
        break;

    /*case WAIT_PCM:
        // GLOBAL ABORT check here as well
        if (!PCM_Start_Enable) {
            stopMotor();
            currentState = IDLE;
            break;
        }

        // Timeout check for PCM spatial limit
        if (currentPosition >= 5 && PCM_Handover == 0) {
            error = PCM_HANDOVER_TIMEOUT;
            stopMotor();
            currentState = IDLE;
            break;
        }
        
        // Successful Handover
        if (PCM_Handover == 1) {
            currentState = IDLE;
        }
        break;
*/
  } // closes switch statements
  
  return ssq_->errorState;
}
