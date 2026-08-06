//State Machine

#include "StateMachine.h"
#include "state_machine.h" //this is mcsdk state machine
#include "motorcontrol.h"

extern MCI_Handle_t * pMCI[NBR_OF_MOTORS];

uint8_t CheckPodState(systemState *ss){
  
    uint8_t IsPodStopped = 0;
    uint8_t isMotorStateRun = 0;
    uint8_t isMotorEncoderNOK = 0;
    uint8_t isMotorBrakeDisengaged = 0;
    
    //if we get a command when the vehicle is moving, dont allow it. Infact stop the pod.
    uint16_t absRPM = 0;
    absRPM = ss->currentRPM;
    if (ss->currentRPM < 0){absRPM = -(ss->currentRPM);}
                
    // whether pod is moving or not, it is already in a run state. so we dont use another start command, go to stop
    if ((ss->MCSDK_STATE == START) || (ss->MCSDK_STATE == START_RUN)){  
      isMotorStateRun = 1;
    }
    if(absRPM < 5){IsPodStopped = 1;} // only allow start if pod is stopped.
    if(ss->CustomFaults != NO_FAULTS){isMotorEncoderNOK = 1;}
    if(ss->brakeState == 0){isMotorBrakeDisengaged = 1;}
    
    if (IsPodStopped==1 && isMotorStateRun==0 && isMotorEncoderNOK==0 && isMotorBrakeDisengaged == 1){
      return 1;
    }else{
      return 0;
    }
}

void updateTMCMState(systemState *ss){  
  ss->DC_VOLTAGE = VBS_GetAvBusVoltage_V(&RealBusVoltageSensorParamsM1._Super);
  ss->currentRPM= 6*MC_GetMecSpeedAverageMotor1();
  
  uint16_t absRPM = 0;
  absRPM = ss->currentRPM;
  if (ss->currentRPM < 0){absRPM = -(ss->currentRPM);}
  
  ss->currentAbsRpm = absRPM;
  ss->MCSDK_PreFault =  MC_GetOccurredFaultsMotor1();
  ss->MCSDK_CurrentFault = MC_GetCurrentFaultsMotor1();
  ss->MCSDK_STATE = MC_GetSTMStateMotor1();
  ss->phaseVoltageRMS = (int16_t)((MC_GetPhaseVoltageAmplitudeMotor1())/1.414f);
  ss->phaseCurrentRMS = (int16_t)((MC_GetPhaseCurrentAmplitudeMotor1())/1.414f);
}


uint8_t TMCM_SpeedLoop_TurnOff(void){   
    MC_StopMotor1();      
    MC_ProgramSpeedRampMotor1(0, 0); // asynchronous
    MC_Clear_IqdrefMotor1();
    pMCI[0]->pSTC->TargetFinal = 0;
    pMCI[0]->pSTC->SpeedRefUnitExt = 0;
    pMCI[0]->pSTC->TorqueRef = 0;
    return 1;
}

