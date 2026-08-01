/*
* FDCAN.c
*
*  Created on: Mar 5, 2023
*      Author: Jonathan
*/

#include "FDCAN.h"
#include "mc_type.h"
#include "motorcontrol.h"
#include "PRECHARGE.h"
#include "Ramp.h"
#include "StateMachine.h"
#include "ControlFns.h"
#include "TemperatureLogic.h"
#include "Configuration.h"
#include "EncFaults.h"
#include "StartSequence.h"
#include <stdlib.h>

extern RampingOperation RampOp;
extern RampState rampstate;
extern FDCAN_HandleTypeDef hfdcan2;
extern FOCVars_t FOCVars[1];
extern systemState ss;
extern continousControlTimer ccT;
extern Temp t;
extern brakeCtrl b;
//CAN variables here
uint32_t functionID,source_address, destination_address;

FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
extern FDCAN_RxHeaderTypeDef   RxHeader;
extern uint8_t RxData[8];
FDCAN_FilterTypeDef sFilterConfig;

extern int16_t hTargetSpeedUserDefined;
extern FOCVars_t FOCVars[1];
extern uint8_t PCOn;
uint8_t inputsOk,podOK;
extern float k;
extern uint8_t CircleLimitationState;

extern uint8_t cc_stopMsg_oneTime;
extern uint8_t cc_turnOff;

void FDCAN_TxInit(void)
{
  if(HAL_FDCAN_Start(&hfdcan2)!= HAL_OK)
  {
    //Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    //Error_Handler();
  }
  TxHeader.Identifier = 0x0E090102;//This is our identifier
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  TxHeader.FDFormat = FDCAN_FD_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}

void FDCAN_RxFilterInit(void)
{
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK; //FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00;//destination address of flyer 0x00000200 (uint32_t)S.CAN_ID<<8 // 2 - is just a number
  sFilterConfig.FilterID2 = 0x7FFFFFFF;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    //Error_Handler();
  }
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
  
  //HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0);
  RxData[0]=RxData[1]=RxData[2]=RxData[3]=RxData[4]=0;
}


uint8_t CheckCanInputs(CAN_SS_Input *s){
    uint8_t IsSpeedUnderLimit = 0,IsRampUnderLimit = 0,IsDirectionOk=0;
    float rampTemp=0;
      
    if(s->targetRPM > 100 && s->targetRPM < 2000){
        IsSpeedUnderLimit = 1;
      }
      
    rampTemp = (float)s->targetRPM /s->rampupTime ;
    if(rampTemp <=0.15f && rampTemp > 0.015f){ //min ramptime = 2s , max ramptime = 20s
      IsRampUnderLimit = 1;
    }
    
    if ((s->direction == 0xFF) || (s->direction == 0xAA)){
      IsDirectionOk = 1;
    }
    
    if ((IsSpeedUnderLimit == 1 ) && (IsRampUnderLimit == 1) &&(IsDirectionOk == 1)){
      return 1;
    }else{
      return 0;
    }
}



uint8_t CheckCanInputsContinuous(CAN_Continuous_Input *s){
    uint8_t IsDirectionOk=0,startInputsOK=0;
    
    if ((s->direction == 0xFF) || (s->direction == 0xAA)){
      IsDirectionOk = 1;
    }
    
    if ((s->controlQty == 0) && (s->indexID == 0)){
      startInputsOK = 1;
    }
    
    if (IsDirectionOk==1 && startInputsOK==1){
      return 1;
    }else{
      return 0;
    }
}



void FDCAN_runtimedataFromMotor(void)
{ 
  TxHeader.Identifier = 0x7781020;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  
  TxData[0]= (hTargetSpeedUserDefined)>>8;
  TxData[1]= hTargetSpeedUserDefined;
  TxData[2]= ss.currentRPM>>8;
  TxData[3]= ss.currentRPM;
  TxData[4]= (uint8_t)(ss.DC_VOLTAGE/2);
  TxData[5]= FOCVars[0].Iqdref.q>>8;
  TxData[6]= FOCVars[0].Iqdref.q;
  if (ss.CustomFaults != NO_FAULTS){
    TxData[7] = ss.CustomFaults;
  }else{
  TxData[7]= ss.MCSDK_PreFault;
  }
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}


void FDCAN_CC_TMCM_sendRunTimeData(void)
{ 
  TxHeader.Identifier = 0x7801020;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  
  TxData[0]= ss.indexID>>8;
  TxData[1]= ss.indexID;
  TxData[2]= ss.targetRPM>>8;
  TxData[3]= ss.targetRPM;
  TxData[4]= ss.currentRPM>>8;
  TxData[5]= ss.currentRPM;
  TxData[6]= FOCVars[0].Iqdref.q>>8;
  TxData[7]= FOCVars[0].Iqdref.q;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}

void FDCAN_CC_TMCM_sendRPM_To_CCM(void)
{ 
  TxHeader.Identifier = 0x18353020;//30 dst - is brake, 20 s-src is tmcm
  TxHeader.DataLength = FDCAN_DLC_BYTES_2;
  
  TxData[0]= ss.currentRPM>>8;
  TxData[1]= ss.currentRPM;

  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}



void FDCAN_CC_TMCM_sendStatusData(void){
  TxHeader.Identifier = 0x7811020;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  
  TxData[0]= t.motorTempC;
  TxData[1]= t.mosfetTempC;
  TxData[2]= ss.DC_VOLTAGE/2;
  if (ss.CustomFaults != NO_FAULTS){
    TxData[3] = ss.CustomFaults;
  }else{
  TxData[3]= ss.MCSDK_PreFault;
  }
  TxData[4]= FOCVars[0].Iqd.q>>8;
  TxData[5]= FOCVars[0].Iqd.q;
  TxData[6]= ss.phaseVoltageRMS>>8;
  TxData[7]= ss.phaseVoltageRMS;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}


void FDCAN_CC_TMCM_log(void)
{ 
  TxHeader.Identifier = 0x7801020;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  
  TxData[0]= ss.indexID>>8;
  TxData[1]= ss.indexID;
  TxData[2]= ss.targetRPM>>8;
  TxData[3]= ss.targetRPM;
  TxData[4]= ss.currentRPM>>8;
  TxData[5]= ss.currentRPM;
  TxData[6]= FOCVars[0].Iqdref.q>>8;
  TxData[7]= FOCVars[0].Iqdref.q;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
  
  
  TxHeader.Identifier = 0x7811020;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  
  TxData[0]= t.motorTempC;
  TxData[1]= t.mosfetTempC;
  TxData[2]= ss.DC_VOLTAGE/2;
  if (ss.CustomFaults != NO_FAULTS){
    TxData[3] = ss.CustomFaults;
  }else{
  TxData[3]= ss.MCSDK_PreFault;
  }
  TxData[4]= CircleLimitationState;
  TxData[5]= 0;
  TxData[6]= 0;
  TxData[7]= 0;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
  
}




void FDCAN_SendStopBrakeMsg(void)
{
  TxHeader.Identifier =(0x18343020);  //src 0x30-tmcm, dst 0x20 - brake board 
  TxHeader.DataLength = FDCAN_DLC_BYTES_3;
  TxData[0]= 0;
  TxData[1]= 0;
  TxData[2]= 0;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}

void FDCAN_SendControlledBrakeMsg(void)
{
  TxHeader.Identifier =(0x18343020);  //src 0x30-tmcm, dst 0x20 - brake board 
  TxHeader.DataLength = FDCAN_DLC_BYTES_3;
  TxData[0]= 0;
  TxData[1]= 0;
  TxData[2]= 1;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}
void FDCAN_SendSlamBrakeMsg(void)
{
  TxHeader.Identifier =(0x18343020);  //src 0x30-tmcm, dst 0x20 - brake board 
  TxHeader.DataLength = FDCAN_DLC_BYTES_3;
  uint16_t rpm = 300;
  TxData[0]= rpm>>8;
  TxData[1]= rpm;
  TxData[2]=1;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}

void FDCAN_SendPCMAckMsg(uint8_t msgType)
{
  TxHeader.Identifier =(0x7791020); 
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxData[0]= msgType;
  TxData[1]= 0x00;
  TxData[2]= 0x00;
  TxData[3]= 0x00;
  TxData[4]= 0x00;
  TxData[5]= 0x00;
  TxData[6]= 0x00;
  TxData[7]= 0x00;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
}


void FDCAN_TMCM_StopFrame(uint8_t errorReason){
  TxHeader.Identifier =(0x7821020); 
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxData[0]= errorReason;
  TxData[1]= 0x00;
  TxData[2]= 0x00;
  TxData[3]= 0x00;
  TxData[4]= 0x00;
  TxData[5]= 0x00;
  TxData[6]= 0x00;
  TxData[7]= 0x00;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, TxData);
  
}

uint32_t t1=0,prevTick=0,dT=0;
uint8_t firstMsg = 0;
uint8_t deltaID=0;
uint16_t dRPM=0;

void FDCAN_parseForMotor(void){ // This gets toggled inside the interrupt. Whenever a reception happened on the CAN

  functionID=((RxHeader.Identifier)&0xFF0000)>>16;
  source_address=(RxHeader.Identifier)&0xFF;
  
  switch (functionID) {
    
  case MOTORSTATE_FUNCTIONID:
    
    canSSIp.motorAction = RxData[0];
    canSSIp.direction = RxData[1];
    canSSIp.targetRPM = ((RxData[3]<<8)|(RxData[2]));
    canSSIp.rampupTime =  ((RxData[5]<<8)|(RxData[4]));
    canSSIp.steadyStateTime =  ((RxData[7]<<8)|(RxData[6]));
        
    if(canSSIp.motorAction == STOP_COASTING){ 
      MC_StopMotor1();
      RampTurnOff();
      MC_Clear_IqdrefMotor1();
      hTargetSpeedUserDefined=0;
      ss.cc_ramp = CC_RAMPOFF;
      FDCAN_SendPCMAckMsg(1);
    }
    
    else if(canSSIp.motorAction == MOTORON){
      
      inputsOk = CheckCanInputs(&canSSIp); // checks set RPM , and checks ramp up time
      podOK = CheckPodState(&ss);  // checks if motor state is not run, checks if pod is not moving, checks if encoder is OK
      //also check temperature 
      
      if(inputsOk == 1 && podOK==1){
        ss.targetRPM = canSSIp.targetRPM;
        if(canSSIp.direction==0xFF){ss.direction = 1;}//Forward direction
        else if(canSSIp.direction==0xAA){ss.direction = -1;}//reverse direction
        
        RampOp.steadyStateTime_s = canSSIp.steadyStateTime*0.001; 
        RampOp.rampUpTime = canSSIp.rampupTime; // need this later to check if we re taking too long to rampUp
        
        //reset circle limitation
        k = 10;CircleLimitationState = 0;
        ResetEncFaults(&encFlts);
        
        MC_ProgramSpeedRampMotor1(ss.direction*ss.targetRPM/6, RampOp.rampUpTime);
        MC_StartMotor1();
        StartRamps(); 
        ss.CustomFaults = NO_FAULTS;
        ss.runType=SS;
        
        FDCAN_SendPCMAckMsg(1);
        cc_stopMsg_oneTime = 0;
      }
      else{ 
        ss.targetRPM=0;
        MC_StopMotor1();
        RampTurnOff();
        MC_Clear_IqdrefMotor1();
        hTargetSpeedUserDefined = 0;
        FDCAN_SendPCMAckMsg(2);
      }
    }
    
    else if (canSSIp.motorAction == SS_STOP_REGEN){
      // stop motor with regen braking
      applyRegen(ss.direction);
      rampstate = RAMPING_DOWN;
      RampOp.startSteadyStateTimerBool =0; //stop steady state timer
      RampOp.steadyStateElapsedTime = 0;
      ss.cc_ramp = CC_RAMPOFF;
      ss.runType=NO_RUN;
      FDCAN_SendPCMAckMsg(1);
    }
    
    else if (canSSIp.motorAction == SS_STOP_COAST_MECH_BRAKE){
      // coasting and mech braking
      ss.targetRPM=0;
      MC_StopMotor1();
      RampTurnOff();
      MC_Clear_IqdrefMotor1();
      hTargetSpeedUserDefined = 0;
      ss.cc_ramp = CC_RAMPOFF;
      ss.runType=NO_RUN;
      //FDCAN_SendBrakeMsg(1); //break engage
      FDCAN_SendControlledBrakeMsg();
      ss.brakeState++;
      FDCAN_SendPCMAckMsg(1);
    }
    
    else if (canSSIp.motorAction == SS_DISENGAGE_MECHBRAKE){
      FDCAN_SendStopBrakeMsg(); //break disengage
      ss.brakeState--;
      FDCAN_SendPCMAckMsg(1);
    }
    
    else if (canSSIp.motorAction == SS_ENGAGE_MECHBRAKE){
      // coasting and mech braking
      ss.targetRPM=0;
      MC_StopMotor1();
      RampTurnOff();
      MC_Clear_IqdrefMotor1();
      hTargetSpeedUserDefined = 0;
      ss.cc_ramp = CC_RAMPOFF;
      ss.runType=NO_RUN;
     // FDCAN_SendSlamBrakeMsg(); //break engage
      FDCAN_SendControlledBrakeMsg();
      ss.brakeState++;
      FDCAN_SendPCMAckMsg(1);
    }
    
    else if (canSSIp.motorAction == SS_REGEN_MECH_BRAKE){
      // stop motor with regen braking and mech braking
      applyRegen(ss.direction);
      rampstate = RAMPING_DOWN;
      RampOp.startSteadyStateTimerBool =0; //stop steady state timer
      RampOp.steadyStateElapsedTime = 0;
      ss.cc_ramp = CC_RAMPOFF;
      ss.runType=NO_RUN;
      //FDCAN_SendBrakeMsg(1); //break engage 
      FDCAN_SendControlledBrakeMsg();
      ss.brakeState++;
      FDCAN_SendPCMAckMsg(1);
    }
    else if(canSSIp.motorAction == RESET_TMCM){
      FDCAN_SendPCMAckMsg(1);
      HAL_Delay(10);
      HAL_NVIC_SystemReset(); // reset Motor!Not working
    }
    else{
    }
    break;
    
        
  case PRECHARGE_FUNCTIONID: //always make a sound

    pcv.Precharge_Stage = PRECHARGE_START;
    PCOn = RxData[0];
    ss.targetRPM =  0;
    ss.indexID  = 0;
    ss.travelledDist = 0;
    ss.cc_state =CC_IDLE;
    ss.CustomFaults = NO_FAULTS; //remove faults when u get precharge
    b.brakeCounter = 0;
    
    //reset start seq
    resetStartSeqParams( &ssq);
    resetStartSeqErrorState( &ssq);
    
    
    PID_HandleInit(&PIDSpeedHandle_M1);   
    PID_HandleInit(&PIDIqHandle_M1);
    PID_HandleInit(&PIDIdHandle_M1);
    
    FDCAN_SendPCMAckMsg(1);
    break;
    
  case CONTINUOUS_DATA:
    
    canContIp.motorAction = RxData[0];
    canContIp.direction = RxData[1];
    canContIp.controlQty = ((RxData[3]<<8)|(RxData[2]));
    canContIp.indexID =  ((RxData[5]<<8)|(RxData[4]));
      
    
    if (canContIp.motorAction == MOTORON){ 
      /*if motor is already running : turn off , else get direction, check if other data is empty , then send ACK.
        Prepare for continous values, by starting a timer , and resetting the index values to zero. can turn on PWM with Zero duty
        check temperatures within limits, encoder reading is coming properly also.*/
        podOK = CheckPodState(&ss);  // checks if motor state is not run, checks if pod is not moving, checks if encoder is OK
        inputsOk =  CheckCanInputsContinuous(&canContIp);
        if (podOK && inputsOk){
          ss.indexID = 0;
          ss.targetRPM = 0;
          if(canContIp.direction==0xFF){
            if (c.positionInPod == LEFT_SIDE){ss.direction = -1;}
            if (c.positionInPod == RIGHT_SIDE){ss.direction = 1;}
          }
          else if(canContIp.direction==0xAA){
            if (c.positionInPod == LEFT_SIDE){ss.direction = 1;}
            if (c.positionInPod == RIGHT_SIDE){ss.direction = -1;}
          }
          
          firstMsg = 1;
          //start Timer 
          //ccT.PCM_timer = 0;
          //ccT.timerOnBool = 1;
          ss.CustomFaults = NO_FAULTS;
          ss.cc_state = CC_RUNNING;
          ss.cc_ramp = CC_RAMPUP;
          ss.runType=CC;
          ss.travelledDist = 0;
          
          ResetEncFaults(&encFlts);
          EnableEncoderFltChking(&encFlts);
          
          ResetEncFaults(&encFlts);
          FDCAN_SendPCMAckMsg(1);
                    
          //start the StartSeq
          ssq.PCM_startCommand = 1;
          
          //MC_ProgramSpeedRampMotor1(ss.direction * ss.targetRPM/6, 10);
          //MC_StartMotor1();
          

        }else{
          //send one error msg
          MC_StopMotor1();
          MC_Clear_IqdrefMotor1();
          FDCAN_SendPCMAckMsg(2);
          ss.cc_state=CC_IDLE;
          ss.cc_ramp = CC_RAMPOFF;
          ss.runType=NO_RUN;
        }        
    }
    else if (canContIp.motorAction == STOP_COASTING){
      //coasting Turn off
      //stop looking at any further continuous Data values
        MC_StopMotor1();
        MC_Clear_IqdrefMotor1();
        hTargetSpeedUserDefined = 0;
        FDCAN_SendPCMAckMsg(1);
        ss.cc_state=CC_IDLE;
        ss.cc_ramp = CC_RAMPOFF;
        ss.runType=NO_RUN;
    } 
    /*else if (canContIp.motorAction == CC_DATA_IP){ 
      //check if time we ve got it in is correct
      //check if index is correct. check if control Quantity value is plausible. 
      //apply if OK..if weve errored, then more msgs like this needs to be handled, with
      // another error

      if (ss.cc_state != CC_ERROR){
        ccT.PCM_timer = 0;
        deltaID = canContIp.indexID - ss.indexID;
        dRPM = abs(canContIp.controlQty - ss.targetRPM);
        
        if (canContIp.controlQty < ss.targetRPM){ //in ramp up till we see a msg where the new value is less than the old value
          ss.cc_ramp = CC_RAMPDOWN;
        }
        
        if (ss.cc_ramp == CC_RAMPDOWN){
          if (canContIp.controlQty  < 30){
            cc_turnOff = 1; //onetime turnoff in while loop
          }
        }
        
        if (((deltaID <= 5) && (dRPM < 25)) || (firstMsg==1)){ //first msg index is zero , so it fails the delta ID check
          ss.targetRPM = canContIp.controlQty;
          ss.indexID = canContIp.indexID;
          firstMsg = 0;
          //MC_ProgramSpeedRampMotor1(ss.direction * ss.targetRPM/6, 10); 
        }else{            
          ss.CustomFaults = PCM_BAD_CAN_MSG;
          ss.cc_state = CC_ERROR;
        }
        FDCAN_CC_TMCM_sendRunTimeData();
        FDCAN_CC_TMCM_sendRPM_To_CCM(); // seperate msg to CCM, later remove it?
      }
    }*/
    
    
    /*else if (motorAction == TURN_OFF_W_REGEN){
      //turn OFF with fixed Regen.
      //stop looking at any further continuous Data values
    }
    else if (motorAction == TURN_OFF_COASTING_W_MECHBRAKE){
      //turn OFF and apply mech brake.
      //stop looking at any further continuous Data values
    }
    else if (motorAction == ONLY_MECH_BRAKE_ENGAGE){
      //if turned off and motor not working apply mech brake.
      //put flag and do not allow motor to run if mech brake is engaged. 
      //needs to move out from here.
    }
    else if (motorAction == ONLY_MECH_BRAKE_DISENGAGE){
      //turn OFF and apply mech brake.
      //stop looking at any further continuous Data values
    }

*/

  default:
    break;
  }
}
