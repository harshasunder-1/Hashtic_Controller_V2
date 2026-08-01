#ifndef FDCAN_FDCAN_H_
#define FDCAN_FDCAN_H_

#include "stm32g4xx_hal.h"

#define TMCM_CAN_ADDRESS 0x20
#define PCM_CAN_ADDRESS 0x10

#define MOTORSTATE_FUNCTIONID			0x32 //from PCM 0x32 , from PCAN 0x77
#define PRECHARGE_FUNCTIONID			0x33 // from PCM 0x33, from PCAN 0x76
#define CONTINUOUS_DATA                         0x34

//MotorActions for one shot Ramp input

#define STOP_COASTING 1  
#define MOTORON 2
#define SS_STOP_REGEN 3
#define SS_STOP_COAST_MECH_BRAKE 4
#define SS_DISENGAGE_MECHBRAKE 5
#define SS_ENGAGE_MECHBRAKE 6
#define SS_REGEN_MECH_BRAKE 7

#define CC_DATA_IP 3
#define CC_STOP_REGEN 4
#define CC_STOP_COAST_MECHBRAKE 5
#define CC_DISENGAGE_MECHBRAKE 6
#define CC_ENGAGE_MECHBRAKE 7


#define RESET_TMCM 8


typedef struct{
  uint8_t motorAction;
  uint16_t targetRPM;
  uint16_t rampupTime;
  uint8_t direction;
  uint16_t steadyStateTime;
  uint8_t stopCommand;
}CAN_SS_Input; //single shot input

typedef struct{
  uint8_t motorAction;
  uint8_t direction;
  uint16_t controlQty;
  uint16_t indexID;
}CAN_Continuous_Input; //single shot input

extern CAN_SS_Input canSSIp;
extern CAN_Continuous_Input canContIp;

void FDCAN_TxInit(void);
void FDCAN_RxFilterInit(void);
void FDCAN_parseForMotor(void);
void FDCAN_runtimedataFromMotor(void);
void FDCAN_SendBrakeMsg(uint8_t breakEnable);
void FDCAN_SendPCMAckMsg(uint8_t msgType);
uint8_t CheckCanInputs(CAN_SS_Input *r);
uint8_t CheckCanInputsContinuous(CAN_Continuous_Input *s);
void FDCAN_CC_TMCM_log(void);
void FDCAN_TMCM_StopFrame(uint8_t errorReason);
void FDCAN_CC_TMCM_sendStatusData(void);
void FDCAN_CC_TMCM_sendRunTimeData(void);
void FDCAN_CC_TMCM_sendRPM_To_CCM(void);
void FDCAN_SendStopBrakeMsg(void);
void FDCAN_SendControlledBrakeMsg(void);
void FDCAN_SendSlamBrakeMsg(void);


extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_RxHeaderTypeDef   RxHeader;
extern FDCAN_TxHeaderTypeDef   TxHeader;


#endif /* FDCAN_FDCAN_H_ */
