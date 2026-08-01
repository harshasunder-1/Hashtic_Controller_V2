#include "main.h"
#include "PRECHARGE.h"
#include "motorcontrol.h"


extern uint8_t PCOn, OneTimeContactorOpen;
void PrechargingLogic(void)
{
  
  if(PCOn == 1)
  {
    PCOn = 0;
    pcv.Precharge_Stage = PRECHARGE_START;
  }
  
  /*if(OneTimeContactorOpen == 1)
  {
    OneTimeContactorOpen = 0;
    pcv.Precharge_Stage =  PRECHARGE_OFF_COMMAND;
    HAL_GPIO_WritePin(CONTACTOR_COIL_GPIO_Port,CONTACTOR_COIL_Pin,GPIO_PIN_SET); // 1 - OFF
    HAL_Delay(CONTACTOR_TURN_OFF_ON_TIME);
    HAL_GPIO_WritePin(PRECHARGE_GPIO_Port,PRECHARGE_Pin,GPIO_PIN_SET); // 1 - OFF
    HAL_Delay(PRECHARGE_TURN_OFF_ON_TIME);
    HAL_GPIO_WritePin(ACTIVE_DISCHARGE_GPIO_Port,ACTIVE_DISCHARGE_Pin,GPIO_PIN_SET);
    HAL_Delay(ACTIVE_DISCHARGE_DELAY);
    HAL_GPIO_WritePin(ACTIVE_DISCHARGE_GPIO_Port,ACTIVE_DISCHARGE_Pin,GPIO_PIN_RESET);
  } */ 
  
  switch (pcv.Precharge_Stage)
  {
    
  case PRECHARGE_OFF:
    //Do Nothing
    pcv.allChecksDone = 0;
    break;
    
    
  case PRECHARGE_START:
    pcv.startCount = 1;
    pcv.auxContactorSignal = HAL_GPIO_ReadPin(CONTACTOR_AUX_GPIO_Port, CONTACTOR_AUX_Pin); // It is expected to be 1 at startup. After sucessful precharge, the value will change to 0
    HAL_GPIO_WritePin(PRECHARGE_GPIO_Port,PRECHARGE_Pin,GPIO_PIN_RESET);
    pcv.tim17_1sec_precharge = 0; // Reset the 1-second counter
    pcv.Precharge_Stage = PRECHARGE_WAITING;
    pcv.allChecksDone = 0;
    break;
    
  case PRECHARGE_WAITING:
    if (pcv.tim17_1sec_precharge >= 3)
    {
      pcv.Precharge_Stage = PRECHARGE_CHECK_VOLTAGE;
    }
    pcv.allChecksDone = 0;
    break;
    
  case PRECHARGE_CHECK_VOLTAGE:
    pcv.startCount = 0;
    if (VBS_GetAvBusVoltage_V(&RealBusVoltageSensorParamsM1._Super) >= UD_VOLTAGE_THRESHOLD_V) {
      pcv.Precharge_Stage = PRECHARGE_COMPLETE;
    }
    else {
      pcv.Precharge_Stage = PRECHARGE_ERROR; 
    }
    pcv.allChecksDone = 0;
    break;  
    
  case PRECHARGE_COMPLETE:
    HAL_GPIO_WritePin(CONTACTOR_COIL_GPIO_Port,CONTACTOR_COIL_Pin,GPIO_PIN_RESET); // 0 - ON
    HAL_Delay(200);
    pcv.Precharge_Stage = AUX_CHECK_PIN;
    pcv.allChecksDone = 0;
    break;    
    
  case AUX_CHECK_PIN:
    MC_AcknowledgeFaultMotor1();
    pcv.Precharge_Stage = AUX_COMPLETE;
    pcv.allChecksDone = 0;
    break; 
    
  case PRECHARGE_OFF_COMMAND:
    pcv.Precharge_Stage = PRECHARGE_OFF;
    break;
    
  case PRECHARGE_ERROR: //Turn off CONTACTOR, Precharge and Active Discharge
    pcv.allChecksDone = 1;
    HAL_GPIO_WritePin(CONTACTOR_COIL_GPIO_Port,CONTACTOR_COIL_Pin,GPIO_PIN_SET); // 1 - OFF
    HAL_Delay(CONTACTOR_TURN_OFF_ON_TIME);
    HAL_GPIO_WritePin(PRECHARGE_GPIO_Port,PRECHARGE_Pin,GPIO_PIN_SET); // 1 - OFF
    HAL_Delay(PRECHARGE_TURN_OFF_ON_TIME);
    HAL_GPIO_WritePin(ACTIVE_DISCHARGE_GPIO_Port,ACTIVE_DISCHARGE_Pin,GPIO_PIN_SET);
    HAL_Delay(ACTIVE_DISCHARGE_DELAY);
    HAL_GPIO_WritePin(ACTIVE_DISCHARGE_GPIO_Port,ACTIVE_DISCHARGE_Pin,GPIO_PIN_RESET);
    break;
    
  case AUX_ERROR: //Turn off CONTACTOR, Precharge and Active Discharge
    pcv.allChecksDone = 1;
    HAL_GPIO_WritePin(CONTACTOR_COIL_GPIO_Port,CONTACTOR_COIL_Pin,GPIO_PIN_SET); // 1 - OFF
    HAL_Delay(CONTACTOR_TURN_OFF_ON_TIME);
    HAL_GPIO_WritePin(PRECHARGE_GPIO_Port,PRECHARGE_Pin,GPIO_PIN_SET); // 1 - OFF
    HAL_Delay(PRECHARGE_TURN_OFF_ON_TIME);
    HAL_GPIO_WritePin(ACTIVE_DISCHARGE_GPIO_Port,ACTIVE_DISCHARGE_Pin,GPIO_PIN_SET);
    HAL_Delay(ACTIVE_DISCHARGE_DELAY);
    HAL_GPIO_WritePin(ACTIVE_DISCHARGE_GPIO_Port,ACTIVE_DISCHARGE_Pin,GPIO_PIN_RESET);
    break;
    
  case AUX_COMPLETE:
    //MC_AcknowledgeFaultMotor1();
    HAL_GPIO_WritePin(PRECHARGE_GPIO_Port,PRECHARGE_Pin,GPIO_PIN_SET); // 1 - OFF
    pcv.allChecksDone = 1;
    break;    
  }
}

//unused
void ActiveDischarge(void)
{
  HAL_GPIO_WritePin(CONTACTOR_COIL_GPIO_Port,CONTACTOR_COIL_Pin,GPIO_PIN_SET); // 1 - OFF
  HAL_Delay(CONTACTOR_TURN_OFF_ON_TIME);
  HAL_GPIO_WritePin(PRECHARGE_GPIO_Port,PRECHARGE_Pin,GPIO_PIN_SET); // 1 - OFF
  HAL_Delay(PRECHARGE_TURN_OFF_ON_TIME);
  HAL_GPIO_WritePin(ACTIVE_DISCHARGE_GPIO_Port,ACTIVE_DISCHARGE_Pin,GPIO_PIN_SET);
  HAL_Delay(ACTIVE_DISCHARGE_DELAY);
  HAL_GPIO_WritePin(ACTIVE_DISCHARGE_GPIO_Port,ACTIVE_DISCHARGE_Pin,GPIO_PIN_RESET);
}
