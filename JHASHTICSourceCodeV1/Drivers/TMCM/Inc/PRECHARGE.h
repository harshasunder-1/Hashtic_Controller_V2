#ifndef PRECHARGE_H_
#define PRECHARGE_H_

#define ACTIVE_DISCHARGE_DELAY 300
#define CONTACTOR_TURN_OFF_ON_TIME 300
#define PRECHARGE_TURN_OFF_ON_TIME 300

typedef enum {
  PRECHARGE_OFF = 0,             
  PRECHARGE_START = 1,     
  PRECHARGE_WAITING = 2,   
  PRECHARGE_CHECK_VOLTAGE = 3,
  PRECHARGE_COMPLETE = 4,
  AUX_CHECK_PIN = 5,
  AUX_ERROR = 6,
  AUX_COMPLETE = 7,
  PRECHARGE_OFF_COMMAND = 8,
  PRECHARGE_ERROR = -1             
} Precharge_State_t;

typedef struct {
  uint8_t allChecksDone;
  uint8_t startCount;
  uint16_t tim17_1sec_precharge;
  uint8_t auxContactorSignal;
  Precharge_State_t Precharge_Stage;
} PrechargeVariables;

extern PrechargeVariables pcv;

void PrechargingLogic(void);
void ActiveDischarge(void);
#endif