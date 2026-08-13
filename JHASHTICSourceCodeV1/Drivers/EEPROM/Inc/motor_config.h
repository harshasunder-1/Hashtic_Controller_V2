#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "main.h"
#include "eeprom_emul.h"
#include "Configuration.h"

/************* EEPROM Addresses *************/
#define EE_MOTOR_ID_ADDR          1
#define EE_MOTOR_ID_COPY_ADDR     2

#define EE_SIGN_ADDR              3
#define EE_SIGN_COPY_ADDR         4

#define EE_POSITION_ADDR          5
#define EE_POSITION_COPY_ADDR     6
/************* Function Prototypes *************/

#define EEPROM_VALUES_OK 0
#define EEPROM_IDS_OUT_OF_RANGE 1
#define EEPROM_IDS_NOT_MATCHING 2
#define EEPROM_CWSIGN_OUT_OF_RANGE 3
#define EEPROM_CWSIGNS_NOT_MATCHING 4
#define EEPROM_SIDES_NOT_MATCHING 5
#define EEPROM_SIDES_OUT_OF_RANGE 6

typedef struct {
  uint8_t motorID;
  uint8_t motorIDCopy;
  int8_t signforCW;
  int8_t signforCWCopy;
  uint8_t positionInPod;
  uint8_t positionInPodCopy;
  uint8_t eeReadStatus;
}EEConfig;

void MotorConfig_Init(void);
EE_Status MotorConfig_Write(uint32_t motorID,int32_t sign,int32_t position);
EE_Status MotorConfig_Read(EEConfig *ee);
uint8_t CheckEEConfigValues(EEConfig *ee);

#endif