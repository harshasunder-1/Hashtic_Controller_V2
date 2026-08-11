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

/************* Function Prototypes *************/
void MotorConfig_Init(void);

EE_Status MotorConfig_Write(uint32_t motorID, int32_t sign);

EE_Status MotorConfig_Read(Config *cfg);

#endif