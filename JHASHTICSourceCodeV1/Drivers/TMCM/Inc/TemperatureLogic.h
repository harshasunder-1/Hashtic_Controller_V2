//TemperatureLogic.h


#ifndef TEMPLOGIC_H_
#define TEMPLOGIC_H_


#include "stdint.h"
#include "regular_conversion_manager.h"

#define MOTOR_TEMP 1
#define MOSFET_TEMP 2
#define TEMP_LPF_ALPHA 0.9

typedef struct{
  uint8_t motorTempC;
  uint8_t prevMotorTempC;
  uint8_t mosfetTempC;
  uint8_t prevMosfetTempC;
  uint16_t motorADC;
  uint16_t mosfetADC;
  uint8_t motorADC_readOK;
  uint8_t mosfetADC_readOK;
  uint8_t motorHandler;
  uint8_t mosfetHandler;
}Temp;


uint8_t  InitializeADC(RegConv_t *adcHandler,ADC_TypeDef *adc,uint8_t channelNo,uint32_t sampleTime);
int getCelciusValue(int RawADCValue,int ofWhat);
void ReadTemperatureADC(Temp *t,uint8_t ofWhat);
void ProcessTemperatureADCs(Temp *t);
extern Temp t;



#endif