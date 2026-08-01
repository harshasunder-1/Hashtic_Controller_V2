//Temperature Logic 


#include "TemperatureLogic.h"
#include "main.h"

/** Thermistor and Celcius Look Up Tables **/
int THERMISTOR_LUT_MOTOR[150]={48716,47863,47253,46635,46009,45376,44736,44090,43439,42783,42122,41458,40791,40121,39450,38777,\
  38104,37431,36759,36087,35418,34750,34086,33425,32768,32115,31467,30825,30188,29558,28933,28316,27706,27104,26509,25922,25344,\
    24774,24213,23660,23117,22583,22058,21543,21036,20540,20052,19575,19106,18648,18199,17759,17328,16907,16495,16092,15699,15314,\
      14938,14571,14213,13863,13521,13188,12863,12545,12236,11934,11640,11353,11073,10801,10535,10276,10024,9778,9539,9306,9079,8858,\
        8642,8433,8228,8030,7836,7647,7464,7285,7111,6941,6776,6616,6459,6307,6159,6015,5874,5737,5604,5474,5348,5225,5105,4988,4875,4764,\
          4656,4551,4449,4350,4253,4158,4066,3976,3889,3804,3721,3640,3561,3484,3410,3337,3265,3196,3128,3063,2998,2936,2874,2815,2757,2700,\
            2645,2591,2538,2486,2436,2387,2340,2293,2248,2203,2160,2117,2076,2036,1996,1958,1920,1883};

int THERMISTOR_LUT_MOSFET[150]={53022,52352,51870,51377,50876,50367,49848,49322,48787,48245,47696,47140,46578,46009,45435,44856,44272,43683,43091,42495,\
  41896,41295,40691,40086,39480,38872,38265,37658,37051,36446,35841,35239,34639,34042,33448,32857,32270,31686,31108,\
    30534,29965,29401,28843,28291,27745,27205,26671,26144,25624,25111,24605,24106,23614,23130,22653,22184,21723,21269,\
      20822,20384,19953,19529,19114,18706,18306,17913,17528,17151,16780,16418,16062,15714,15373,15039,14712,14392,14079,\
        13773,13473,13180,12893,12612,12338,12070,11807,11551,11300,11055,10816,10582,10354,10130,9912,9699,9491,\
          9288,9089,8895,8705,8520,8340,8163,7991,7822,7658,7497,7340,7187,7038,6892,6749,6610,6474,6341,6211,6084,5961,\
            5840,5722,5606,5494,5384,5276,5171,5069,4969,4871,4775,4682,4590,4501,4414,4329,4246,4164,4085,4007,3931,3857,\
              3784,3713,3644,3576,3510,3445,3382,3319,3259,3200,3141};

int CELCIUS_LUT[150] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30, \
  31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,\
    55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,\
      85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,\
        112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134, \
          135,136,137,138,139,140,141,142,143,144,145,146,147,148,149};

int getCelciusValue(int RawADCValue,int ofWhat){
  if (ofWhat == MOTOR_TEMP){
    for(int i = 0; i < 150; i++){   
      if(RawADCValue > THERMISTOR_LUT_MOTOR[i]){
        return CELCIUS_LUT[i];
      }
    }
  }
  else if (ofWhat == MOSFET_TEMP){
    for(int i = 0; i < 150; i++){   
      if(RawADCValue > THERMISTOR_LUT_MOSFET[i]){
        return CELCIUS_LUT[i];
      }
    }
  }
  else{
    return -1;
  }
  
  return -1; //never reached
}

uint8_t InitializeADC( RegConv_t *adcHandler,ADC_TypeDef *adc,uint8_t channelNo,uint32_t sampleTime){
  adcHandler->regADC = adc;
  adcHandler->channel = channelNo;
  adcHandler->samplingTime = sampleTime;
  uint8_t out =  RCM_RegisterRegConv(adcHandler);
  return out;
}

//meant to run in the while loop where all the calculation can happen.
void ProcessTemperatureADCs(Temp *t){
  uint16_t tempC = 0;
  if (t->motorADC_readOK == 1){
      tempC = getCelciusValue(t->motorADC,MOTOR_TEMP); 
      t->motorTempC = (uint8_t)(tempC * TEMP_LPF_ALPHA + t->prevMotorTempC * (1.0f - TEMP_LPF_ALPHA));
      t->prevMotorTempC = t->motorTempC;
  }
  
  if (t->mosfetADC_readOK == 1){
      tempC = getCelciusValue(t->mosfetADC,MOSFET_TEMP); 
      t->mosfetTempC = (uint8_t)(tempC * TEMP_LPF_ALPHA + t->prevMosfetTempC * (1.0f - TEMP_LPF_ALPHA));
      t->prevMosfetTempC = t->mosfetTempC;
  }
}

void ReadTemperatureADC(Temp *t,uint8_t ofWhat){
  uint8_t *handle = 0,*readOK = 0;
  uint16_t *adc = 0;
  
  if (ofWhat == MOTOR_TEMP){
    handle = &(t->motorHandler);
    adc = &(t->motorADC);
    readOK = &(t->motorADC_readOK);
  }
  else if (ofWhat == MOSFET_TEMP){
    handle = &(t->mosfetHandler);
    adc = &(t->mosfetADC);
    readOK = &(t->mosfetADC_readOK);
  }
  
  RCM_UserConvState_t adcState = RCM_GetUserConvState();
  if( adcState == RCM_USERCONV_IDLE){
    RCM_RequestUserConv(*handle);
    HAL_Delay(1);
    uint16_t result = RCM_GetUserConv();
    if (result != 0xFFFF){
      *adc = result;
      *readOK =1;
    }
  }       
  
  /*
  if (ofWhat == MOTOR_TEMP){
     t->motorADC_readOK = 0;
     if(RCM_GetUserConvState() == RCM_USERCONV_IDLE){
        RCM_RequestUserConv(t->motorHandler);
        HAL_Delay(1);
        t->motorADC = RCM_GetUserConv();
        if (t->motorADC != 0xFFFF){ //means conversion was not complete
            t->motorADC_readOK = 1;
          }  
        } 
      }
    
  if (ofWhat == MOSFET_TEMP){
      t->mosfetADC_readOK = 0;
      if((RCM_GetUserConvState() == RCM_USERCONV_IDLE)){
        RCM_RequestUserConv(t->mosfetHandler);
        HAL_Delay(1);
        t->mosfetADC = RCM_GetUserConv();
        if (t->mosfetADC != 0xFFFF){ //means conversion was not complete
          t->mosfetADC_readOK = 1;
        }         
      }
    }
    */
}
