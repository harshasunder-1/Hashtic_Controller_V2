#include "Configuration.h"

int16_t getMotorIndexFromMotorID(uint8_t motorID){
  switch (motorID){
  case MOTOR1:
    return MOTOR1_ZERO_POS;
  case MOTOR2:
    return MOTOR2_ZERO_POS;
  case MOTOR3:
    return MOTOR3_ZERO_POS;
  case MOTOR4:
    return MOTOR4_ZERO_POS;
  case MOTOR5:
    return MOTOR5_ZERO_POS;
  case MOTOR6:
    return MOTOR6_ZERO_POS;
  case MOTOR7:
    return MOTOR7_ZERO_POS;
  case MOTOR8:
    return MOTOR8_ZERO_POS;
  case MOTOR9:
    return MOTOR9_ZERO_POS;
  case MOTOR10:
    return MOTOR10_ZERO_POS;
  default: 
    return -1;

  }
}
    