
#ifndef CONFIG_H_
#define CONFIG_H_

#include "stdint.h"

//direction of motor in axle when sitting inside pod and looking out forward.
//For the left side motor, moving anticlockwise, makes pod go forward
//for tight side motor, moving clockwise makes pod go forward.
#define LEFT_SIDE 1
#define RIGHT_SIDE 2

#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3
#define MOTOR4 4
#define MOTOR5 5
#define MOTOR6 6
#define MOTOR7 7
#define MOTOR8 8
#define MOTOR9 9

#define MOTOR1_ZERO_POS 2586
#define MOTOR2_ZERO_POS 3721
#define MOTOR3_ZERO_POS 11863
#define MOTOR4_ZERO_POS 5228
#define MOTOR5_ZERO_POS 8775
#define MOTOR6_ZERO_POS 12600
#define MOTOR7_ZERO_POS 4580
#define MOTOR8_ZERO_POS 12089
#define MOTOR9_ZERO_POS 5492
#define MOTOR10_ZERO_POS 0

typedef struct {
  uint8_t motorID;
  uint16_t zeroPos;
  uint8_t positionInPod;
  int8_t signForCWRotation;
  uint8_t EncoderABIConfigLoaded;
  uint8_t EncoderZeroPosLoaded;
}Config;

extern Config c;
int16_t getMotorIndexFromMotorID(uint8_t motorID);














#endif
