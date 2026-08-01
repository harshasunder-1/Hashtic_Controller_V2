#include "ControlFns.h"
#include "mc_api.h"
#include "Ramp.h"

extern float k;
extern uint8_t CircleLimitationState;
extern RampingOperation RampOp;

//lookup table for continuous control to work from 
//0,0,0,0,0,0,0,0,1,1,1,2,2,3,3,4,4,5,5,6,7,8,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,25,26,27,29,30,31,33,34,
const uint16_t CONTINOUS_RPMS[701]={30,30,30,30,30,30,30,30,30,30,31,31,31,31,31,31,31,32,32,32,32,32,32,33,33,33,33,33,34,34,34,34,34,35,35,
35,37,38,40,41,43,45,46,48,50,51,53,55,56,58,60,62,64,65,67,69,71,73,75,77,79,81,83,85,87,89,91,93,95,97,99,102,104,106,108,110,113,
115,117,119,122,124,126,129,131,133,136,138,140,143,145,147,150,152,155,157,160,162,164,167,169,172,174,177,179,182,184,187,190,192,
195,197,200,202,205,207,210,213,215,218,220,223,226,228,231,233,236,239,241,244,247,249,252,254,257,260,262,265,267,270,273,275,278,
281,283,286,288,291,294,296,299,302,304,307,309,312,315,317,320,322,325,327,330,333,335,338,340,343,345,348,350,353,355,358,360,363,
365,368,370,373,375,378,380,383,385,387,390,392,395,397,399,402,404,407,409,411,414,416,418,420,423,425,427,430,432,434,436,438,441,
443,445,447,449,452,454,456,458,460,462,464,466,468,470,472,474,476,478,480,482,484,486,488,490,492,494,495,497,499,501,503,504,506,
508,510,511,513,515,517,518,520,521,523,525,526,528,529,531,532,534,535,537,538,540,541,543,544,545,547,548,549,551,552,553,554,556,
557,558,559,560,562,563,564,565,566,567,568,569,570,571,572,573,574,575,576,577,577,578,579,580,581,581,582,583,584,584,585,586,586,
587,587,588,589,589,590,590,591,591,591,592,592,593,593,593,594,594,594,594,595,595,595,595,596,596,596,596,596,596,596,596,596,596,
596,596,596,596,596,596,596,595,595,595,595,594,594,594,594,593,593,593,592,592,591,591,591,590,590,589,589,588,587,587,586,586,585,
584,584,583,582,581,581,580,579,578,577,577,576,575,574,573,572,571,570,569,568,567,566,565,564,563,562,560,559,558,557,556,554,553,
552,551,549,548,547,545,544,543,541,540,538,537,535,534,532,531,529,528,526,525,523,521,520,518,517,515,513,511,510,508,506,504,503,
501,499,497,495,494,492,490,488,486,484,482,480,478,476,474,472,470,468,466,464,462,460,458,456,454,452,449,447,445,443,441,438,436,
434,432,430,427,425,423,420,418,416,414,411,409,407,404,402,399,397,395,392,390,387,385,383,380,378,375,373,370,368,365,363,360,358,
355,353,350,348,345,343,340,338,335,333,330,327,325,322,320,317,315,312,309,307,304,302,299,296,294,291,288,286,283,281,278,275,273,
270,267,265,262,260,257,254,252,249,247,244,241,239,236,233,231,228,226,223,220,218,215,213,210,207,205,202,200,197,195,192,190,187,
184,182,179,177,174,172,169,167,164,162,160,157,155,152,150,147,145,143,140,138,136,133,131,129,126,124,122,119,117,115,113,110,108,
106,104,102,99,97,95,93,91,89,87,85,83,81,79,77,75,73,71,69,67,65,64,62,60,58,56,55,53,51,50,48,46,45,43,41,40,38,37,35,34,33,31,30,
29,27,26,25,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,8,7,6,5,5,4,4,3,3,2,2,1,1,1,0,0,0,0,0,0,0,0};


//MotorControl Functions
void applyRegen(int currentDirection){
  if(currentDirection == 1){  //change this later..make it cleaner, direction from outside?
      MC_ProgramTorqueRampMotor1(-ONE_SHOT_REGEN_FIXED_IQ, 1000 ); //for pod9, +ve
    }else{
      MC_ProgramTorqueRampMotor1(ONE_SHOT_REGEN_FIXED_IQ, 1000 );
    }
}

void laptopRun(laptopControl *lc,systemState *ss){
  uint16_t absTarget = 0;
  
  if (lc->targetRPM < 0){
    absTarget = -lc->targetRPM;
  }else{
    absTarget = lc->targetRPM ; 
  }
   
  uint8_t speedRPM_OK = 0,rampUpTime_OK = 0,IsPodStopped=0,isMotorEncoderNOK=0;
   
  if ((absTarget > 100) && (absTarget < 2000)){
    speedRPM_OK = 1;
  }
    
  float rampTemp = (float)absTarget /lc->rampUpTime;
  if(rampTemp <=0.15f && rampTemp > 0.015f){ //min ramptime = 2s , max ramptime = 20s
    rampUpTime_OK = 1;
  }
    
  if(ss->currentAbsRpm < 5){IsPodStopped = 1;} // only allow start if pod is stopped.
  if(ss->CustomFaults == ENCODER_INDEX_LOAD_FAIL){isMotorEncoderNOK = 1;}
  if(ss->CustomFaults == BAD_MOTOR_INDEX){isMotorEncoderNOK = 1;}
  
  if (speedRPM_OK==1 && rampUpTime_OK==1 && IsPodStopped==1 && isMotorEncoderNOK==0){
      //reset circle limitation
      k = 10;CircleLimitationState = 0;
      
      ss->targetRPM = lc->targetRPM; 
      if (ss->targetRPM > 0){ss->direction = 1;}else{ss->direction = -1;}
      RampOp.steadyStateTime_s = lc->steadyStateTime*0.001; 
      RampOp.rampUpTime = lc->rampUpTime;
      
      MC_ProgramSpeedRampMotor1(ss->targetRPM/6, lc->rampUpTime);
      MC_StartMotor1();
      StartRamps();
    } 
} 

void laptopStop(laptopControl *lc,systemState *ss){
    ss->targetRPM=0;
    MC_StopMotor1();
    RampTurnOff();
    MC_Clear_IqdrefMotor1();
}

void laptopChangeRPM(laptopControl *lc){
    MC_ProgramSpeedRampMotor1(lc->targetRPM/6, lc->changeRPMTime);
}


void laptopCC_on(laptopContinuousControl *lcc){
    lcc->idx = 0;
    if (lcc->direction == 0){
      lcc->direction = 1;
    }
    lcc->stopIdx = 700;
    MC_ProgramSpeedRampMotor1(0,10);
    MC_StartMotor1();
    lcc->timerOn_bool=1;
}

void laptopCC_increment(laptopContinuousControl *lcc){
  lcc->idx += 1;
  if (lcc->idx > lcc->stopIdx){
    MC_StopMotor1();
    MC_Clear_IqdrefMotor1();
    lcc->timerOn_bool = 0;
  }else{
    lcc->target = CONTINOUS_RPMS[lcc->idx] * lcc->direction ;
   
    MC_ProgramSpeedRampMotor1(lcc->target/6,10);
  }
}

void laptopCC_stop(laptopContinuousControl *lcc){
  MC_StopMotor1();
  MC_Clear_IqdrefMotor1();
  lcc->timerOn_bool = 0;
  lcc->idx = 0;
  lcc->target = 0;
}
  
