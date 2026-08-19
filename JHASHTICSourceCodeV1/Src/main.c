/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
* All rights reserved.</center></h2>
*
* This software component is licensed by ST under Ultimate Liberty license
* SLA0044, the "License"; You may not use this file except in compliance with
* the License. You may obtain a copy of the License at:
*                             www.st.com/SLA0044
*
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motorcontrol.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
#include "AS5x47P.h"
#include "EncoderFns.h"
#include "FDCAN.h"
#include "TemperatureLogic.h"
#include "StateMachine.h"
#include "Configuration.h"
#include "ControlFns.h"
#include "EncFaults.h"
#include "States.h"
#include "StartSequence.h"
#include "motor_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define ON 1
#define OFF 0
#define MAX_TEMP_MOTOR 100
#define MAX_TEMP_MOSFET 100
#define MIN_TEMP_MOTOR 60
#define MIN_TEMP_MOSFET 60
#define CAPPING_TEMP_MOTOR 80
#define CAPPING_TEMP_MOSFET 80
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

Temp t;
CAN_SS_Input canSSIp;
CAN_Continuous_Input canContIp;
continousControlTimer ccT;
systemState ss;
Config c;
laptopControl lc;
laptopContinuousControl lcc;
EncFaults encFlts;
startSeq ssq;
brakeCtrl b;
EEConfig eecfg;


extern FOCVars_t FOCVars[1];
extern int16_t hTargetSpeedUserDefined;
extern SpeednTorqCtrl_Handle_t *pSTC[NBR_OF_MOTORS];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CORDIC_HandleTypeDef hcordic;

DAC_HandleTypeDef hdac1;

FDCAN_HandleTypeDef hfdcan2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CORDIC_Init(void);
static void MX_DAC1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
RegConv_t Motor_ADC,MOSFET_ADC; // These ADC handlers are defined here to make it easy to see what ADCs we are reading.

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  
uint8_t sendData, PCOn, activeDischarge;
FDCAN_RxHeaderTypeDef   RxHeader;
uint8_t RxData[8];

extern float k;
extern uint8_t CircleLimitationState;

uint8_t cc_turnOff = 0,triggerFault =0,nvicReset=0;
int16_t deltaRPM = 0, rpmError = 0;
FDCAN_RxHeaderTypeDef   RxHeader;
uint8_t RxData[8];
uint8_t CircleLimitationState;

uint8_t cc_stopMsg_oneTime=0;

uint8_t st_calib_on = 0,st_calib_off=0,st_increment=0,st_counter=0;
uint16_t step_current = 0,st_encVal_start=0,st_encVal=0,stEnc_absDeltaVal=0;
int16_t stEnc_deltaVal=0;

int16_t spiRaw = 0 ;
uint8_t triggerSPIAngleReading = 0;
float ABI_elAngle = 0,spi_elAngle=0,deltaAngles=0,encFault_deltaAngle=0,encFault_spiAngle=0;
uint8_t encFaultCounter = 0,testDirError = 0,resetCustomFaults=0,spi_FailCount = 0;
uint8_t ssqErrorState = 0,resetStartSeq=0;
State_t start_state;
extern MCI_Handle_t * pMCI[NBR_OF_MOTORS];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  
  /* USER CODE END 1 */
  

  /* MCU Configuration---3-----------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  
  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CORDIC_Init();
  MX_DAC1_Init();
  MX_FDCAN2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_MotorControl_Init();
  MX_TIM7_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_GPIO_WritePin(PRECHARGE_GPIO_Port,PRECHARGE_Pin,GPIO_PIN_RESET);
  HAL_Delay(200);
  MC_AcknowledgeFaultMotor1();
  HAL_Delay(200);
  
  //set up interrupts for TIM2 encoder err detection
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_DIR);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_IERR);
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_TERR);
  
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim17); // Timer 17 - Configured as interrupt -  100ms 
  
  FDCAN_TxInit(); 
  FDCAN_RxFilterInit();  
  HAL_Delay(1000); //wait for some time so that voltage at Encoder is also stabilized.
  
  t.motorHandler=InitializeADC(&Motor_ADC,ADC1,2,ADC_SAMPLETIME_6CYCLES_5);
  t.mosfetHandler=InitializeADC(&MOSFET_ADC,ADC1,3,ADC_SAMPLETIME_6CYCLES_5);
  
  ss.CustomFaults = NO_FAULTS;
  ss.cc_state = CC_IDLE;
   
  /********************* motor id verification from EEPROM ***************************/
  //Load Motor ID,sign for CW and position in POD from Eeprom
  //sign for CW = -1 means, motor shaft rotates counter clockwise when Iq is +ve
  //check if within Range, both readings are same.
  /*  
  POD 5,motor10,signForCW=-1,LEFT_SIDE  /  motor6,signForCW= -1,RIGHT_SIDE
  POD 2,motor9, signForCW= 1,RIGHT_SIDE
  POD 4,motor5,signForCW= 1,LEFT_SIDE

  POD 3,motor3,signForCW= 1,RIGHT_SIDE


  */
  
  MotorConfig_Init();
  
  //Uncomment only once when programming a new inverter motor pair 
  //MotorConfig_Write(3, 1, RIGHT_SIDE);  // (motor ID, signforCW,positioninPod,
  
  if (MotorConfig_Read(&eecfg) == EE_OK){  //TODO : use uint8 or 16 bit read/write functions inside motorConfig
    if (CheckEEConfigValues(&eecfg) == EEPROM_VALUES_OK){
      c.motorID = eecfg.motorID;
      c.signForCWRotation = eecfg.signforCW;
      c.positionInPod = eecfg.positionInPod;
    }else{
      ss.CustomFaults = BAD_EEPROM_VALUES;
      ss.cc_state = CC_ERROR;
    }
  }else{
    ss.CustomFaults = EEPROM_READING_ERR;
    ss.cc_state = CC_ERROR;
  }
  
  /********************************************/
  
  if (ss.cc_state == CC_IDLE){ // if no errors use values read to setup inverter
    
      int16_t response = getMotorIndexFromMotorID(c.motorID);
      if (response != -1){
          c.zeroPos = response;
          c.EncoderZeroPosLoaded = updateEncoderZeroPosition(c.zeroPos);
          c.EncoderABIConfigLoaded = setupMotorEncoder_inABI_Mode(); 
          if (c.EncoderABIConfigLoaded == 0){ // this fault only comes during starting.
              ss.CustomFaults = ENCODER_INDEX_LOAD_FAIL; //If this fails, we dont let the machine to start
              ss.cc_state = CC_ERROR;
            }
      }else{
          ss.CustomFaults = BAD_MOTOR_INDEX;
          ss.cc_state = CC_ERROR;
      }
   }
    
  /*Now setup all state variables */
  if (ss.cc_state == CC_IDLE){
    InitializeEncFaults(&encFlts);
    initializeStartSeqParams(&ssq);
    
    CircleLimitationState = 0;
    
    ss.runType = NO_RUN;
    ss.cc_state = CC_IDLE;
    ss.cc_ramp = CC_RAMPOFF;

    ccT.PCM_timer_thresh = 6;
    cc_stopMsg_oneTime = 0;
    
    updateTMCMState(&ss); //voltage
        
   //Wait till State goes to IDLE. The turn the motor on and off , so that the ABI and SPI sensors synchronize.
    start_state = MC_GetSTMStateMotor1();
    if (start_state == IDLE){
      MC_ProgramSpeedRampMotor1(0, 300 );
      MC_StartMotor1();
      HAL_Delay(1000);
      TMCM_SpeedLoop_TurnOff();
    }else{
      //if state is not idle here, then some MCSDK fault. TODO: Read that fault and handle it
      ss.CustomFaults = START_IDLE_NOT_REACHED;
      ss.cc_state = CC_ERROR;
    }
  }
  
  //if any of the errors above have fired, do not allow the inverter to start. Error msgs to PCM will keep going from interrupt.
  if (ss.cc_state == CC_ERROR){
    while(1){
      //Wait for ever and keep sending error msg
      ss.neverStarting++;
      HAL_Delay(1000);
    }
  }
 
  // TODO: we also want to check temperature here and see if it is available, and also if its within limits when starting.
  // TODO: How do we check Encoder health during running?-DONE
  // TODO : check voltage, if voltage not there, throw error and send codes. Need to handle MCSDK errors properly, only custom errors 
  // are being checked 
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    ReadTemperatureADC(&t,MOTOR_TEMP); // has a delay inside so cant put in interrupt    
    ReadTemperatureADC(&t,MOSFET_TEMP);
    ProcessTemperatureADCs(&t);
    updateTMCMState(&ss); //voltage
    
    if(resetCustomFaults){
      ss.cc_state =CC_IDLE;
      ss.CustomFaults = NO_FAULTS;
      resetCustomFaults = 0;
    }
    

    //----------------------------------------------
    //Happens for all types of run all the time.
    //Stop if SPI and ABI dont have the same reading.
    if (triggerSPIAngleReading){
      spiRaw = ENC_getRawReadingFromSPI();
      if (spiRaw==-1){ 
        AS5047_checkEncoderError(); // TODO: will clear the error on the chip -> later keep counts of which error and how many times.
        spi_FailCount ++;
        if (spi_FailCount >= 3){
          ss.CustomFaults = ENCODER_SPI_READ_FAIL;
          ss.cc_state = CC_ERROR;
        }
      }else{
        spi_FailCount = 0;
        ABI_elAngle = (float)(htim2.Instance->CNT%409)/409.6f *360.0f;
        spi_elAngle = (spiRaw %3276)/3276.0f*360.0f;
        deltaAngles = (ABI_elAngle - spi_elAngle);
        if (fabs(deltaAngles) > 5.0f){
          encFaultCounter++;
          if (encFaultCounter >=3){
            encFault_spiAngle = spi_elAngle;
            encFault_deltaAngle = deltaAngles;
            ss.CustomFaults = ENCODER_ABI_SPI_DELTA;
            ss.cc_state = CC_ERROR;
          }
        }else{
          encFaultCounter = 0;
        }
      }
      triggerSPIAngleReading =  0;
    }
    
    //-----------------------------------------------
    
    //manual continous running from an array
    if (lcc.start){
      laptopCC_on(&lcc);
      ss.podDirection = lcc.direction;
      ss.motorDirection = ss.podDirection;
      ss.cc_state = CC_RUNNING_CL;
      ss.cc_ramp = CC_RAMPUP;
      ss.runType=LCC;
      
      ss.travelledDist=0;
      
      ss.CL_DeltaRPMThreshold = Calculate_CLDeltaRPMThreshold(600);
      
      DisableEncoderFltChking(&encFlts);
      ResetEncFaults(&encFlts);
      EnableEncoderFltChking(&encFlts);
      
      lcc.start = 0;
    }
    
    
    if (lcc.increment_signal){
      laptopCC_increment(&lcc,&ss); 
      ss.indexID = lcc.idx;
      if (abs(lcc.target) < ss.targetRPM){
        ss.cc_ramp = CC_RAMPDOWN;
      }
      if ((ss.cc_ramp == CC_RAMPDOWN) && (abs(lcc.target) < 30)){
        lcc.stop = 1;
      }
      ss.targetRPM = lcc.target;
      lcc.increment_signal=0;
    }
    
    if(lcc.stop){
      TMCM_SpeedLoop_TurnOff(); 
      laptopCC_stop(&lcc);
      ss.targetRPM =  0;
      ss.indexID  = 0;
      ss.cc_state = CC_IDLE;
      ss.cc_ramp = CC_RAMPOFF;
      ss.runType=NO_RUN;
      lcc.stop = 0;
    }
    
    /*---end of lcc code -------*/
    
    //BRAKING FOR CC
    if (ss.runType == CC){
      if (ss.travelledDist >= ss.brakeDistance){ //should not be more than 60
        ss.engageBrake = 1;
      }
    }
    // hitting break once
    if (ss.engageBrake == 1){
        if (b.brakeCounter == 0){
           TMCM_SpeedLoop_TurnOff();
           FDCAN_SendControlledBrakeMsg();     
           ss.brakeState++;
           b.brakeCounter++;
           ss.engageBrake = 0;
           ss.cc_ramp = CC_RAMPOFF;
           ss.cc_state = CC_ERROR; // has to be error to stop it looking at the continous can msgs
           FDCAN_SendPCMAckMsg(1); 
        }
}
    
  /*   hit break twice to test
    if (ss.engageBrake == 1){
        if (b.brakeCounter == 0){
           TMCM_SpeedLoop_TurnOff();
           FDCAN_SendControlledBrakeMsg(); 
           b.brakeTime1 = HAL_GetTick(); 
           ss.brakeState++;
           b.brakeCounter++;
        }
        
       if (b.brakeCounter == 1){
          if(HAL_GetTick()-b.brakeTime1>=200){
             FDCAN_SendControlledBrakeMsg();
             ss.engageBrake = 0;
             ss.cc_ramp = CC_RAMPOFF;
             ss.cc_state = CC_ERROR; // has to be error to stop it looking at the continous can msgs
             FDCAN_SendPCMAckMsg(1); 
            }
        }
   } // closes engage brake == 1
 */

    // NEEDS TO BE REMOVED IF WE RE TRYING TO RUN LONGER DISTANCES THAN X dist
    if (ss.cc_state == CC_RUNNING_CL || ss.cc_state == CC_RUNNING_OL){
      if (ss.travelledDist >= 200){ //hardocded 
       TMCM_SpeedLoop_TurnOff(); 
       ss.cc_ramp = CC_RAMPOFF;
       ss.cc_state = CC_ERROR; // has to be error to stop it looking at the continous can msgs
       
       FDCAN_SendSlamBrakeMsg();
       ss.brakeState++;
       b.brakeCounter++;
       FDCAN_SendPCMAckMsg(1);
      }
    }
    
      //Starting Seq
    if (ssq.PCM_startCommand == 1){
        ssqErrorState = ExecStartSeq(&ssq,&ss);
        if (ssqErrorState != NO_ERROR){
          ssq.globalStartSeqTimer = 0;
          ss.CustomFaults = CC_START_SEQ_FAIL;
          ss.cc_state = CC_ERROR;
        }
    }
    
    if (resetStartSeq){
       resetStartSeqParams( &ssq);
       resetStartSeqErrorState( &ssq);
       resetStartSeq = 0;
    } 
    
    if ((ss.cc_state == CC_RUNNING_CL) && (ssq.currentState == TRANSITION_DONE)){ 
      
      //RPM condition 1
      if (abs(hTargetSpeedUserDefined) > 100){
        deltaRPM = abs(hTargetSpeedUserDefined) - ss.currentAbsRpm;
        if (deltaRPM < 0){deltaRPM = -deltaRPM;}
        if(deltaRPM > ss.CL_DeltaRPMThreshold){
            ss.cc_state = CC_ERROR;
            ss.CustomFaults = CC_CLOSED_LOOP_FAIL;
        }
      }
                                
       //if we re giving large current and not moving , stop.
       // DOESNT WORK WITH LCC
        uint16_t absIqRef = abs(FOCVars[0].Iqdref.q);
        if ((absIqRef > 4000) && (ss.currentAbsRpm < 5)){  //requires about 1300 to start.even with metal bed if we re giving so much and not moving,stop
          ss.cc_state = CC_ERROR;
          ss.CustomFaults = CC_OVC_STALL;
        }
        
       /* while motoring, if you detect motion in the opp direction , turn off. SHOULD WE keep this for REGEN also? In case regen doesnt stop youll go in reverse*/
       startEncoderChecking(&encFlts,ss.currentAbsRpm);
       stopEncoderChecking(&encFlts,abs(ss.targetRPM));
       if (encFlts.encoderCheckingOn){
         if (encFlts.EncoderErrFlag == 1){
            ss.cc_state = CC_ERROR;
           if (encFlts.directionErrFlag){
               ss.CustomFaults = ENCODER_DIR_ERROR;
           }
           else if (encFlts.indexErrFlag){
             ss.CustomFaults = ENCODER_INDEX_ERROR;
           }
           else if (encFlts.transitionErrFlag){
             ss.CustomFaults = ENCODER_TRANSITION_ERROR;
           }
           else{}
        }
       }    
    } //closes RUNNING_CL
       
    
    //turn off during Regen. Whenever regen, state is being made RUNNING_OL. TODO: think if this is best way
    if (ss.cc_state == CC_RUNNING_OL){
      if (ss.currentAbsRpm < 100){
        cc_turnOff = 1;
      }
    }
        
    if (ss.cc_state == CC_ERROR){ // turn off. running this continously prevents any other command from restarting the motor.Precharge command from PCM resets this.
        TMCM_SpeedLoop_TurnOff(); 
       // ss.engageBrake =1; //do mech braking here also
        ccT.timerOnBool = 0;
        ss.cc_ramp = CC_RAMPOFF;
        ss.runType=NO_RUN;
        hTargetSpeedUserDefined=0;
        DisableEncoderFltChking(&encFlts);
        ssq.PCM_startCommand = 0;
    }
          
    if (cc_turnOff){ // used everywhere to turn off correctly
      TMCM_SpeedLoop_TurnOff(); 
      ss.cc_ramp = CC_RAMPOFF;
      ss.cc_state = CC_FINISH;
      ss.runType=NO_RUN;
      hTargetSpeedUserDefined=0;
      DisableEncoderFltChking(&encFlts);
      FDCAN_SendPCMAckMsg(1);
      cc_turnOff = 0;
    }
    
    if(nvicReset){
       HAL_NVIC_SystemReset();
    }
    
   //  HAL_GPIO_WritePin(FAULT_1_GPIO_Port,FAULT_1_Pin,GPIO_PIN_SET);
   //  HAL_GPIO_WritePin(FAULT_2_GPIO_Port,FAULT_2_Pin,GPIO_PIN_SET);
   
    
  }// closes while
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_8) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM1_BRK_TIM15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  /* TIM1_UP_TIM16_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 3, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  
  
  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  
  
  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  
  
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */
  
  
  /* USER CODE END ADC2_Init 0 */

  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */
  
  
  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Injected Channel 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  
  
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */
  
  
  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */
  
  
  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */
  
  
  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */
  
  
  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */
  
  
  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization 
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config 
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  
  
  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */
  
  
  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */
  
  
  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = ENABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 17;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 2;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */
  
  
  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  
  
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  
  
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  
  
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  
  
  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  
  
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = ((PWM_PERIOD_CYCLES) / 2);
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = (REP_COUNTER);
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
  sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
  sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_LOW;
  if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK2, &sBreakInputConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;
  sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS) / 2);
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_ENABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  
  
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  
  
  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  
  
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = M1_PULSE_NBR;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = M1_ENC_IC_FILTER;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = M1_ENC_IC_FILTER;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  
  
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */
  
  
  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */
  
  
  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1699;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
  
  
  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */
  
  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */
  
  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1700;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  
  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */
  
  
  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */
  
  
  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 1700;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 9999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */
  
  
  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  
  
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  
  
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  
  
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PRECHARGE_GPIO_Port, PRECHARGE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ACTIVE_DISCHARGE_Pin|FAULT_LED_Pin|GREEN_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CONTACTOR_COIL_GPIO_Port, CONTACTOR_COIL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, FAULT_1_Pin|FAULT_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GREEN_LED2_Pin|SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PRECHARGE_Pin */
  GPIO_InitStruct.Pin = PRECHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PRECHARGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ACTIVE_DISCHARGE_Pin FAULT_LED_Pin GREEN_LED1_Pin */
  GPIO_InitStruct.Pin = ACTIVE_DISCHARGE_Pin|FAULT_LED_Pin|GREEN_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : CONTACTOR_AUX_Pin */
  GPIO_InitStruct.Pin = CONTACTOR_AUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CONTACTOR_AUX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CONTACTOR_COIL_Pin FAULT_1_Pin FAULT_2_Pin */
  GPIO_InitStruct.Pin = CONTACTOR_COIL_Pin|FAULT_1_Pin|FAULT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_LED2_Pin */
  GPIO_InitStruct.Pin = GREEN_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Start_Stop_Pin */
  GPIO_InitStruct.Pin = Start_Stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_Stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INDEX_Pin */
  GPIO_InitStruct.Pin = INDEX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INDEX_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

// Modified function that replaces the weak function in MCSDK. Used to set the correct direction for the torque
// when the motor is wound opposite
void FOC_CalcCurrRef(uint8_t bMotor)
{
  if(FOCVars[bMotor].bDriveInput == INTERNAL)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    
    // 2. Convert to Electrical Iq, flipping the sign if reverse wound
    if (c.signForCWRotation == -1) {
        FOCVars[bMotor].Iqdref.q = -(FOCVars[bMotor].hTeref);
    } else {
        FOCVars[bMotor].Iqdref.q = FOCVars[bMotor].hTeref;
    }
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
      FDCAN_parseForMotor();
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  }
}
uint16_t _100ms_counter;
uint8_t stopCAN = 0;
uint8_t badTimer=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // This callback is automatically called by the HAL on the Intrupt Event when timer overflows
  if(htim->Instance == TIM17) /** 100ms timer **/
  {    
    if(ccT.timerOnBool){ //continous control timer. if we dont get a reading for OVF thresh, turn off.
      ccT.PCM_timer ++;
      if (ccT.PCM_timer > ccT.PCM_timer_thresh){
        badTimer = ccT.PCM_timer;
        ss.CustomFaults = PCM_CANTIMER_OVF;
        ss.cc_state = CC_ERROR;
        ccT.timerOnBool = 0;
      }
    }
    
    _100ms_counter++;
    if(lcc.timerOn_bool){lcc.increment_signal=1;} //For manual continous control 
    if (ssq.PCM_startCommand){ssq.globalStartSeqTimer+=100;}
    triggerSPIAngleReading = 1;
    ss.travelledDist += (ss.currentAbsRpm/1800.0f) * 0.628f; //rpm /((60 * 10) * GR) * pi*D, D= 0.2m
    

    if(_100ms_counter%10 == 0){
      if(st_calib_on){st_increment=1;st_counter++;} // for stiction torque calibration
    }
   
      //KEEP SENDING DATA
      FDCAN_CC_TMCM_sendRunTimeData();
      FDCAN_CC_TMCM_sendStatusData();
      if (ss.cc_state == CC_ERROR){
        FDCAN_TMCM_StopFrame(ss.CustomFaults);
      }
  }
  
}

uint16_t IndexPinCounter;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_5)
  {
    IndexPinCounter++;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  
  
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
  tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
