/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_pwr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PRECHARGE_Pin GPIO_PIN_4
#define PRECHARGE_GPIO_Port GPIOF
#define M1_CURR_AMPL_U_Pin GPIO_PIN_1
#define M1_CURR_AMPL_U_GPIO_Port GPIOC
#define M1_CURR_AMPL_V_Pin GPIO_PIN_2
#define M1_CURR_AMPL_V_GPIO_Port GPIOC
#define M1_BUS_VOLTAGE_Pin GPIO_PIN_0
#define M1_BUS_VOLTAGE_GPIO_Port GPIOA
#define DBG_DAC_CH1_Pin GPIO_PIN_4
#define DBG_DAC_CH1_GPIO_Port GPIOA
#define DBG_DAC_CH2_Pin GPIO_PIN_5
#define DBG_DAC_CH2_GPIO_Port GPIOA
#define ACTIVE_DISCHARGE_Pin GPIO_PIN_4
#define ACTIVE_DISCHARGE_GPIO_Port GPIOC
#define M1_OCP_Pin GPIO_PIN_14
#define M1_OCP_GPIO_Port GPIOE
#define M1_PWM_WL_Pin GPIO_PIN_15
#define M1_PWM_WL_GPIO_Port GPIOB
#define CONTACTOR_AUX_Pin GPIO_PIN_11
#define CONTACTOR_AUX_GPIO_Port GPIOD
#define CONTACTOR_COIL_Pin GPIO_PIN_12
#define CONTACTOR_COIL_GPIO_Port GPIOD
#define FAULT_1_Pin GPIO_PIN_14
#define FAULT_1_GPIO_Port GPIOD
#define FAULT_2_Pin GPIO_PIN_15
#define FAULT_2_GPIO_Port GPIOD
#define FAULT_LED_Pin GPIO_PIN_6
#define FAULT_LED_GPIO_Port GPIOC
#define GREEN_LED1_Pin GPIO_PIN_7
#define GREEN_LED1_GPIO_Port GPIOC
#define GREEN_LED2_Pin GPIO_PIN_0
#define GREEN_LED2_GPIO_Port GPIOG
#define SPI1_CS_Pin GPIO_PIN_1
#define SPI1_CS_GPIO_Port GPIOG
#define M1_PWM_UH_Pin GPIO_PIN_8
#define M1_PWM_UH_GPIO_Port GPIOA
#define M1_PWM_VH_Pin GPIO_PIN_9
#define M1_PWM_VH_GPIO_Port GPIOA
#define M1_PWM_WH_Pin GPIO_PIN_10
#define M1_PWM_WH_GPIO_Port GPIOA
#define M1_PWM_UL_Pin GPIO_PIN_11
#define M1_PWM_UL_GPIO_Port GPIOA
#define M1_PWM_VL_Pin GPIO_PIN_12
#define M1_PWM_VL_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Start_Stop_Pin GPIO_PIN_10
#define Start_Stop_GPIO_Port GPIOC
#define Start_Stop_EXTI_IRQn EXTI15_10_IRQn
#define M1_ENCODER_A_Pin GPIO_PIN_3
#define M1_ENCODER_A_GPIO_Port GPIOD
#define M1_ENCODER_B_Pin GPIO_PIN_4
#define M1_ENCODER_B_GPIO_Port GPIOD
#define INDEX_Pin GPIO_PIN_5
#define INDEX_GPIO_Port GPIOD
#define INDEX_EXTI_IRQn EXTI9_5_IRQn
#define UART_TX_Pin GPIO_PIN_0
#define UART_TX_GPIO_Port GPIOE
#define UART_RX_Pin GPIO_PIN_1
#define UART_RX_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
typedef enum {
  NO_FAULTS = 0,
  START_TIME_OVF = 99,
  CLOSED_LOOP_ERR = 98,
  RAMPTIME_TOO_LONG = 97,
  ENCODER_INDEX_LOAD_FAIL = 96,
  BAD_MOTOR_INDEX = 95,
  PCM_CANTIMER_OVF = 94,
  PCM_BAD_CAN_MSG = 93,
  CC_CLOSED_LOOP_FAIL = 92,
  CC_OVC_STALL = 91,
  ENCODER_SPI_READ_FAIL = 90,
  ENCODER_ABI_SPI_DELTA = 89,
  ENCODER_DIR_ERROR = 88,
  ENCODER_INDEX_ERROR = 87,
  ENCODER_TRANSITION_ERROR = 86,
  CC_START_SEQ_FAIL = 85,
} CustomErrors;

typedef struct{
  uint8_t SWITCH;
  int16_t SET_RPM;
  uint8_t MUL_FACT;
  uint16_t SEC;
  int16_t SPEED_RPM;
  uint16_t ABS_SPEED_RPM;
  uint8_t PRE_FAULT;
  uint8_t FAULT_OCCURED;
  uint8_t MY_MOTOR_STATE;
  int16_t DC_VOLTAGE;
  int8_t POLARITY;
  uint8_t MODE;
  uint8_t ALIGN;
  uint16_t SET_AMPS;
  uint16_t RAMPUPTIME;
  CustomErrors CustomFaults;
}SystemVariables_Struct_t;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
