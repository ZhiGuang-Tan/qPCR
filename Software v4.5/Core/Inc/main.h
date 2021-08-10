/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "semphr.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct
{
  uint8_t cmd;
  int32_t dx;
  int32_t dy;
} __attribute__((packed)) StepperCMD_TypeDef;

typedef struct
{
  uint8_t IsActivated;
  float value;
} FluorHandle_TypeDef;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart3;

extern QueueHandle_t TempcMessageQueueHandle;
extern QueueHandle_t StepperMessageQueueHandle;

extern osSemaphoreId_t ReceiveBufferBusyBiSemHandle;
extern osSemaphoreId_t StepperBusyBiSemHandle;
extern osSemaphoreId_t FluorSendSuccBiSemHandle;

extern osThreadId_t defaultTaskHandle;
extern const osThreadAttr_t defaultTask_attributes;
extern float REALTEMP[2];

extern osThreadId_t tempcTaskHandle;
extern osThreadId_t stepperTaskHandle;
extern const osThreadAttr_t tempcTask_attributes;
extern const osThreadAttr_t stepperTask_attributes;

extern FluorHandle_TypeDef fluor_handle[16];

extern uint8_t FluorDataReady;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define BITBAND_ADDR(regAddr, bitNum) ((volatile unsigned long *)((regAddr & 0xF0000000)+0x2000000+((regAddr&0xFFFFF)<<5)+(bitNum<<2)))
#define BITBAND(regAddr, bitNum) (*((volatile unsigned long *)((regAddr & 0xF0000000)+0x2000000+((regAddr&0xFFFFF)<<5)+(bitNum<<2))))

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void StartTempcTask(void *);
void StartStepperTask(void *);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
