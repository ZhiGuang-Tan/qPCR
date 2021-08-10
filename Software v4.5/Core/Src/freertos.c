/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <math.h>
#include "print.h"
#include "tempc.h"
#include "fluor.h"
#include "stepper.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t tempcTaskHandle;
const osThreadAttr_t tempcTask_attributes = {
  .name = "tempcTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 8
};
osThreadId_t stepperTaskHandle;
const osThreadAttr_t stepperTask_attributes = {
  .name = "stepperTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 8
};

QueueHandle_t TempcMessageQueueHandle;
QueueHandle_t StepperMessageQueueHandle;
osSemaphoreId_t FluorSendSuccBiSemHandle;
FluorHandle_TypeDef fluor_handle[16];


/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void scan_channels(void)
{
  int32_t i;

  STEPPER_MoveRelative(0, -23);
    vTaskDelay(300);
  for(i=0; i<7; i++)
  {
    // if(fluor_handle[i].IsActivated)
    // {
      get_fluor_data(i);
      // xSemaphoreTake(FluorSendSuccBiSemHandle, portMAX_DELAY);
    // }
    STEPPER_MoveRelative(23, 0);
    vTaskDelay(300);
  }
  // if(fluor_handle[i].IsActivated)
  // {
    get_fluor_data(i);
    // xSemaphoreTake(FluorSendSuccBiSemHandle, portMAX_DELAY);
  // }
  STEPPER_MoveRelative(0, -23);
    vTaskDelay(300);
  for(i=8; i<15; i++)
  {
    // if(fluor_handle[i].IsActivated)
    // {
      get_fluor_data(i);
      // xSemaphoreTake(FluorSendSuccBiSemHandle, portMAX_DELAY);
    // }
    STEPPER_MoveRelative(-23, 0);
    vTaskDelay(300);
  }
  // if(fluor_handle[i].IsActivated)
  // {
    get_fluor_data(i);
    // xSemaphoreTake(FluorSendSuccBiSemHandle, portMAX_DELAY);
  // }
  STEPPER_MoveRelative(0, 46);
  vTaskDelay(100);
}


void StartTempcTask(void *argument)
{
  static TEMPC_CommandTypeDef cmd;
  static TEMPC_HandleTypeDef htempc;
  portTickType xLastExecutionTime;
  int32_t i;

  float target, tmp;
  
  FluorGotBiSemHandle = xSemaphoreCreateBinary();
  TempcRepliedBiSemHandle = xSemaphoreCreateBinary();
  TempcMessageQueueHandle = xQueueCreate(1, sizeof(TEMPC_CommandTypeDef));
  
  htempc.stages = 2;
  htempc.temp_stage[0] = 95.00f;
  htempc.time_stage[0] = 5000/portTICK_RATE_MS;
  htempc.temp_stage[1] = 60.00f;
  htempc.time_stage[1] = 30000/portTICK_RATE_MS;

  for(;;)
  {
    xQueueReceive(TempcMessageQueueHandle, (void *)&cmd, portMAX_DELAY);
    switch(cmd.cmd)
    {
      case 0x01:
      {
        print_uart(&huart4, "RecSta\r\n");
        TEMPC_FAN_NEN = 1;

        TEMPC_Start(1);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        TEMPC_Start(2);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);

        TEMPC_SetTemperature(1, 52.00f);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        TEMPC_SetTemperature(2, 52.00f);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        do
        {
          vTaskDelay(100);
          tmp = (REALTEMP[0]+REALTEMP[1])/2.0f;
          // print_uart(&huart4, "%f\r\n", tmp);
        } while(fabs(tmp-52.00f)>1.0f);
        vTaskDelay(300000);
        TEMPC_SetTemperature(1, 95.00f);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        TEMPC_SetTemperature(2, 95.00f);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        do
        {
          vTaskDelay(100);
          tmp = (REALTEMP[0]+REALTEMP[1])/2.0f;
          // print_uart(&huart4, "%f\r\n", tmp);
        } while(fabs(tmp-95.00f)>1.0f);
        vTaskDelay(10000);

        while(cmd.num_of_cycles>0)
        {
          cmd.num_of_cycles --;

          for(i=0; i<htempc.stages; i++)
          {
            target = htempc.temp_stage[i];
            // print_uart(&huart4, "\r\n------%5.2f deg------\r\n", tmp);
            TEMPC_SetTemperature(1, target);
            // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
            TEMPC_SetTemperature(2, target);
            // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
            do
            {
              vTaskDelay(100);
              tmp = (REALTEMP[0]+REALTEMP[1])/2.0f;
              // print_uart(&huart4, "%f\r\n", tmp);
            } while(fabs(tmp-target)>1.0f);
            vTaskDelay(htempc.time_stage[i]);
          }
          scan_channels();
          print_uart(&huart4, "RecOver\r\n");
        }
        TEMPC_Stop(1);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        TEMPC_Stop(2);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        TEMPC_FAN_NEN = 0;
        print_uart(&huart4, "Done!\r\n");
        STEPPER_MoveRelative(-65, 182);

        break;
      }
      case 0x02:
      {
        print_uart(&huart4, "RecSta\r\n");
        TEMPC_FAN_NEN = 1;
        TEMPC_Start(1);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        TEMPC_Start(2);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        TEMPC_SetTemperature(1, 63.00f);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        TEMPC_SetTemperature(2, 63.00f);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);do
        do {
          vTaskDelay(100);
          tmp = (REALTEMP[0]+REALTEMP[1])/2.0f;
          // print_uart(&huart4, "%f\r\n", tmp);
        } while(fabs(tmp-63.00f)>0.5f);
        for(i=0; i<cmd.num_of_cycles; i++)
        {
          xLastExecutionTime = xTaskGetTickCount();
          scan_channels();
          print_uart(&huart4, "RecOver\r\n");
          vTaskDelayUntil(&xLastExecutionTime, 60000/portTICK_RATE_MS );
        }
        TEMPC_Stop(1);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        TEMPC_Stop(2);
        // print_uart(&huart4, "%s\r\n", TEMPC_ReceiveBuffer);
        TEMPC_FAN_NEN = 0;
        print_uart(&huart4, "Done!\r\n");
        STEPPER_MoveRelative(-65, 182);
        break;
      }
      case 0x03:
      {
        TEMPC_FAN_NEN = 1;
        break;
      }
      case 0x04:
      {
        TEMPC_FAN_NEN = 0;
      }
    }
  }
}

void StartStepperTask(void *argument)
{
  static StepperCMD_TypeDef cmd;
  StepperMessageQueueHandle = xQueueCreate(4, sizeof(StepperCMD_TypeDef));

  for(;;)
  {
    xQueueReceive(StepperMessageQueueHandle, (void *)&cmd, portMAX_DELAY);
    switch(cmd.cmd)
    {
      case 0x00:
      {
        STEPPER_AWO1 = 1;
        STEPPER_AWO2 = 1;
        hstpr1.State = STEPPER_STATE_READY;
        hstpr2.State = STEPPER_STATE_READY;
        break;
      }
      case 0x01:
      {
        print_uart(&huart4, "Waiting...\r\n");
        // if(GPIO_PIN_SET==HAL_GPIO_ReadPin(GPIOB, 10))
        // {
        //   STEPPER_MoveRelative(0, 23);
        //   vTaskDelay(2000);
        // }
        // if(GPIO_PIN_SET==HAL_GPIO_ReadPin(GPIOB, 11))
        // {
        //   STEPPER_MoveRelative(23, 0);
        //   vTaskDelay(2000);
        // }
        if(GPIO_PIN_SET==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11))
        {
          STEPPER_MoveRelative(23, 0);
        }
        do
        {
          HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        } while(HAL_OK!=STEPPER_MoveRelative(-32767, -32767));
        vTaskDelay(100);
        while(HAL_OK!=STEPPER_MoveRelative(0, 250));
        vTaskDelay(100);
        print_uart(&huart4, "Done!\r\n");
        break;
      }
      case 0x02:
      {
        STEPPER_MoveRelative(65, -182);
        vTaskDelay(1000);
        break;
      }
      default:
      {
        hstpr1.State = STEPPER_STATE_RESET;
        hstpr2.State = STEPPER_STATE_RESET;
        STEPPER_AWO1 = 0;
        STEPPER_AWO2 = 0;
      }
    }
  }
}

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
