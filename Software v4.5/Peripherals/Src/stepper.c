/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "main.h"
#include "stepper.h"


/* Private variables ---------------------------------------------------------*/
STEPPER_HandleTypeDef hstpr1;
STEPPER_HandleTypeDef hstpr2;

STEPPER_ReceiveTypeDef UART5_ReceiveBuffer;
STEPPER_TransmitTypeDef UART5_TransmitBuffer;


/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes the STEPPER parameters according to the specified
  *         values in the STEPPER_HandleTypeDef.
  * @note   GPIOs and TIM1, TIM2 initialization should be finished at the moment
  * @param  STEPPER handle
  * @retval HAL status
  */
HAL_StatusTypeDef STEPPER_Init(void)
{
  if ((&hstpr1==NULL) || (&hstpr2==NULL))
  {
    return HAL_ERROR;
  }

  if (hstpr1.State == STEPPER_STATE_RESET)
  {
    hstpr1.Resolution = 50000;
    hstpr1.htim_cnt = &htim14;
    hstpr1.ch = TIM_CHANNEL_1;
    hstpr1.Count = 0;
  }
  if (hstpr2.State == STEPPER_STATE_RESET)
  {
    hstpr2.Resolution = 50000;
    hstpr2.htim_cnt = &htim13;
    hstpr2.ch = TIM_CHANNEL_1;
    hstpr2.Count = 0;
  }

  /* Set the STEPPER state */
  hstpr1.State = STEPPER_STATE_BUSY;
  hstpr2.State = STEPPER_STATE_BUSY;

  /* Basic configuration */
    STEPPER_DIR1 = DIR_CW;
    STEPPER_AWO1 = 1;
    STEPPER_CS1 = 1;

    STEPPER_DIR2 = DIR_CW;
    STEPPER_AWO2 = 1;
    STEPPER_CS2 = 1;

  /* Initialize the STEPPER state*/
  hstpr1.State = STEPPER_STATE_READY;
  hstpr2.State = STEPPER_STATE_READY;
  
  return HAL_OK;
}

/**
  * @brief  Generate specific number of pulses
  * @note   TIM3, TIM4 initialization should be finished at the moment
  * @param  STEPPER handle
  * @retval HAL status
  */
HAL_StatusTypeDef STEPPER_MoveRelative(int32_t deltaX, int32_t deltaY)
{
  int32_t theta1, theta2;
  
  if((hstpr1.State!=STEPPER_STATE_READY)||(hstpr2.State!=STEPPER_STATE_READY))
  {
    return HAL_BUSY;
  }
  
  theta1 = (deltaX+deltaY)*500;
  theta2 = (deltaX-deltaY)*500;
  STEPPER_DIR1 = DIR_CW;
  if(theta1<0)
  {
    theta1 = - theta1;
    STEPPER_DIR1 = DIR_CCW;
  }
  hstpr1.Count = 0;
  hstpr1.num_of_pulses = theta1;
  STEPPER_DIR2 = DIR_CW;
  if(theta2<0)
  {
    theta2 = - theta2;
    STEPPER_DIR2 = DIR_CCW;
  }
  hstpr2.Count = 0;
  hstpr2.num_of_pulses = theta2;
  
  if(theta1>0)
  {
    hstpr1.State = STEPPER_STATE_BUSY;
  }
  if(theta2>0)
  {
    hstpr2.State = STEPPER_STATE_BUSY;
  }
  vTaskSuspendAll();
  if(theta1>0)
  {
    __HAL_TIM_ENABLE_IT(hstpr1.htim_cnt, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start(hstpr1.htim_cnt, hstpr1.ch);
  }
  if(theta2>0)
  {
    __HAL_TIM_ENABLE_IT(hstpr2.htim_cnt, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start(hstpr2.htim_cnt, hstpr2.ch);
  }
  xTaskResumeAll();
  xSemaphoreTake(StepperBusyBiSemHandle, portMAX_DELAY);

  return HAL_OK;
}

HAL_StatusTypeDef STEPPER_MoveTo(int16_t x, int16_t y)
{
  int16_t deltaX, deltaY;
  
  deltaX = x - (hstpr1.Count+hstpr2.Count)>>1;
  deltaY = y - (-hstpr1.Count+hstpr2.Count)>>1;
  
  STEPPER_MoveRelative(deltaX, deltaY);

  return HAL_OK;
}

HAL_StatusTypeDef STEPPER_SetSpeed(int16_t omega1, int16_t omega2)
{
  __HAL_TIM_SET_AUTORELOAD(hstpr1.htim_cnt, 4200*12/omega1);
  __HAL_TIM_SET_AUTORELOAD(hstpr2.htim_cnt, 4200*12/omega2);

  return HAL_OK;
}

void STEPPER_EmergencyStop(void)
{
  STEPPER_AWO1=0;
  STEPPER_AWO2=0;
  HAL_TIM_PWM_Stop(hstpr1.htim_cnt, hstpr1.ch);
  HAL_TIM_PWM_Stop(hstpr2.htim_cnt, hstpr2.ch);
}
