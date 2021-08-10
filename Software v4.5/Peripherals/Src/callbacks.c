#include "stepper.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "task.h"
#include "print.h"
#include "stepper.h"
#include "tempc.h"
#include "fluor.h"
#include <string.h>


 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  static portBASE_TYPE xHigherPriorityTaskWoken;

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(hstpr2.htim_cnt->Instance==htim->Instance)
  {
    hstpr2.Count ++;
    if(hstpr2.Count>=hstpr2.num_of_pulses)
    {
      HAL_TIM_PWM_Stop(htim, hstpr2.ch);
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
      hstpr2.State = STEPPER_STATE_READY;
      if(STEPPER_STATE_READY==hstpr1.State)
      {
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(StepperBusyBiSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
  if(hstpr1.htim_cnt->Instance==htim->Instance)
  {
    hstpr1.Count ++;
    if(hstpr1.Count>=hstpr1.num_of_pulses)
    {
      HAL_TIM_PWM_Stop(htim, hstpr1.ch);
      __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
      hstpr1.State = STEPPER_STATE_READY;
      if(STEPPER_STATE_READY==hstpr2.State)
      {
        xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(StepperBusyBiSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }

  /* USER CODE END Callback 1 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint8_t flag=0x00;

  if(GPIO_PIN_10==GPIO_Pin)
  {
    STEPPER_DIR2 = DIR_CCW;
    hstpr2.State = STEPPER_STATE_BUSY;
    hstpr2.num_of_pulses=hstpr1.num_of_pulses;
    __HAL_TIM_ENABLE_IT(hstpr2.htim_cnt, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start(hstpr2.htim_cnt, hstpr2.ch);
    flag |= 0x01;
  }
  if(GPIO_PIN_11==GPIO_Pin)
  {
    STEPPER_DIR2 = DIR_CW;
    hstpr2.State = STEPPER_STATE_BUSY;
    hstpr2.num_of_pulses=hstpr1.num_of_pulses;
    __HAL_TIM_ENABLE_IT(hstpr2.htim_cnt, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start(hstpr2.htim_cnt, hstpr2.ch);
    flag |= 0x02;
  }
  if(0x03==flag)
  {
    flag = 0x00;
    hstpr1.num_of_pulses=05;
    hstpr2.num_of_pulses=05;
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

  if(UART4==huart->Instance)
  {
    xSemaphoreGiveFromISR(ReceiveBufferBusyBiSemHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  if(UART5==huart->Instance)
  {
    xSemaphoreGiveFromISR(FluorGotBiSemHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  if(USART3==huart->Instance)
  {
    xSemaphoreGiveFromISR(TempcRepliedBiSemHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if(USART3==huart->Instance)
  {
    memset(TEMPC_ReceiveBuffer, 0x0, 64);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3, TEMPC_ReceiveBuffer, 0x40);
  }
  if(UART5==huart->Instance)
  {
    memset(FLUOR_ReceiveBuffer, 0x00, 32);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart5, FLUOR_ReceiveBuffer, 0x20);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  static portBASE_TYPE xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(PrintBusyBiSemHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
