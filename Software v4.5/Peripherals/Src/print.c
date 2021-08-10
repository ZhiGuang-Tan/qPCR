#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "main.h"
#include "print.h"
#include "tempc.h"
#include "stream_buffer.h"

uint8_t TransmitBuffer[PRINT_BUFFER_LENGTH];
StreamBufferHandle_t PRINT_StreamBuffer;
osSemaphoreId_t PrintBusyBiSemHandle;

int32_t print_uart(UART_HandleTypeDef *huart, const char *format, ...)
{
  int32_t length;

  va_list ap;
  va_start(ap, format);

  if(pdPASS==xSemaphoreTake(PrintBusyBiSemHandle, portMAX_DELAY))
  {
    memset(TransmitBuffer, 0x00, PRINT_BUFFER_LENGTH);
    length = vsnprintf((char *)TransmitBuffer, PRINT_BUFFER_LENGTH, (const char *)format, ap);
    if(length>0)
    {
      while(HAL_OK!=HAL_UART_Transmit_DMA(huart, TransmitBuffer, length));
    }
    else
    {
      xSemaphoreGive(PrintBusyBiSemHandle);
    }
  }
 
  va_end(ap);

  return length;
}

void error(const char *message, int32_t length)
{
  vTaskSuspendAll();
  HAL_UART_Transmit(&huart4, (uint8_t *)message, length, 20);
  xSemaphoreGive(PrintBusyBiSemHandle);
  TEMPC_Stop(1);
  TEMPC_Stop(2);
  TEMPC_FAN_NEN = 0;
  while(1)
  {
  }
}


static HAL_StatusTypeDef UART_WaitUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while ((__HAL_UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if ((Timeout == 0U) || ((HAL_GetTick() - Tickstart) > Timeout))
      {
        /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_TXEIE));
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        huart->gState  = HAL_UART_STATE_READY;
        huart->RxState = HAL_UART_STATE_READY;

        /* Process Unlocked */
        __HAL_UNLOCK(huart);

        return HAL_TIMEOUT;
      }
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef UART_ReceiveUntil(UART_HandleTypeDef *huart, uint8_t *pData, const uint8_t Ch, uint32_t Timeout)
{
  uint32_t tickstart = 0U;

  /* Check that a Rx process is not already ongoing */
  if (huart->RxState == HAL_UART_STATE_READY)
  {
    if (pData == NULL)
    {
      return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;

    /* Init tickstart for timeout managment */
    tickstart = HAL_GetTick();

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    /* Check the remain data to be received */
    do
    {
      if (UART_WaitUntilTimeout(huart, UART_FLAG_RXNE, RESET, tickstart, Timeout) != HAL_OK)
      {
        return HAL_TIMEOUT;
      }
      *pData = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
    } while (*pData++ != Ch);
    *pData = '\0';

    /* At end of Rx process, restore huart->RxState to Ready */
    huart->RxState = HAL_UART_STATE_READY;

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
