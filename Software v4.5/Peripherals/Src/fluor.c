#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "main.h"
#include "print.h"
#include "tempc.h"
#include "fluor.h"
#include <string.h>

uint8_t FLUOR_ReceiveBuffer[32];

osSemaphoreId_t FluorGotBiSemHandle;

HAL_StatusTypeDef get_fluor_data(int32_t index)
{
  int32_t i=0;
  do
  {
    print_uart(&huart5, "data\r\n");

    i ++;
    if(i>20)
    {
      error("Waiting for reply from fluor!\r\n", 31);
    }
  } while(pdFAIL==xSemaphoreTake(FluorGotBiSemHandle, 30) || (NULL==strstr((char *)FLUOR_ReceiveBuffer, "data(V)")));// while(HAL_OK!=HAL_UART_Receive(&huart5, FLUOR_ReceiveBuffer, 32, 20));
  fluor_handle[index].value = strtof((char *)FLUOR_ReceiveBuffer+8, NULL);
  // print_uart(&huart4, "%s\r\n", FLUOR_ReceiveBuffer);
  print_uart(&huart4, "FLU%02d_%6.4f\r\n", index, fluor_handle[index].value);
  // xSemaphoreGive(FluorSendSuccBiSemHandle);

  return HAL_OK;
}
