#include "stm32f4xx_hal.h"
#include "main.h"
#include "print.h"
#include "tempc.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>


uint8_t TEMPC_ReceiveBuffer[64];
osSemaphoreId_t TempcRepliedBiSemHandle;

static char cmd[32];
static int32_t length;

TEMPC_HandleTypeDef htempc;

static HAL_StatusTypeDef CheckCommand(void)
{
  if(length<0 || length>32)
  {
    print_uart(&huart4, "Invalid TEMPC command!");
    return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef TEMPC_SendCommand(const char *cmd)
{
  int32_t i;

  i=0;
  if(HAL_OK==CheckCommand())
  {
    do
    {
      if(i>20)
      {
        error("Tempc disconnected!\r\n", 21);
      }
      print_uart(&huart3, cmd);
      i ++;
    } while(pdFAIL==xSemaphoreTake(TempcRepliedBiSemHandle, 20));
    return HAL_OK;
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef TEMPC_SetTemperature(int32_t id, float target)
{
  int32_t i;
    do
    {
  memset(cmd, 0x0, 32);
  length = snprintf(cmd, 32, "TC%1d:TCADJUSTTEMP=%5.2f\r", id, target);
  if(HAL_OK==CheckCommand())
  {
      TEMPC_SendCommand(cmd);
      for(i=0; i<32; i++)
      {
        if('='==TEMPC_ReceiveBuffer[i])
        {
          break;
        }
      }
      if('1'==TEMPC_ReceiveBuffer[i+1])
      {
        break;
      }
  }
    } while(1);
    return HAL_OK;
  //return HAL_ERROR;
}

HAL_StatusTypeDef TEMPC_Start(int32_t id)
{
  memset(cmd, 0x0, 32);
  length = snprintf(cmd, 32, "TC%1d:TCSW=1\r", id);
  if(HAL_OK==CheckCommand())
  {
    TEMPC_SendCommand(cmd);
    return HAL_OK;
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef TEMPC_Stop(int32_t id)
{
  memset(cmd, 0x0, 32);
  length = snprintf(cmd, 32, "TC%1d:TCSW=0\r", id);
  if(HAL_OK==CheckCommand())
  {
    TEMPC_SendCommand(cmd);
    return HAL_OK;
  }
  return HAL_ERROR;
}

float TEMPC_GetTemperature(int32_t id)
{
  int32_t i;
  char *ptr;
  memset(cmd, 0x0, 32);

  i=0;
  do
  {
    if(i>20)
    {
      return 0.0f;
    }
    length = snprintf(cmd, 32, "TC%1d:TCACTUALTEMP?\r", id);
    if(HAL_OK==CheckCommand())
    {
      TEMPC_SendCommand(cmd);
      ptr = strstr((char *)TEMPC_ReceiveBuffer, "TCACTUALTEMP=");
    }
    i ++;
  } while(NULL==ptr);
  return strtof(ptr+13, NULL);
}
