#ifndef __DEBUG_H__
#define __DEBUG_H__


#include "cmsis_os.h"
#include "semphr.h"
#include "stream_buffer.h"


/* Private defines -----------------------------------------------------------*/
#define PRINT_BUFFER_LENGTH  64


extern StreamBufferHandle_t PRINT_StreamBuffer;
extern osSemaphoreId_t PrintBusyBiSemHandle;


/* Exported functions prototypes ---------------------------------------------*/
int32_t print_uart(UART_HandleTypeDef *, const char *, ...);
void error(const char *, int32_t);
HAL_StatusTypeDef UART_ReceiveUntil(UART_HandleTypeDef *huart, uint8_t *pData, uint8_t Ch, uint32_t Timeout);
HAL_StatusTypeDef UART_ReceiveUntil_IT(UART_HandleTypeDef *, uint8_t);

#endif /* #ifndef __DEBUG_H__ */
