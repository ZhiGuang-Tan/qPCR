#ifndef __TEMPC_H__
#define __TEMPC_H__

#include "main.h"

#define TEMPC_FAN_NEN BITBAND(GPIOE_BASE+20, 8)


#define TEMPC_CMD_OPENTC1  0
#define TEMPC_CMD_GETTEMP1 1
#define TEMPC_CMD_SET1_94  2
#define TEMPC_CMD_SET1_56  3
#define TEMPC_CMD_SET1_72  4
#define TEMPC_CMD_CLOSETC1 5

#define TEMPC_CMD_OPENTC2  6
#define TEMPC_CMD_GETTEMP2 7
#define TEMPC_CMD_SET2_94  8
#define TEMPC_CMD_SET2_56  9
#define TEMPC_CMD_SET2_72  10
#define TEMPC_CMD_CLOSETC2 11


typedef struct
{
  uint8_t stages;
  float temp_stage[4];
  uint32_t time_stage[4];
} TEMPC_HandleTypeDef;

typedef struct
{
  uint8_t cmd;
  uint16_t num_of_cycles;
} __attribute__((packed)) TEMPC_CommandTypeDef;


extern uint8_t TEMPC_ReceiveBuffer[64];
extern osSemaphoreId_t TempcRepliedBiSemHandle;
extern TEMPC_HandleTypeDef htempc;


HAL_StatusTypeDef TEMPC_SendCommand(const char *);
HAL_StatusTypeDef TEMPC_SetTemperature(int32_t, float);
HAL_StatusTypeDef TEMPC_Start(int32_t);
HAL_StatusTypeDef TEMPC_Stop(int32_t);
float TEMPC_GetTemperature(int32_t);


#endif /* #ifndef __TEMPC_H__ */
