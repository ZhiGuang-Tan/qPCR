#ifndef __STEPPER_H__
#define __STEPPER_H__


#include "stm32f4xx_hal.h"



#define STEPPER_MAX_COUNT 120


// #define STEPPER_PLS1  BITBAND(GPIOE_BASE+20, 7)
#define STEPPER_DIR1  BITBAND(GPIOA_BASE+20, 2)
#define STEPPER_AWO1  BITBAND(GPIOA_BASE+20, 3)
#define STEPPER_CS1   BITBAND(GPIOA_BASE+20, 4)
#define STEPPER_ALM1  BITBAND(GPIOA_BASE+16, 1)
#define STEPPER_TIM1  BITBAND(GPIOA_BASE+16, 0)

// #define STEPPER_PLS2  BITBAND(GPIOA_BASE+20, 6)
#define STEPPER_DIR2  BITBAND(GPIOC_BASE+20, 1)
#define STEPPER_AWO2  BITBAND(GPIOC_BASE+20, 2)
#define STEPPER_CS2   BITBAND(GPIOC_BASE+20, 3)
#define STEPPER_ALM2  BITBAND(GPIOC_BASE+16, 0)
#define STEPPER_TIM2  BITBAND(GPIOA_BASE+16, 5)

#define DIR_CW  0
#define DIR_CCW 1


typedef enum
{
  STEPPER_STATE_RESET             = 0x00U,    /*!< Peripheral not yet initialized or disabled  */
  STEPPER_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
  STEPPER_STATE_BUSY              = 0x02U,    /*!< The stepper is running                      */
  STEPPER_STATE_ERROR             = 0x03U     /*!< Reception process is ongoing                */
} STEPPER_StateTypeDef;

typedef struct
{
  int32_t Resolution;
  STEPPER_StateTypeDef State;
  TIM_HandleTypeDef *htim_pls;
  TIM_HandleTypeDef *htim_cnt;
  uint32_t ch;
  int32_t Count;
  int32_t num_of_pulses;
} STEPPER_HandleTypeDef;

typedef struct
{
  uint8_t _res;
  uint8_t cmd;
  int8_t vx;
  int8_t vy;
  int16_t dx;
  int16_t dy;
} __attribute__((packed)) STEPPER_ReceiveTypeDef;

typedef struct
{
  uint8_t state;
  int16_t x;
  int16_t y;
} __attribute__((packed)) STEPPER_TransmitTypeDef;


extern STEPPER_ReceiveTypeDef UART5_ReceiveBuffer;
extern STEPPER_TransmitTypeDef UART5_TransmitBuffer;

extern STEPPER_HandleTypeDef hstpr1;
extern STEPPER_HandleTypeDef hstpr2;

extern int32_t STEPPER_Count_Left1;
extern int32_t STEPPER_Count_Left2;


HAL_StatusTypeDef STEPPER_Init(void);
HAL_StatusTypeDef STEPPER_MoveRelative(int32_t, int32_t);
HAL_StatusTypeDef STEPPER_MoveTo(int16_t, int16_t);
HAL_StatusTypeDef STEPPER_SetSpeed(int16_t, int16_t);
void STEPPER_EmergencyStop(void);


#endif /* #ifndef __STEPPER_H__ */
