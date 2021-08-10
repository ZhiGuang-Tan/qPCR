#ifndef __FLUOR_H__
#define __FLUOR_H__


#define FLUOR_LED_NEN BITBAND(GPIOE_BASE+20, 9)


extern osSemaphoreId_t FluorGotBiSemHandle;
extern uint8_t FLUOR_ReceiveBuffer[32];


HAL_StatusTypeDef get_fluor_data(int32_t index);


#endif /* #ifndef __FLUOR_H__ */
