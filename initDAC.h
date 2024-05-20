#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "stm32f4xx_hal_dac.h"

extern GPIO_InitTypeDef GPIO_InitStruct;
extern DAC_HandleTypeDef dacStruct;
extern DAC_ChannelConfTypeDef dacConfig;



void initDac(void);
