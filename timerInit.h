#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "stm32f4xx_hal_tim.h"

extern TIM_HandleTypeDef timer;

void timerInit(void);

void TIM2_IRQHandler(void);


