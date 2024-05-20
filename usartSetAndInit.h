#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "stm32f4xx_hal_tim.h"


extern UART_HandleTypeDef uartModule;

extern uint16_t rByte;

extern uint16_t sByte;

void usartSetAndInit(void);

void USART3_IRQHandler(void);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
