#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "stm32f4xx_hal_adc.h"

extern GPIO_InitTypeDef GPIO_ADCInitStruct;
extern ADC_HandleTypeDef adcStruct;
extern ADC_ChannelConfTypeDef adcConfig;

extern DMA_HandleTypeDef dmaStruct;
extern ADC_ChannelConfTypeDef adcNewConfig;

extern uint16_t adc_buffer[2];

extern uint16_t dma_buffer[2];


void initAdc(void);

void ADC_IRQHandler(void);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

