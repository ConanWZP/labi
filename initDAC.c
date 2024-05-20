#include "initDAC.h"


GPIO_InitTypeDef GPIO_InitStruct;
DAC_HandleTypeDef dacStruct;
DAC_ChannelConfTypeDef dacConfig;


void initDac(void) {

	__HAL_RCC_DAC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE(); 
	
	
	GPIO_InitStruct.Pin = GPIO_PIN_4; 
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 
	
	
	dacStruct.Instance = DAC;
  HAL_DAC_Init(&dacStruct);
    

	dacConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	dacConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	HAL_DAC_ConfigChannel(&dacStruct, &dacConfig, DAC_CHANNEL_1);
    
	
	HAL_DAC_Start(&dacStruct, DAC_CHANNEL_1);
	
	
	
}
