#include "initADC.h"
#include "timerInit.h"
#include "pwmSetPulse.h"
#include "initShim.h"
#include "initTim4.h"



GPIO_InitTypeDef GPIO_ADCInitStruct;
ADC_HandleTypeDef adcStruct;
ADC_ChannelConfTypeDef adcConfig;

DMA_HandleTypeDef dmaStruct;

// task 12
uint16_t adc_buffer[2];

uint16_t dma_buffer[2];

// task 13
ADC_ChannelConfTypeDef adcNewConfig;

void initAdc(void) {

	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();
	
	
	
	GPIO_ADCInitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6; 
	GPIO_ADCInitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_ADCInitStruct.Pull = GPIO_NOPULL;
	GPIO_ADCInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_ADCInitStruct); 
	
	
	adcStruct.Instance = ADC1;
	adcStruct.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	adcStruct.Init.Resolution = ADC_RESOLUTION10b;
	adcStruct.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	//disable for 4-9 task lr 5-6
	//adcStruct.Init.ScanConvMode = DISABLE;
	adcStruct.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	//adcStruct.Init.ContinuousConvMode = DISABLE;
	adcStruct.Init.ContinuousConvMode = ENABLE;
	
	//adcStruct.Init.NbrOfConversion = 1;
	adcStruct.Init.DiscontinuousConvMode = DISABLE;
	//adcStruct.Init.NbrOfDiscConversion = 0;
	adcStruct.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	adcStruct.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	//disable for 4-9 task lr 5-6
	//adcStruct.Init.DMAContinuousRequests = DISABLE;
	
		// task 13, need modificate adcStruct
	adcStruct.Init.ScanConvMode = ENABLE; // turn on scan
	adcStruct.Init.ContinuousConvMode = ENABLE; 

	adcStruct.Init.NbrOfConversion = 2; // number of channels
	adcStruct.Init.DMAContinuousRequests = ENABLE; // enable DMA
	
	
  HAL_ADC_Init(&adcStruct);
    

	adcConfig.Channel = ADC_CHANNEL_5;
	adcConfig.Rank = 1;
	adcConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
	
	HAL_ADC_ConfigChannel(&adcStruct, &adcConfig);
    
		
		// task 13
/*	adcNewConfig.Channel = ADC_CHANNEL_6;
	adcNewConfig.Rank = 2;
	adcNewConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
	*/
	
	adcConfig.Channel = ADC_CHANNEL_6;
	adcConfig.Rank = 2;
	adcConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
	
	HAL_ADC_ConfigChannel(&adcStruct, &adcConfig);



	
	//HAL_ADC_Start(&adcStruct);
	
	// ne nado dlya dma
	//HAL_NVIC_EnableIRQ(ADC_IRQn);
	
	//HAL_ADC_Start_IT(&adcStruct);
	
	
	
	
	// config DMA
	dmaStruct.Instance = DMA2_Stream0;
	dmaStruct.Init.Channel = DMA_CHANNEL_0;
	dmaStruct.Init.Direction = DMA_PERIPH_TO_MEMORY;
	dmaStruct.Init.PeriphInc = DMA_PINC_DISABLE;
	dmaStruct.Init.MemInc = DMA_MINC_ENABLE;
	dmaStruct.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	dmaStruct.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
	dmaStruct.Init.Mode = DMA_CIRCULAR;
	dmaStruct.Init.Priority = DMA_PRIORITY_LOW;
	
	HAL_DMA_Init(&dmaStruct);
	
	
	
	
	//adc_buffer[0] = 0;
	//adc_buffer[1] = 0;
	
	

	
	


//	dma_buffer = (uint16_t)adc_buffer;
	//uint32_t *dma_buffer = (uint16_t *)adc_buffer; // yavnoe privedenie tipov
	
	// svyazivanie ADC and DMA
	__HAL_LINKDMA(&adcStruct, DMA_Handle, dmaStruct);
	
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	
	HAL_ADC_Start_DMA(&adcStruct, (uint32_t *)adc_buffer, 2); 
	
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	//(adcVar > 1000) ? (pwmSetPulse(1000, timerFour)) : (pwmSetPulse(adcVar, timerFour));
	// this code can be transfered to main, like id was made for 117 line
	
	if (hadc == &adcStruct) {
			
        // obrabotka dannix from ADC for first channel ( red diode )
        uint16_t adc_value_red = (uint16_t)adc_buffer[0];
        // yarkost' of red diod controlled with SHIM
				(adc_value_red > 1000) ? (pwmSetPulse(1000, timerFour)) : (pwmSetPulse(adc_value_red, timerFour));
        
        // obrabotka dannix form ADC for second channel (green diode)
        uint16_t adc_value_green = (uint16_t)adc_buffer[1];
        if (adc_value_green > 500) {
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
        }
    }
	
	
}

void ADC_IRQHandler(void){
	HAL_ADC_IRQHandler(&adcStruct);	
}

void DMA2_Stream0_IRQHandler(void){
	HAL_DMA_IRQHandler(&dmaStruct);	
}

