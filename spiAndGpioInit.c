#include "spiAndGpioInit.h"

SPI_HandleTypeDef spiStruct;

uint8_t flagFromSpi = 0;

cmd_t sendCmd;


cmd_t readCmd;

void spiAndGpioInit(void) {

	__HAL_RCC_SPI2_CLK_ENABLE();
	
	
	// init PB12; PB13; PB14; PB15
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	GPIO_InitTypeDef spiGpioInit;
	
	spiGpioInit.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	
	spiGpioInit.Mode = GPIO_MODE_AF_PP;

	spiGpioInit.Alternate = GPIO_AF5_SPI2;
	
	spiGpioInit.Speed = GPIO_SPEED_LOW;
	
	spiGpioInit.Pull = GPIO_NOPULL;
	
	HAL_GPIO_Init(GPIOB, &spiGpioInit);
	
	// set INIT SPI
	
	
	spiStruct.Instance = SPI2;
	
	spiStruct.Init.Mode = SPI_MODE_MASTER;
	
	spiStruct.Init.Direction = SPI_DIRECTION_2LINES;
	
	spiStruct.Init.DataSize = SPI_DATASIZE_8BIT;
	
	spiStruct.Init.CLKPolarity = SPI_POLARITY_LOW;

	spiStruct.Init.CLKPhase = SPI_PHASE_1EDGE;
	
	spiStruct.Init.NSS = SPI_NSS_HARD_OUTPUT;
	
	spiStruct.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	
	spiStruct.Init.FirstBit = SPI_FIRSTBIT_MSB;
	
	
	HAL_SPI_Init(&spiStruct);
	
	HAL_NVIC_EnableIRQ(SPI2_IRQn);

}

void SPI2_IRQHandler(void){
	HAL_SPI_IRQHandler(&spiStruct);	
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &spiStruct){
		flagFromSpi = 1;
	}	
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &spiStruct){
		HAL_SPI_Receive_IT(&spiStruct, &readCmd.val, 1);
	}	
}
