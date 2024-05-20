#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "stm32f4xx_hal_tim.h"


void spiAndGpioInit(void);

void SPI2_IRQHandler(void);


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);

extern SPI_HandleTypeDef spiStruct;

extern uint8_t flagFromSpi;




typedef struct  {
	uint8_t cmd :4;
	uint8_t info :4;	
} byte_t;


// byte_t customName;
// customName.info = 2;



typedef union {
	uint8_t val;
	byte_t cmd_s;
} cmd_t;


extern cmd_t sendCmd;


extern cmd_t readCmd;

