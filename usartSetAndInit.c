#include "usartSetAndInit.h"


UART_HandleTypeDef uartModule;

uint16_t rByte;

uint16_t sByte;

void USART3_IRQHandler(void);

void usartSetAndInit(void) {
	
	
	
	
	// usart 3 enable
	__HAL_RCC_USART3_CLK_ENABLE();

	__HAL_RCC_GPIOD_CLK_ENABLE();
	

	GPIO_InitTypeDef uartPins;

	uartPins.Pin = GPIO_PIN_8 | GPIO_PIN_9;

	uartPins.Mode = GPIO_MODE_AF_PP;

	uartPins.Alternate = GPIO_AF7_USART3;

	uartPins.Speed = GPIO_SPEED_HIGH;

	uartPins.Pull = GPIO_PULLUP;


	HAL_GPIO_Init(GPIOD, &uartPins);
	
	
	// set and init uart
	
	//UART_HandleTypeDef uartModule;
	
	uartModule.Instance = USART3;
	
	uartModule.Init.BaudRate = 9600; // baud speed
	
	uartModule.Init.WordLength = 8; // 8 or 9 bit
	
	uartModule.Init.StopBits = 1; // amount of stop bit 1 or 2
	
	uartModule.Init.Parity = UART_PARITY_NONE ; // has even(chetnoct') bit
	
	uartModule.Init.Mode = UART_MODE_TX_RX; // regim raboty recive, transm or together
	
	
	HAL_UART_Init(&uartModule);
	
	
	// allow prerivanie of module uart;
	
	HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void USART3_IRQHandler(void){	
	HAL_UART_IRQHandler(&uartModule);	
}

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart){
	if(huart == &uartModule) {
		sByte = rByte;
		sByte += 10;
		HAL_UART_Receive_IT(&uartModule, &rByte, 1);
		HAL_UART_Transmit_IT(&uartModule, &sByte, 1);
	}     
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &uartModule) {
		
	}		
}