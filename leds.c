#include "leds.h"

/* CMSIS LEDS GPIO INIT */
void ledsRegInit(void){ 
	RCC->AHB1ENR |= (1<<3);

	
	// output
	GPIOD->MODER |= (1<<24);
	GPIOD->MODER &= ~(1u<<25);
	
	GPIOD->MODER |= (1<<26);
	GPIOD->MODER &= ~(1u<<27); // zero to 27 bit
	
	// push pull
	GPIOD->OTYPER &= ~(1u<<12);
	GPIOD->OTYPER &= ~(1u<<13);
	
	// low speed
	GPIOD->OSPEEDR &= ~(1u<<24);
	GPIOD->OSPEEDR &= ~(1u<<25);
	
	GPIOD->OSPEEDR &= ~(1u<<26);
	GPIOD->OSPEEDR &= ~(1u<<27);
	
	
	// high level ODR
	//GPIOD->ODR |= (1<<12);
	//GPIOD->ODR |= (1<<13);
}

void ledsHalInit(void) {
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	GPIO_InitTypeDef ledInit;
	
	ledInit.Pin = GPIO_PIN_14 | GPIO_PIN_15;
	
	ledInit.Mode = GPIO_MODE_OUTPUT_PP;
	
	ledInit.Speed = GPIO_SPEED_LOW;
	
	HAL_GPIO_Init(GPIOD, &ledInit);
}


void allLedsHalInit(void) {
	
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	GPIO_InitTypeDef allLedInit;
	
	allLedInit.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	
	allLedInit.Mode = GPIO_MODE_OUTPUT_PP;
	
	allLedInit.Speed = GPIO_SPEED_LOW;
	
	HAL_GPIO_Init(GPIOD, &allLedInit);

}