#include "button.h"



// initialize button PA0
void buttonRegInit(void){	
	RCC->AHB1ENR |= (1<<0);
	
	// initialize button PA0
	GPIOA->MODER &= ~(1u<<0);
	GPIOA->MODER &= ~(1u<<1);
	
	//GPIOA->OTYPER &= ~(1u<<0);
	
	// input to ground (pull down)
	GPIOA->PUPDR &= ~(1u<<0);
	GPIOA->PUPDR |= (1u<<1);
	

	GPIOA->OSPEEDR &= ~(1u<<0);
	GPIOA->OSPEEDR &= ~(1u<<1);
	
	
}


void buttonHalInit(void){
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitTypeDef buttonInit;
	
	buttonInit.Pin = GPIO_PIN_0;
//	buttonInit.Mode = GPIO_MODE_INPUT;
//	buttonInit.Pull = GPIO_PULLDOWN;
	buttonInit.Speed = GPIO_SPEED_LOW;
	// dlya 4 labi
  buttonInit.Mode = GPIO_MODE_IT_RISING;
  buttonInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &buttonInit);
}