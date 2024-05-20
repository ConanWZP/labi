#include "timerInit.h"


TIM_HandleTypeDef timer;

void timerInit(void){

	
	timer.Instance = TIM2;

	timer.Init.Prescaler = 12500;
	timer.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer.Init.Period = 1000;

	__HAL_RCC_TIM2_CLK_ENABLE();

	HAL_TIM_Base_Init(&timer);

	HAL_TIM_Base_Start_IT(&timer);

	// priorety of prerivanie, if  in same time TIM1 and TIM2 will be used firtst TIM2
	HAL_NVIC_EnableIRQ(TIM2_IRQn); 
}

void TIM2_IRQHandler(void){	
    if (__HAL_TIM_GET_FLAG(&timer, TIM_FLAG_UPDATE) != RESET){      
          __HAL_TIM_CLEAR_FLAG(&timer, TIM_FLAG_UPDATE);           
					
					HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
    } 		
}

