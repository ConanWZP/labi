#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "stm32f4xx_hal_tim.h"

#include "leds.h"
#include "button.h"
#include "timerInit.h"
#include "pwmSetPulse.h"


/* turn on red */
#define RED_ON() 		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET)
/* turn off red */
#define RED_OFF() 	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET)

/* turn on blue */
#define BLUE_ON() 		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET)
/* turn off blue */
#define BLUE_OFF() 	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET)

/* turn on orange */
#define ORANGE_ON() 		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET)
/* turn off red */
#define ORANGE_OFF() 	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET)

//GPIO_PinState HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);

//void HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
/* turn on green */
#define GREEN_ON() 		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET)
/* turn off green */
#define GREEN_OFF() 	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET)

#define IS_BLUE  HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15)

#define IS_RED  HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)

#define IS_ORANGE  HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12)

#define IS_GREEN  HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13)

void switchLeds(void);
void SystemClock_Config(void);
void SysTick_Handler(void);
//void TIM2_IRQHandler(void);


	


int main (void){
	
HAL_Init();

SystemClock_Config();

	
	
	
/*
	// in leds c
	// 12 and 13 pins
	RCC->AHB1ENR |= (1<<3);
	RCC->AHB1ENR |= (1<<0);
	
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
	GPIOD->ODR |= (1<<12);
	GPIOD->ODR |= (1<<13);
	

	// in button !!!!!!!!!!!!!!!!!!!!!!
	// initialize button PA0
	GPIOA->MODER &= ~(1u<<0);
	GPIOA->MODER &= ~(1u<<1);
	
	//GPIOA->OTYPER &= ~(1u<<0);
	
	// input to ground (pull down)
	GPIOA->PUPDR &= ~(1u<<0);
	GPIOA->PUPDR |= (1u<<1);
	

	GPIOA->OSPEEDR &= ~(1u<<0);
	GPIOA->OSPEEDR &= ~(1u<<1);

*/


/*
	// in leds
	// HAL PD14 PD15
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_InitTypeDef ledInit;
	
	ledInit.Pin = GPIO_PIN_14 | GPIO_PIN_15;
	
	ledInit.Mode = GPIO_MODE_OUTPUT_PP;
	
	ledInit.Speed = GPIO_SPEED_LOW;
	
	HAL_GPIO_Init(GPIOD, &ledInit);
	
	// macros
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	
	
	// HAL PA0
	
	GPIO_InitTypeDef buttonInit;
	
	buttonInit.Pin = GPIO_PIN_0;
	buttonInit.Mode = GPIO_MODE_INPUT;
	buttonInit.Pull = GPIO_PULLDOWN;
	buttonInit.Speed = GPIO_SPEED_LOW;
	
	HAL_GPIO_Init(GPIOA, &buttonInit);
	
*/
/* HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
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
	*/
	
	// high level ODR
	//GPIOD->ODR |= (1<<12);
	//GPIOD->ODR |= (1<<13);
	
	
	// HAL PD14 PD15
	//__HAL_RCC_GPIOD_CLK_ENABLE();
	
	// HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
	/*
	GPIO_InitTypeDef ledInit;
	
	ledInit.Pin = GPIO_PIN_14 | GPIO_PIN_15;
	
	ledInit.Mode = GPIO_MODE_OUTPUT_PP;
	
	ledInit.Speed = GPIO_SPEED_LOW;
	
	HAL_GPIO_Init(GPIOD, &ledInit);
	
	HAL_GPIO_WritePin (GPIOD, GPIO_PIN_15, GPIO_PIN_SET);

	switchLeds();
	*/
/*	
	uint8_t ns=0,ls=0;
	
	
	while(1){
		
		
		ns = (GPIOA->IDR>>0) & 1;
		
		if((ns==1)&&(ls==0)){
			
			
			if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == 1){
				
				
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
				
			} 
			else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14) == 1){
				
				
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
				
			} 
			else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13) == 1){
				
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
				
			} 
			else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == 1){
				
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
				
			}
			
			
		}
		ls = ns;
	}
*/

// zadaniya part 1 dlya lab 2
/*
ledsRegInit();
ledsHalInit();
buttonRegInit();
buttonHalInit();
switchLeds();
*/






// probably need to comment for fourth task of part 2 lab 2
ledsRegInit();
ledsHalInit();
//buttonRegInit();
//buttonHalInit();

/*
while(1){
	BLUE_ON();
	HAL_Delay(500);
	BLUE_OFF();
	HAL_Delay(500);
}
*/


/*
timer.Instance = TIM2;

timer.Init.Prescaler = 12500;
timer.Init.CounterMode = TIM_COUNTERMODE_UP;
timer.Init.Period = 1000;

__HAL_RCC_TIM2_CLK_ENABLE();

HAL_TIM_Base_Init(&timer);

HAL_TIM_Base_Start_IT(&timer);

// priorety of prerivanie, if  in same time TIM1 and TIM2 will be used firtst TIM2
HAL_NVIC_EnableIRQ(TIM2_IRQn); 
*/

// call func of third task of part 2 lab 2
timerInit();


__HAL_RCC_GPIOD_CLK_ENABLE();

// init shim
GPIO_InitTypeDef SHIM;

SHIM.Pin = GPIO_PIN_14;

// altarate + push pull 
SHIM.Mode = GPIO_MODE_AF_PP;

SHIM.Alternate = GPIO_AF2_TIM4;

SHIM.Speed = GPIO_SPEED_LOW;

HAL_GPIO_Init(GPIOD, &SHIM);
	

	
//init timer
__HAL_RCC_TIM4_CLK_ENABLE();

TIM_HandleTypeDef timerFour;

timerFour.Instance = TIM4;

timerFour.Init.Prescaler = 13;
timerFour.Init.CounterMode = TIM_COUNTERMODE_UP;
timerFour.Init.Period = 1000;


HAL_TIM_Base_Init(&timerFour);



	
uint16_t varFlag = 0;
uint16_t cnt = 0;


while(1){

	if (varFlag) {
		pwmSetPulse(cnt, timerFour);
	} else {
		pwmSetPulse(1000-cnt, timerFour);
	}
	
	cnt++;
	
	if (cnt == 1000) {
		varFlag = varFlag ? 0 : 1;
		cnt = 0;
	}
	HAL_Delay(1);

}

// don't use
//TIM2_IRQHandler();

}







// combination of zadanie 2 and 3 of part 1 lab 2
void switchLeds(void){
	
	
	uint8_t ns=0,ls=0;
	
	
	while(1){
		
		//
		ns = (GPIOA->IDR>>0) & 1;
		
		if((ns==1)&&(ls==0)){
			
			
			if(IS_BLUE == 1){
				
				
				BLUE_OFF();
				
				RED_ON();
				
			} 
			else if(IS_RED == 1){
				
				
				RED_OFF();
				GREEN_ON();
				
			} 
			else if(IS_GREEN == 1){
				
				GREEN_OFF();
				ORANGE_ON();
				
			} 
			else if(IS_ORANGE == 1){
				
				ORANGE_OFF();
				BLUE_ON();
				
			}
			
			
		}
		ls = ns;
	}
}


void SystemClock_Config(void)
{
        RCC_OscInitTypeDef RCC_OscInitStruct = {0};
        RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

        /** Configure the main internal regulator output voltage 
        */
        __HAL_RCC_PWR_CLK_ENABLE();
        __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

        /** Initializes the CPU, AHB and APB busses clocks 
        */
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        RCC_OscInitStruct.PLL.PLLM = 8;
        RCC_OscInitStruct.PLL.PLLN = 50;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
        RCC_OscInitStruct.PLL.PLLQ = 7;
        HAL_RCC_OscConfig(&RCC_OscInitStruct);
	
        /** Initializes the CPU, AHB and APB busses clocks 
        */
        RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
        RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
        RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
        RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
        RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
        HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);		
}

void SysTick_Handler(void) {	
    HAL_IncTick();
}
