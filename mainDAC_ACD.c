#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "stm32f4xx_hal_tim.h"

#include "leds.h"
#include "button.h"
#include "timerInit.h"
#include "pwmSetPulse.h"
#include "initShim.h"
#include "initTim4.h"
#include "usartSetAndInit.h"
#include "spiAndGpioInit.h"
#include "initDAC.h"
#include "initADC.h"

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


	



// dlya 4 labi
void EXTI0_IRQHandler(void);

void turnOnAll(void);
void turnOffALl(void);

int main (void){
	
	HAL_Init();

	SystemClock_Config();

	// lr 5
	
	
	//initDac();
	//uint8_t eightBitVar = 0;
	//HAL_DAC_SetValue(&dacStruct, DAC_CHANNEL_1, DAC_ALIGN_8B_R, eightBitVar);

	
	initAdc();
	uint32_t adcVar = 0;
	
	// zadaniya part 1 dlya lab 2
	/*
	ledsRegInit();
	ledsHalInit();
	buttonRegInit();
	buttonHalInit();
	switchLeds();
	*/


	// probably need to comment for fourth task of part 2 lab 2
	//ledsRegInit();
	//ledsHalInit();
	
	
	//buttonRegInit();
	//buttonHalInit();



	// call func of third task of part 2 lab 2
	//timerInit();

	// call func of fourth task lr 2 + for laba 5
	initShim();
		
	initTim4();


	
	
	
	//uint16_t varFlag = 0;
	//uint16_t cnt = 0;
	
	
	// 4 lr
	/*
	spiAndGpioInit();
	
	// vse gpio for svetodiodov
	allLedsHalInit();
	
	buttonHalInit();
	HAL_NVIC_EnableIRQ(EXTI0_IRQn); 

	// task 7 laba 4
	//HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	
	sendCmd.cmd_s.cmd = 0x0E;
	
	HAL_SPI_Transmit(&spiStruct, &sendCmd.val, 1, 12000);	
	*/
	//uint16_t varFlag = 1;
	//uint32_t cnt = 1;
	
	while(1){
		
		//lr 5 task 3
		/*
		if (eightBitVar < 255) {
			eightBitVar++;
		} else {
			eightBitVar = 0;
		}
		HAL_DAC_SetValue(&dacStruct, DAC_CHANNEL_1, DAC_ALIGN_8B_R, eightBitVar);
		HAL_Delay(20);
		*/
			// lr 5 task 5
		/*
		HAL_ADC_PollForConversion(&adcStruct, 100);
		adcVar = HAL_ADC_GetValue(&adcStruct);
		*/
			// lr 5 task 6
		/*
		HAL_ADC_Start(&adcStruct);
		HAL_ADC_PollForConversion(&adcStruct, 100);
		adcVar = HAL_ADC_GetValue(&adcStruct);

		(adcVar > 1000) ? (pwmSetPulse(1000, timerFour)) : (pwmSetPulse(adcVar, timerFour));
		HAL_ADC_Stop(&adcStruct);
		*/

		// lr 5 task 8 and 9 (depend on adcStruct.Init.ContinuousConvMode  = Enable/disable)
		HAL_ADC_Start_IT(&adcStruct);
		adcVar = HAL_ADC_GetValue(&adcStruct);
		(adcVar > 1000) ? (pwmSetPulse(1000, timerFour)) : (pwmSetPulse(adcVar, timerFour));
		
		
		
		//HAL_Delay(1);
		/*
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
		*/
		
		/*
		if (flagFromSpi) {
		
			switch (readCmd.cmd_s.cmd) {
				case 0:
					(readCmd.cmd_s.info == 1) ? (ORANGE_ON()) : (ORANGE_OFF());
					break;
				case 1:
					(readCmd.cmd_s.info == 1) ? (RED_ON()) : (RED_OFF());
					break;
				case 2:
					(readCmd.cmd_s.info == 1) ? (BLUE_ON()) : (BLUE_OFF());
					break;
				case 3:
					(readCmd.cmd_s.info == 1) ? (GREEN_ON()) : (GREEN_OFF());
					break;
				case 0x0F:
					(readCmd.cmd_s.info == 1) ? (turnOnAll()) : (turnOffALl());
					break;
		}
			flagFromSpi = 0;
		}
		*/
	}

// don't use
//TIM2_IRQHandler();

}

void turnOnAll(void) {
	GREEN_ON();
	RED_ON();
	BLUE_ON();
	ORANGE_ON();
}

void turnOffALl(void) {
	GREEN_OFF();
	RED_OFF();
	BLUE_OFF();
	ORANGE_OFF();
}

void USART3_IRQHandler(void);

void HAL_UART_RxCpltCallback (UART_HandleTypeDef *huart);


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

// dlya 4 labi
void EXTI0_IRQHandler(void){	
	//__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

  sendCmd.cmd_s.cmd = 0x01;
	HAL_SPI_Transmit_IT(&spiStruct, &sendCmd.val, 1);
	
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