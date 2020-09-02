/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "system.h"
#include "PWM.h"
#include "SPI.h"
#include "hcms.h"
#include "characters.h"
#include "Serial.h"
#include "diag/Trace.h"
#include "cmsis_device.h"

TIM_HandleTypeDef htim5;
UART_HandleTypeDef huart1;
static TIM_HandleTypeDef SamplingTimer 	= { .Instance = TIM9 };
static float deltaDegrees	 			= 0.0f;
static float degreesPsec	 			= 0.0f;
static uint32_t timeElapUs 				= 0;
static uint32_t timeElapMs 				= 0;

PWM_Out PWMtimer;

static int InitSamplingTimer(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

static void setHighSystemClk(void);

int main(int argc, char* argv[])
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	setHighSystemClk();
	HAL_Init();

	MX_GPIO_Init();

	init_display();

	InitSamplingTimer();
	MX_TIM5_Init();
	InitSerial(115200, UART_STOPBITS_1, UART_WORDLENGTH_8B, UART_PARITY_NONE);

	InitPWMOutput();

	HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

	RCC->AHB1ENR 	= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOBEN;

	// LED pins setup
	GPIO_InitTypeDef gLEDPins;
	gLEDPins.Pin 	= LED_A | LED_B | LED_C | LED_D;
	gLEDPins.Mode 	= GPIO_MODE_OUTPUT_PP;
	gLEDPins.Pull 	= GPIO_PULLUP;
	gLEDPins.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(LED_PORT, &gLEDPins);

	// OUT_Z pin setup
	GPIO_InitTypeDef gZPin;
	gZPin.Pin 		= Z_INTERRUPT;
	gZPin.Mode 		= GPIO_MODE_IT_RISING;
	gZPin.Pull 		= GPIO_NOPULL;
	gZPin.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &gZPin);
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_1);

	while (1)
	{
		set_brightness(deltaDegrees/10.0f);

		static char str[4] = {0, 0, 0, 0};
		memset(str, 0, 4);
		sprintf(str, "%d", (int)deltaDegrees);
		add_characters(str, 4);
		update_display();

		PWM_adjust_PulseWidth(&PWMtimer.timer, TIM_CHANNEL_1, htim5.Instance->CNT*(PULSE_NS_PER_CNT/100));
	}

}

/**
* @brief This function handles EXTI line0.
*/
void EXTI9_5_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(Z_INTERRUPT);
	WritePin(LED_PORT, LED_D, !ReadPin(LED_PORT, LED_D));
	htim5.Instance->CNT = 0;
}

/*
 * Timer used for motor ESC control
 */
int InitPWMOutput()
{
	PWMtimer.numChannels 	= 1;
	PWMtimer.frequency 		= PWM_FREQ;
	PWMtimer.TIM 			= TIM1;
	PWMtimer.Channel 		= TIM_CHANNEL_1;
	PWMtimer.timer 			= Initialize_PWM(&PWMtimer);

	return HAL_OK;
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{
	TIM_Encoder_InitTypeDef sConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim5.Instance 				= TIM5;
	htim5.Init.Prescaler 		= 0;
	htim5.Init.CounterMode 		= TIM_COUNTERMODE_UP;
	htim5.Init.Period 			= 0xFFFFFFFF;
	htim5.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
	sConfig.EncoderMode 		= TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity 		= TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection 		= TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler 		= TIM_ICPSC_DIV1;
	sConfig.IC1Filter 			= 0;
	sConfig.IC2Polarity 		= TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection 		= TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler 		= TIM_ICPSC_DIV1;
	sConfig.IC2Filter 			= 0;

	if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger 	= TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode 		= TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
}

static int InitSamplingTimer()
{
	__HAL_RCC_TIM9_CLK_ENABLE();
    SamplingTimer.Init.Prescaler 		= 49; // 5 kHz = 100E6/((250)*(80)*(1))
    SamplingTimer.Init.CounterMode 		= TIM_COUNTERMODE_UP;
    SamplingTimer.Init.Period 			= 80; // 5 kHz = 100E6/((250)*(80)*(1))
    SamplingTimer.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;

    if(HAL_TIM_Base_Init(&SamplingTimer) != HAL_OK)
    	return HAL_ERROR;

    if(HAL_TIM_Base_Start_IT(&SamplingTimer) != HAL_OK)
    	return HAL_ERROR;

    return HAL_OK;
}

void TIM1_BRK_TIM9_IRQHandler(void) { HAL_TIM_IRQHandler(&SamplingTimer); }

static int timer9Divisor = 0;
static int timer9Count = 0;
static float deltaDegreesLast = 0.0f;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	deltaDegrees 	= ((float)(int32_t)TIM5->CNT * (360.0f / (6000.0f * 4.0f)));
	timeElapUs 		+= 40;

	timer9Count++;
	if(timer9Count >= 100)
	{
		degreesPsec = (deltaDegrees - deltaDegreesLast)/((40.0f/1000000.0f)*(float)timer9Count);
		deltaDegreesLast = deltaDegrees;
		timer9Count = 0;
	}

	timer9Divisor++;
	if(timer9Divisor >= 25)
	{
		timeElapMs 		+= 1;
		timer9Divisor 	= 0;
//		int i 			= 0;
//		uint8_t* pU8 	= (uint8_t*) &datMsg.dat[0];
//		datMsg.start 	= 0xAB;
//		datMsg.dat[0] 	= (int32_t)(deltaDegrees*10000.0f); // deg
//		datMsg.dat[1] 	= (uint32_t)timeElapUs;
//
//		datMsg.cksum 	= 0;
//		datMsg.cksum 	+= datMsg.start;
//		datMsg.cksum 	+= pU8[i++]; // Deg
//		datMsg.cksum 	+= pU8[i++];
//		datMsg.cksum 	+= pU8[i++];
//		datMsg.cksum 	+= pU8[i++];
//		datMsg.cksum 	+= pU8[i++]; // Time
//		datMsg.cksum 	+= pU8[i++];
//		datMsg.cksum 	+= pU8[i++];
//		datMsg.cksum 	+= pU8[i++];
//		datMsg.cksum 	= 256 - datMsg.cksum;
//
//		uartBuffer 		= (uint8_t*)&datMsg;
//		UartReady 		= RESET;
//		HAL_UART_Transmit_IT(&huart1, uartBuffer, sizeof(struct DataMsg));

		if(timeElapMs % 500 == 0) WritePin(LED_PORT, LED_C, !ReadPin(LED_PORT, LED_C));
	}
}

/*
 * Need to re-initialize clocks to use HSI for PLL and
 * use PLL as the system clock with proper configs.
 *
 * This does not get set properly initially since after reset
 * needs to use HSI as system clock
 */
void setHighSystemClk(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct;

	// Use HSI and activate PLL with HSI as source.
	// This is tuned for NUCLEO-F411; update it for your board.
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;

	// 16 is the average calibration value, adjust for your own board.
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;

	// This assumes the HSI_VALUE is a multiple of 1 MHz. If this is not
	// your case, you have to recompute these PLL constants.
	RCC_OscInitStruct.PLL.PLLM = (HSI_VALUE/1000000u);
	RCC_OscInitStruct.PLL.PLLN = 400; // for 100 MHz
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; /* 84 MHz, conservative */
	RCC_OscInitStruct.PLL.PLLQ = 7; /* To make USB work. */
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	// Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
	// clocks dividers
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
	  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

	// This is expected to work for most large cores.
	// Check and update it for your own configuration.
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
