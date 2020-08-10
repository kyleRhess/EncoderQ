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
#include "PWM.h"
#include "diag/Trace.h"
#include "cmsis_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

TIM_HandleTypeDef htim5;
UART_HandleTypeDef huart1;
__IO ITStatus UartReady					= SET;
static TIM_HandleTypeDef SamplingTimer 	= { .Instance = TIM9 };
static float deltaDegrees	 			= 0.0f;
static float degreesPsec	 			= 0.0f;
static uint32_t timeElapUs 				= 0;
static uint32_t timeElapMs 				= 0;


PWM_Out PWMtimer;

struct DataMsg {
	uint8_t start;
	int32_t dat[2];
	uint8_t cksum;
} __attribute__((__packed__));

struct DataMsg datMsg;
uint8_t *uartBuffer;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

static int InitSamplingTimer(void);
static void WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ----- main() ---------------------------------------------------------------

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
	InitSamplingTimer();
	MX_TIM5_Init();
	MX_USART1_UART_Init();

	InitPWMOutput();

	HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn,0,0);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

	RCC->AHB1ENR 	= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR 	|= RCC_AHB1ENR_GPIOAEN;

	MX_GPIO_Init();

	GPIO_InitTypeDef gDataPin;
	gDataPin.Pin 	= GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	gDataPin.Mode 	= GPIO_MODE_OUTPUT_PP;
	gDataPin.Pull 	= GPIO_PULLUP;
	gDataPin.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOC, &gDataPin);

	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

	HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_1);

//	PWM_adjust_DutyCycle(&PWMtimer.timer, TIM_CHANNEL_1, 10.1f);

	while (1)
	{
//		PWM_adjust_DutyCycle(&PWMtimer.timer, TIM_CHANNEL_1, (deltaDegrees/360.0f)*100.0f);

//		PWM_adjust_PulseWidth(&PWMtimer.timer, TIM_CHANNEL_1, 0.03f);

		PWM_adjust_PulseWidth(&PWMtimer.timer, TIM_CHANNEL_1, htim5.Instance->CNT*(PULSE_NS_PER_CNT/1000.0f));

//		PWM_adjust_DutyCycle(&PWMtimer.timer, TIM_CHANNEL_1, degreesPsec);

		// Set LED values
		uint32_t bigNum = ((htim5.Instance->CNT / 4) % 4) + 1;
		switch (bigNum)
		{
			case 1:
				WritePin(GPIOC, GPIO_PIN_12, 1);
				WritePin(GPIOC, GPIO_PIN_13, 0);
				WritePin(GPIOC, GPIO_PIN_14, 0);
				WritePin(GPIOC, GPIO_PIN_15, 0);
				break;
			case 2:
				WritePin(GPIOC, GPIO_PIN_12, 0);
				WritePin(GPIOC, GPIO_PIN_13, 1);
				WritePin(GPIOC, GPIO_PIN_14, 0);
				WritePin(GPIOC, GPIO_PIN_15, 0);
				break;
			case 3:
				WritePin(GPIOC, GPIO_PIN_12, 0);
				WritePin(GPIOC, GPIO_PIN_13, 0);
				WritePin(GPIOC, GPIO_PIN_14, 1);
				WritePin(GPIOC, GPIO_PIN_15, 0);
				break;
			case 4:
				WritePin(GPIOC, GPIO_PIN_12, 0);
				WritePin(GPIOC, GPIO_PIN_13, 0);
				WritePin(GPIOC, GPIO_PIN_14, 0);
				WritePin(GPIOC, GPIO_PIN_15, 1);
				break;
			default:
				break;
		}
	}
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

static void WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	if(PinState != GPIO_PIN_RESET)
		GPIOx->BSRR = GPIO_Pin;
	else
		GPIOx->BSRR = (uint32_t)GPIO_Pin << 16U;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
	huart1.Instance 			= USART1;
	huart1.Init.BaudRate 		= 115200;
	huart1.Init.WordLength 		= UART_WORDLENGTH_8B;
	huart1.Init.StopBits 		= UART_STOPBITS_1;
	huart1.Init.Parity 			= UART_PARITY_NONE;
	huart1.Init.Mode 			= UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
	huart1.Init.OverSampling 	= UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart1) != HAL_OK)
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == USART1)
		UartReady = SET;
}

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }

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
		int i 			= 0;
		uint8_t* pU8 	= (uint8_t*) &datMsg.dat[0];
		datMsg.start 	= 0xAB;
		datMsg.dat[0] 	= (int32_t)(deltaDegrees*10000.0f); // deg
		datMsg.dat[1] 	= (uint32_t)timeElapUs;

		datMsg.cksum 	= 0;
		datMsg.cksum 	+= datMsg.start;
		datMsg.cksum 	+= pU8[i++]; // Deg
		datMsg.cksum 	+= pU8[i++];
		datMsg.cksum 	+= pU8[i++];
		datMsg.cksum 	+= pU8[i++];
		datMsg.cksum 	+= pU8[i++]; // Time
		datMsg.cksum 	+= pU8[i++];
		datMsg.cksum 	+= pU8[i++];
		datMsg.cksum 	+= pU8[i++];
		datMsg.cksum 	= 256 - datMsg.cksum;

		uartBuffer 		= (uint8_t*)&datMsg;
		UartReady 		= RESET;
		HAL_UART_Transmit_IT(&huart1, uartBuffer, sizeof(struct DataMsg));
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
