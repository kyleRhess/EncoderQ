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
#include "adc.h"
#include "SPI.h"
#include "clock.h"
#include "hcms.h"
#include "characters.h"
#include "serial.h"
#include "signal.h"
#include "diag/Trace.h"
#include "cmsis_device.h"

UART_HandleTypeDef huart1 = { .Instance = USART1 };
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

typedef struct _LED_Status
{
	GPIO_TypeDef* iPort;
	uint16_t iName;
	uint16_t iLEDCountdownOn;
	uint16_t iLEDCountdownOff;
	int iLEDCountdown;
	uint8_t iLEDState;
}LED_Status;
static LED_Status leds[4];

static int seconds = 0;
static ClockTimerus transmitTimer;
static ClockTimer secTimer;
static ClockTimer screenTimer;

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);

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

	HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
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

	// POWER_SW pin setup
	GPIO_InitTypeDef gPwrPin;
	gPwrPin.Pin 	= POWER_SW_PIN;
	gPwrPin.Mode 	= GPIO_MODE_INPUT;
	gPwrPin.Pull 	= GPIO_PULLDOWN;
	gPwrPin.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(POWER_SW_PORT, &gPwrPin);

	init_display();

	set_brightness(10.0f);

	for (int i = 0; i < 4; ++i)
	{
		leds[i].iPort = LED_PORT;

		switch (i) {
			case 0:
				leds[i].iName = LED_A;
				break;
			case 1:
				leds[i].iName = LED_B;
				break;
			case 2:
				leds[i].iName = LED_C;
				break;
			case 3:
				leds[i].iName = LED_D;
				break;
			default:
				break;
		}
		leds[i].iLEDState = GPIO_PIN_RESET;
		leds[i].iLEDCountdown = 0;
		leds[i].iLEDCountdownOff = 0;
		leds[i].iLEDCountdownOn = 0;
	}
	leds[3].iLEDCountdownOff = 950;
	leds[3].iLEDCountdownOn = 25;


	MX_DMA_Init();
	MX_GPIO_Init();

	// Setup serial interface
	Serial_InitPort(115200, UART_STOPBITS_1, UART_WORDLENGTH_8B, UART_PARITY_NONE);
	Serial_RxData(RX_BUFF_SZ);

	// Start ADC + DMA before base PWM timer
	ADC_Init();

	static uint32_t cmdSeq[2] = {
			CMD_MOTOR_HEARTBEAT,
			CMD_MOTOR_TORQUE
	};
	static uint32_t	nextCmd = CMD_MOTOR_RESET;

	Clock_StartTimer(&secTimer, 1000);
	Clock_StartTimer(&screenTimer, 8);
	Clock_StartTimerUs(&transmitTimer, 10000);

	while (1)
	{
		if(ReadPin(leds[3].iPort, leds[3].iName))
			WritePin(leds[3].iPort, leds[3].iName, GPIO_PIN_RESET);
		else
			WritePin(leds[3].iPort, leds[3].iName, GPIO_PIN_SET);

		static int m_iIsMotorEnabled = 0;
		if(HAL_GPIO_ReadPin(POWER_SW_PORT, POWER_SW_PIN))
		{
			if(!m_iIsMotorEnabled)
			{
				m_iIsMotorEnabled = 1;
				nextCmd = CMD_MOTOR_ENABLE;
				WritePin(leds[2].iPort, leds[2].iName, GPIO_PIN_SET);
			}
		}
		else
		{
			if(m_iIsMotorEnabled)
			{
				m_iIsMotorEnabled = 0;
				nextCmd = CMD_MOTOR_DISABLE;
				WritePin(leds[2].iPort, leds[2].iName, GPIO_PIN_RESET);
			}
		}

		if(Serial_GetResponse()->driveMode & MOTOR_MODE_CRUISING)
			WritePin(leds[0].iPort, leds[0].iName, GPIO_PIN_SET);


		if(Clock_UpdateTimer(&screenTimer))
		{
			char str[5] = {0, 0, 0, 0, 0};
			if(Serial_GetResponse()->driveMode & MOTOR_MODE_OVERCURRENT)
				sprintf(str, "%s", "Amps");
			else if(Serial_GetResponse()->driveMode & MOTOR_MODE_OVERSPEED)
				sprintf(str, "%s", "Sped");
			else if(Serial_GetResponse()->driveMode & MOTOR_MODE_NOHEART)
				sprintf(str, "%s", "HART");
			else
				sprintf(str, "%d", Serial_GetResponse()->millis/1000);

			add_characters(str, 4);
			update_display();
		}


		static uint16_t cmdToSend = 0;
		if(Clock_UpdateTimerUs(&transmitTimer))
		{
			if(1)//!Serial_WaitingForAck())
			{
				if(nextCmd == CMD_MOTOR_NONE)
				{
					Serial_SendCommand(cmdSeq[cmdToSend]);
					cmdToSend++;
					if(cmdToSend >= sizeof(cmdSeq)/sizeof(cmdSeq[0]))
						cmdToSend = 0;
				}
				else
				{
					Serial_SendCommand(nextCmd);
					nextCmd = CMD_MOTOR_NONE;
				}
			}
		}

		static float throtOffset = 0;
		static int throtOffsetCnt = 0;
		if(throtOffsetCnt < 100)
		{
			throtOffset += ADC_GetThrottle();
			throtOffsetCnt++;
		}
		else if(throtOffsetCnt != 999)
		{
			throtOffset = throtOffset/(float)throtOffsetCnt;
			throtOffsetCnt = 999;
		}

		Serial_GetTransmit()->torqueValue = ((ADC_GetThrottle()-throtOffset)/100.0f) * 10.5f;
//		Serial_GetTransmit()->speedValue = (ADC_GetThrottle()/100.0f) * 200.0f;

		if(Clock_UpdateTimer(&secTimer))
		{
			seconds++;
			if(m_iIsMotorEnabled)
			{
				nextCmd = CMD_MOTOR_ENABLE;
			}
		}

#if 0
		if(Clock_UpdateTimer(&secTimer))
		{
			seconds++;
		}

		if(seconds == 5)
		{
			nextCmd = CMD_MOTOR_ENABLE;
			seconds++;
		}

		if(seconds == 7)
		{
//			Serial_GetTransmit()->torqueValue = 5.5f;
			nextCmd = CMD_MOTOR_CRUISE;
			seconds++;
		}

		if(seconds == 12)
		{
//			nextCmd = CMD_MOTOR_CRUISE;
			seconds++;
		}

		if(seconds == 30)
		{
//			nextCmd = CMD_MOTOR_CRUISE;
			seconds++;
		}

		if(seconds == 40)
		{
			nextCmd = CMD_MOTOR_DISABLE;
			seconds++;
		}
#endif

//		memset(str, 0, 4);
//		sprintf(str, "%04X", (unsigned int)q_time.Instance->CNT);
//		add_characters(str, 4);
//		update_display();
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
}

static void MX_DMA_Init(void)
{
	__HAL_RCC_DMA2_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void update_LEDs( void )
{
	for (int i = 0; i < 4; i++)
	{
		if(ReadPin(leds[i].iPort, leds[i].iName) && leds[i].iLEDCountdownOff > 0)
		{
			if(leds[i].iLEDCountdown <= 0)
			{
				WritePin(leds[i].iPort, leds[i].iName, GPIO_PIN_RESET);
				leds[i].iLEDCountdown = leds[i].iLEDCountdownOff;
			}
		}
		else
		{
			if(leds[i].iLEDCountdown <= 0 && leds[i].iLEDCountdownOn > 0)
			{
				WritePin(leds[i].iPort, leds[i].iName, GPIO_PIN_SET);
				leds[i].iLEDCountdown = leds[i].iLEDCountdownOn;
			}
		}
		leds[i].iLEDCountdown -= 1;
	}
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/




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
