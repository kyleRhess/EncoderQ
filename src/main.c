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
#include "PID.h"

TIM_HandleTypeDef q_time;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

#define FULL_RX 64
#define HALF_RX FULL_RX / 2
uint8_t dma_buffer_tx[2048];
uint8_t dma_buffer_rx[2048];

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

static TIM_HandleTypeDef SamplingTimer 	= { .Instance = TIM9 };
static float deltaDegrees	 			= 0.0f;
static float degreesPsec	 			= 0.0f;
static uint32_t timeElapUs 				= 0;

#define ADC_BUF_LEN 1
volatile uint32_t g_ADCValue = 0;
uint32_t g_ADCBuffer[ADC_BUF_LEN];
volatile int m_bRunCurrentLoop = 0;
PWM_Out PWMtimer;
static float sineMsA = 0.0f;
static float sineMsB = 0.0f;
static float sineMsC = 0.0f;

static PID_Controller pi_axis_d;
static PID_Controller pi_axis_q;



static int InitSamplingTimer(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void PWM_Monitor(float a, float b, float c);

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
	//MX_TIM5_Init();

//	InitSerial(115200, UART_STOPBITS_1, UART_WORDLENGTH_8B, UART_PARITY_NONE);


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

#if 0
	// OUT_Z pin setup
	GPIO_InitTypeDef gZPin;
	gZPin.Pin 		= Z_INTERRUPT;
	gZPin.Mode 		= GPIO_MODE_IT_RISING;
	gZPin.Pull 		= GPIO_NOPULL;
	gZPin.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOA, &gZPin);
#endif

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	init_display();


	HAL_TIM_Encoder_Start(&q_time, TIM_CHANNEL_ALL);

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


	MX_GPIO_Init();
	InitPWMOutput();
	MX_DMA_Init();
	MX_USART1_UART_Init();

	MX_ADC1_Init();

	HAL_StatusTypeDef hal_status = HAL_OK;
	for (int i = 0; i < 2000; ++i)
	{
		dma_buffer_tx[i] = 0;
		dma_buffer_rx[i] = 0;
	}

	hal_status = HAL_UART_Receive_DMA(&huart1, dma_buffer_rx, FULL_RX);

//	HAL_ADC_Start_DMA(&hadc1, g_ADCBuffer, ADC_BUF_LEN);

	PWM_adjust_DutyCycle(&PWMtimer.timer, TIM_CHANNEL_1, 0.50f);
	PWM_adjust_DutyCycle(&PWMtimer.timer, TIM_CHANNEL_2, 0.50f);
	PWM_adjust_DutyCycle(&PWMtimer.timer, TIM_CHANNEL_3, 0.50f);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	g_ADCBuffer[0] = HAL_ADC_GetValue(&hadc1);

	pi_axis_d.kP = 0.001f;
	pi_axis_d.kI = 0.0001f;
	pi_axis_d.kD = 0.0f;
	pi_axis_d.setPoint = 0.0f;
	pi_axis_d.deltaTime = (1.0f / 1000.0f);
	PID_Initialize(&pi_axis_d);
	pi_axis_q.kP = 0.001f;
	pi_axis_q.kI = 0.0001f;
	pi_axis_q.kD = 0.0f;
	pi_axis_q.setPoint = 20.0f;
	pi_axis_q.deltaTime = (1.0f / 1000.0f);
	PID_Initialize(&pi_axis_q);

	while (1)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		g_ADCBuffer[0] = HAL_ADC_GetValue(&hadc1);

#define PI			3.141592654f
#define TWOPI		6.283185307f
#define FOURPI		12.566370614f
#define SHIFT_120	2.094395102f
#define SHIFT_240 	4.188790205f

		static float freq = 10.0f;
		sineMsA = 0.5f + 0.5f*sinf(((freq*TWOPI*(float)timeElapUs)/1000000.0f));
		sineMsB = 0.5f + 0.5f*sinf(((freq*TWOPI*(float)timeElapUs)/1000000.0f) + SHIFT_120);
		sineMsC = 0.5f + 0.5f*sinf(((freq*TWOPI*(float)timeElapUs)/1000000.0f) + SHIFT_240);

#define _PI_3				1.047197551f
#define	SQRT_3				1.732050808f
#define SQR_THREE_TWO 		0.866025404f
#define ONE_THIRD			0.333333333f
#define ONE_SQR_THREE		0.577350269f
#define	V_SUPPLY			48.0f
#define	DEG_RAD				0.017453293f
		// fwd clark
		float i_a 			= 10.0f*sinf(((freq*TWOPI*(float)timeElapUs)/1000000.0f));
		float i_b 			= 10.0f*sinf(((freq*TWOPI*(float)timeElapUs)/1000000.0f) + SHIFT_120);
		float i_c 			= -i_a - i_b;

		float i_alpha 		= 1.5f * i_a;
		float i_beta 		= SQR_THREE_TWO*i_b - SQR_THREE_TWO*i_c;

		static float rotor_theta	= 0.0f;
		float i_d 			= i_alpha*cosf(rotor_theta) + i_beta*sinf(rotor_theta);
		float i_q 			= -i_alpha*sinf(rotor_theta) + i_beta*cosf(rotor_theta);


		if(m_bRunCurrentLoop)
		{
			rotor_theta += 0.2f;

			PID_Update(&pi_axis_d, i_d);
			PID_Update(&pi_axis_q, i_q);

			float v_d 		= PID_GetOutput(&pi_axis_d);
			float v_q 		= PID_GetOutput(&pi_axis_q);
			float v_alpha 	= v_d*cosf(rotor_theta) - v_q*sinf(rotor_theta);
			float v_beta 	= v_d*sinf(rotor_theta) + v_q*cosf(rotor_theta);
			float v_a 		= 0.66666667f * v_alpha;
			float v_b 		= -ONE_THIRD*v_alpha + ONE_SQR_THREE*v_beta;
			float v_c 		= -ONE_THIRD*v_alpha - ONE_SQR_THREE*v_beta;

			static float Uq = 0.0f;
			Uq += 0.001f;
			if(Uq >= 48.0f) Uq = 48.0f;

			int sector = ((int)rotor_theta / 60.0f) + 1;

			if(rotor_theta >= 360.0f)
			{
				rotor_theta = 0.0f;
				sector = 1;
			}

			float alpha = rotor_theta - (sector-1)*60.0f;

#define T	0.00002f
			float T1 = Uq/48.0f * sinf((60.0f - alpha) * DEG_RAD);
			float T2 = Uq/48.0f * sinf(alpha * DEG_RAD);
			float T0 = 1 - T1 - T2;

			float Ta,Tb,Tc;
			switch(sector)
			{
				case 1:
					Ta = T1 + T2 + T0/2;
					Tb = T2 + T0/2;
					Tc = T0/2;
					break;
				case 2:
					Ta = T1 +  T0/2;
					Tb = T1 + T2 + T0/2;
					Tc = T0/2;
					break;
				case 3:
					Ta = T0/2;
					Tb = T1 + T2 + T0/2;
					Tc = T2 + T0/2;
					break;
				case 4:
					Ta = T0/2;
					Tb = T1+ T0/2;
					Tc = T1 + T2 + T0/2;
					break;
				case 5:
					Ta = T2 + T0/2;
					Tb = T0/2;
					Tc = T1 + T2 + T0/2;
					break;
				case 6:
					Ta = T1 + T2 + T0/2;
					Tb = T0/2;
					Tc = T1 + T0/2;
					break;
				default:
					// possible error state
					Ta = 0;
					Tb = 0;
					Tc = 0;
			}


			m_bRunCurrentLoop = 0;
			PWM_Monitor(Ta, Tb, Tc);
		}
	}
}

static void PWM_Monitor(float a, float b, float c)
{
	static float pwm_duty = 1.0f;
	PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_1, a);
	PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_2, b);
	PWM_Set_Duty(&PWMtimer.timer, TIM_CHANNEL_3, c);
}

/**
* @brief This function handles EXTI line0.
*/
void EXTI9_5_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(Z_INTERRUPT);
//	WritePin(LED_PORT, LED_D, !ReadPin(LED_PORT, LED_D));
//	q_time.Instance->CNT = 0;
}

/*
 * Timer used for motor ESC control
 */
int InitPWMOutput()
{
	PWMtimer.numChannels 	= 3;
	PWMtimer.frequency 		= PWM_FREQ;
	PWMtimer.TIM 			= TIM1;
	PWMtimer.timer 			= Initialize_PWM(&PWMtimer);

	return HAL_OK;
}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_TIM5_Init(void)
//{
//	TIM_Encoder_InitTypeDef sConfig = {0};
//	TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//	q_time.Instance 			= TIM5;
//	q_time.Init.Prescaler 		= 0;
//	q_time.Init.CounterMode 	= TIM_COUNTERMODE_UP;
//	q_time.Init.Period 			= 0xFFFFFFFF;
//	q_time.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;
//	sConfig.EncoderMode 		= TIM_ENCODERMODE_TI12;
//	sConfig.IC1Polarity 		= TIM_ICPOLARITY_RISING;
//	sConfig.IC1Selection 		= TIM_ICSELECTION_DIRECTTI;
//	sConfig.IC1Prescaler 		= TIM_ICPSC_DIV1;
//	sConfig.IC1Filter 			= 0;
//	sConfig.IC2Polarity 		= TIM_ICPOLARITY_RISING;
//	sConfig.IC2Selection 		= TIM_ICSELECTION_DIRECTTI;
//	sConfig.IC2Prescaler 		= TIM_ICPSC_DIV1;
//	sConfig.IC2Filter 			= 0;
//
//	if (HAL_TIM_Encoder_Init(&q_time, &sConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	sMasterConfig.MasterOutputTrigger 	= TIM_TRGO_RESET;
//	sMasterConfig.MasterSlaveMode 		= TIM_MASTERSLAVEMODE_DISABLE;
//
//	if (HAL_TIMEx_MasterConfigSynchronization(&q_time, &sMasterConfig) != HAL_OK)
//	{
//		Error_Handler();
//	}
//}

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

static int InitSamplingTimer()
{
	__HAL_RCC_TIM9_CLK_ENABLE();
    SamplingTimer.Init.Prescaler 		= 124; // 5 kHz = 100E6/((250)*(80)*(1))
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

static int timer9Divisor = 0;
static int timer9Count = 0;
static float deltaDegreesLast = 0.0f;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	timeElapUs 		+= 100;
	timer9Count++;

	timer9Divisor++;


	if(timer9Divisor >= 10)
	{
		timer9Divisor 	= 0;

		if(!m_bRunCurrentLoop)
			m_bRunCurrentLoop = 1;
	}

#if 0
	if(timer9Divisor >= 25)
	{
		// 1000 ms tick
		timeElapMs 		+= 1;
		timeElapMin		+= (0.001f/60.0f);
		timer9Divisor 	= 0;

//		deltaDegrees 	= ((float)(int32_t)q_time.Instance->CNT * (360.0f / (6000.0f * 4.0f)));
//		degreesPsec = (deltaDegrees - deltaDegreesLast)/0.1f;
//		deltaDegreesLast = deltaDegrees;


//		if(dma_rx_idx - dma_rx_idxLast > 0)
//		{
//			WritePin(leds[0].iPort, leds[0].iName, GPIO_PIN_SET);
//		}
//		else
//		{
//			WritePin(leds[0].iPort, leds[0].iName, GPIO_PIN_RESET);
//		}
//		if(dma_tx_idx - dma_tx_idxLast > 0)
//		{
//			WritePin(leds[1].iPort, leds[1].iName, GPIO_PIN_SET);
//		}
//		else
//		{
//			WritePin(leds[1].iPort, leds[1].iName, GPIO_PIN_RESET);
//		}
		uint32_t adc_sample = g_ADCValue;


		uint8_t pd[4] = {0x12, 0x34, 0x56, 0x78};
		memcpy(&pd, &adc_sample, 4);

		static char str[5] = {0, 0, 0, 0, 0};
		sprintf(str, "%.2f", (((float)adc_sample)/4096.0f)*3.3f);
		HAL_UART_Transmit_DMA(&huart1, str, 5);

//		static char str[5] = {0, 0, 0, 0, 0};
//		sprintf(str, "%.1f", deltaDegrees);
//		add_characters(str, 4);
//		update_display();
		//update_LEDs();
		// !ReadPin(LED_PORT, LED_A));
	}
#endif
}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  *   hdma_memtomem_dma2_stream1
  */
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

	/* DMA2_Stream7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

//	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 3, 0);
//	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_USART1_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}


/* USER CODE BEGIN 1 */
/*
 * UART Interrupts
 */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->hdmatx->ErrorCode == HAL_DMA_ERROR_FE)
	{
		// Since FIFO mode disabled, just ignore this error
		__NOP();
	}
	else
	{
		WritePin(leds[3].iPort, leds[3].iName, GPIO_PIN_SET);
		if(__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_ORE) != RESET)
		{
			if(__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_RXNE) == RESET)
				UartHandle->Instance->DR;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	memcpy(&dma_buffer_tx[0], &dma_buffer_rx[HALF_RX], HALF_RX);
	HAL_UART_Transmit_DMA(&huart1, &dma_buffer_tx[0], HALF_RX);

	HAL_UART_Receive_DMA(&huart1, &dma_buffer_rx[0], FULL_RX);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	memcpy(&dma_buffer_tx[0], &dma_buffer_rx[0], HALF_RX);
	HAL_UART_Transmit_DMA(&huart1, &dma_buffer_tx[0], HALF_RX);
}

void UART_DMATransmitCplt(DMA_HandleTypeDef *hdma)
{
	__NOP();
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	__NOP();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	__NOP();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
	  /* USER CODE BEGIN ADC1_Init 0 */

	  /* USER CODE END ADC1_Init 0 */

	  ADC_ChannelConfTypeDef sConfig = {0};

	  /* USER CODE BEGIN ADC1_Init 1 */

	  /* USER CODE END ADC1_Init 1 */
	  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	  */
	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc1.Init.ScanConvMode = DISABLE;
	  hadc1.Init.ContinuousConvMode = ENABLE;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.NbrOfConversion = 1;
	  hadc1.Init.DMAContinuousRequests = DISABLE;
	  hadc1.Init.EOCSelection = DISABLE;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	  */
	  sConfig.Channel = ADC_CHANNEL_0;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /* USER CODE BEGIN ADC1_Init 2 */

	  /* USER CODE END ADC1_Init 2 */
}


//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	g_ADCValue = 0;
//	for (int i = 0; i < ADC_BUF_LEN; ++i)
//	{
//		g_ADCValue += g_ADCBuffer[i];
//	}
//	g_ADCValue /= ADC_BUF_LEN;
//	adcBufIdx = 0;
//}






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
