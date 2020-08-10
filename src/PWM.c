
#include "PWM.h"

TIM_HandleTypeDef timer_PWM;

TIM_HandleTypeDef Initialize_PWM(PWM_Out * PWMType)
{
	TIM_TypeDef * TIM 	= PWMType->TIM;
	uint16_t nChannels 	= PWMType->numChannels;

	TIM_MasterConfigTypeDef master;
	TIM_OC_InitTypeDef configOC;

	memset(&master, 0, sizeof(master));
	memset(&configOC, 0, sizeof(configOC));
	memset(&timer_PWM, 0, sizeof(timer_PWM));

	timer_PWM.Instance 				= TIM;
	timer_PWM.Init.Prescaler 		= ((CLOCK_CYCLES_PER_SECOND / COUNTERFREQ) - 1);
	timer_PWM.Init.CounterMode 		= TIM_COUNTERMODE_UP;
	timer_PWM.Init.Period 			= PWM_STEPS - 1; // ARR -> counter max
	timer_PWM.Init.ClockDivision 	= TIM_CLOCKDIVISION_DIV1;

	HAL_TIM_PWM_Init(&timer_PWM);

	master.MasterOutputTrigger 		= TIM_TRGO_RESET;
	master.MasterSlaveMode 			= TIM_MASTERSLAVEMODE_DISABLE;

	configOC.OCMode 				= TIM_OCMODE_PWM1;
	configOC.Pulse 					= 0;
	configOC.OCPolarity 			= TIM_OCPOLARITY_HIGH;

	if(nChannels <= 1)
	{
		HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_1);
	}
	else if(nChannels <= 2)
	{
		HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_1);
		HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_2);
	}
	else if(nChannels <= 3)
	{
		HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_1);
		HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_2);
		HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_3);
	}
	else if(nChannels <= 4)
	{
		HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_1);
		HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_2);
		HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_3);
		HAL_TIM_PWM_ConfigChannel(&timer_PWM, &configOC, TIM_CHANNEL_4);
		HAL_TIM_PWM_Start(&timer_PWM, TIM_CHANNEL_4);
	}

	return timer_PWM;
}




void PWM_adjust_DutyCycle(TIM_HandleTypeDef * pwmHandle, uint32_t Channel, float dutyCycle)
{
	if(dutyCycle > 100.0f)
		dutyCycle = 100.0f;
	if(dutyCycle < 0.0f)
		dutyCycle = 0.0f;

	float tempPulseWidth_us = ((float)(1000000/PWM_FREQ))*(dutyCycle/100.0f);

	PWM_adjust_PulseWidth(pwmHandle, Channel, tempPulseWidth_us);
}



void PWM_adjust_PulseWidth(TIM_HandleTypeDef * pwmHandle, uint32_t Channel, float pulseWidth_us)
{
	uint32_t counts_Ccr = CNTS_FROM_US(pulseWidth_us);

	if(counts_Ccr > pwmHandle->Instance->ARR)
	{
		counts_Ccr = pwmHandle->Instance->ARR;
	}

    /*Assign the new DC count to the capture compare register.*/
    switch(Channel)
    {
		case TIM_CHANNEL_1:
			pwmHandle->Instance->CCR1 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_2:
			pwmHandle->Instance->CCR2 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_3:
			pwmHandle->Instance->CCR3 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_4:
			pwmHandle->Instance->CCR4 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_ALL:
			pwmHandle->Instance->CCR1 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR2 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR3 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR4 = counts_Ccr;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

    }
}


void PWM_adjust_Frequency(TIM_HandleTypeDef * pwmHandle, uint32_t Channel, uint32_t newFreq)
{
	uint32_t period_cycles = CLOCK_CYCLES_PER_SECOND / newFreq;
	uint16_t prescaler = (uint16_t)(period_cycles / MAX_RELOAD + 1);
	uint16_t overflow = (uint16_t)((period_cycles + (prescaler / 2)) / prescaler);
	uint16_t duty = (uint16_t)(overflow / 2);

	pwmHandle->Instance->ARR = (uint32_t)overflow;
	pwmHandle->Instance->PSC = (uint32_t)prescaler;

	switch(Channel)
	{
		case TIM_CHANNEL_1:
			pwmHandle->Instance->CCR1 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_2:
			pwmHandle->Instance->CCR2 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_3:
			pwmHandle->Instance->CCR3 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_4:
			pwmHandle->Instance->CCR4 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

		case TIM_CHANNEL_ALL:
			pwmHandle->Instance->CCR1 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR2 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR3 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			pwmHandle->Instance->CCR4 = (uint32_t)duty;  /*Change CCR1 to appropriate channel, or pass it in with function.*/
			break;

	}
}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_pwm->Instance==TIM1)
  {
    /* Peripheral clock enable */
    __TIM1_CLK_ENABLE();
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  }
}
