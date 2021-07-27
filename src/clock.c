

#include "clock.h"
#include "adc.h"

// Background timer for keeping time (25kHz)
static TIM_HandleTypeDef SamplingTimer 	= { .Instance = TIM9 };
static volatile uint32_t timeElapUs 	= 0;
static volatile uint32_t timeElapUsLast	= 0;
static volatile uint32_t timeElapMs 	= 0;

int InitSamplingTimer()
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

uint32_t Clock_GetMs(void)
{
	return timeElapMs;
}

uint32_t Clock_GetUs(void)
{
	return timeElapUs;
}

void Clock_StartTimer(ClockTimer *ct, uint32_t periodMs)
{
	ct->timeRemaining 	= periodMs;
	ct->timeMsLast 		= timeElapMs;
	ct->timerActive 	= 1;
}

void Clock_StartTimerUs(ClockTimerus *ct, uint32_t periodUs)
{
	ct->timeRemaining 	= periodUs;
	ct->timeUsLast 		= timeElapUs;
	ct->timerActive 	= 1;
}

int Clock_UpdateTimer(ClockTimer *ct)
{
	int rc = 0;
	if((timeElapMs - ct->timeMsLast) >= ct->timeRemaining)
	{
		ct->timeMsLast = Clock_GetMs();
		rc = 1;
	}
	return rc;
}

int Clock_UpdateTimerUs(ClockTimerus *ct)
{
	int rc = 0;
	if((timeElapUs - ct->timeUsLast) >= ct->timeRemaining)
	{
		ct->timeUsLast = Clock_GetUs();
		rc = 1;
	}
	return rc;
}

static int timer9Divisor = 0;
static int timer9Count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	timeElapUs 		+= 40;
	timer9Count++;

	hadc1.Instance->CR2 |= ADC_CR2_SWSTART;

	timer9Divisor++;
	if(timer9Divisor >= 25)
	{
		// 1000 us tick
		timeElapMs 		+= 1;
		timer9Divisor 	= 0;
	}
}
