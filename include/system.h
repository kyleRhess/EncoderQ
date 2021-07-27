#ifndef SYSTEM_H_ /* include guard */
#define SYSTEM_H_

#define LED_A 				GPIO_PIN_12
#define LED_B 				GPIO_PIN_13
#define LED_C 				GPIO_PIN_14
#define LED_D 				GPIO_PIN_15
#define LED_PORT 			GPIOC

#define UART_PORT			GPIOB
#define UART_TX_PIN			GPIO_PIN_6
#define UART_RX_PIN			GPIO_PIN_7

#define ADC_PORT			GPIOA
#define ADC_THROTTLE		GPIO_PIN_5

#define POWER_SW_PORT		GPIOA
#define POWER_SW_PIN		GPIO_PIN_7

#include "stm32f4xx_hal.h"
#include "signal.h"

void WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
uint8_t ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

float mapVal(float x, float in_min, float in_max, float out_min, float out_max);


#endif /* SYSTEM_H_ */
