#ifndef SYSTEM_H_ /* include guard */
#define SYSTEM_H_

#include "stm32f4xx_hal.h"



void WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
uint8_t ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

float mapVal(float x, float in_min, float in_max, float out_min, float out_max);



#endif /* SYSTEM_H_ */
