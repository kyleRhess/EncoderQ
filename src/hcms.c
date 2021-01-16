#include <string.h>
#include "system.h"
#include "SPI.h"
#include "hcms.h"
#include "characters.h"

uint8_t all_chars[256][5];
GPIO_InitTypeDef gResetPin;
GPIO_InitTypeDef gRSPin;
GPIO_InitTypeDef gBlankPin;

static float lookuparr[64] = {
	0.0000f,0.0000f,0.0000f,0.0000f,0.5270f,0.8500f,1.0230f,1.2410f,
	1.5500f,1.6500f,1.7000f,2.0770f,2.4090f,2.5000f,2.5730f,3.3000f,
	3.3500f,3.6270f,3.6500f,4.1500f,4.6500f,4.8910f,5.0000f,5.5800f,
	5.8500f,6.0590f,6.7000f,7.1300f,7.5000f,8.3000f,8.5410f,9.0000f,
	9.3000f,10.9500f,11.4700f,11.5000f,11.7000f,13.1400f,14.5700f,
	15.0000f,15.0000f,16.7900f,18.0000f,18.5000f,18.6000f,21.9000f,
	23.0000f,23.5000f,24.8000f,27.0100f,30.0000f,30.0000f,31.0000f,
	34.3100f,37.0000f,40.0000f,43.8000f,47.0000f,50.0000f,58.4000f,
	60.0000f,73.0000f,80.0000f,100.0000f
};

static uint8_t peak_arr[64] = {
	0x20,0x10,0x00,0x30,0x20,0x10,0x20,0x00,
	0x20,0x10,0x30,0x20,0x00,0x10,0x20,0x30,
	0x10,0x20,0x00,0x10,0x20,0x00,0x30,0x20,
	0x10,0x00,0x30,0x20,0x10,0x30,0x00,0x10,
	0x20,0x00,0x20,0x10,0x30,0x00,0x20,0x10,
	0x30,0x00,0x30,0x10,0x20,0x00,0x30,0x10,
	0x20,0x00,0x10,0x30,0x20,0x00,0x30,0x10,
	0x00,0x30,0x10,0x00,0x30,0x00,0x30,0x30
};

static uint8_t pwm_arr[64] = {
	0x00,0x00,0x00,0x00,0x01,0x01,0x02,0x01,
	0x03,0x02,0x01,0x04,0x02,0x03,0x05,0x02,
	0x04,0x06,0x03,0x05,0x07,0x04,0x03,0x08,
	0x06,0x05,0x04,0x09,0x07,0x05,0x06,0x08,
	0x0A,0x07,0x0B,0x09,0x06,0x08,0x0C,0x0A,
	0x07,0x09,0x08,0x0B,0x0D,0x0A,0x09,0x0C,
	0x0E,0x0B,0x0D,0x0A,0x0F,0x0C,0x0B,0x0E,
	0x0D,0x0C,0x0F,0x0E,0x0D,0x0F,0x0E,0x0F
};

static SPI_Bus SPI_Bus_1;

static int lookup(float target, float *arr, int idx, int size)
{
	size /= 2;
	if(target == arr[idx] || size == 1)
	{
		return idx;
	}
	else if(target < arr[idx])
	{
		return lookup(target, arr, idx - (size/2), size);
	}
	else if(target > arr[idx])
	{
		return lookup(target, arr, idx + (size/2), size);
	}
	return 0;
}

void init_reset_pin()
{
	// Reset pin setup
	gResetPin.Pin 		= GPIO_PIN_1;
	gResetPin.Mode 		= GPIO_MODE_OUTPUT_PP;
	gResetPin.Pull 		= GPIO_PULLDOWN;
	gResetPin.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &gResetPin);
	WritePin(GPIOB, GPIO_PIN_1, 0);
	WritePin(GPIOB, GPIO_PIN_1, 1);
}

void init_reg_pin()
{
	// Register select pin setup
	gRSPin.Pin 			= GPIO_PIN_8;
	gRSPin.Mode 		= GPIO_MODE_OUTPUT_PP;
	gRSPin.Pull 		= GPIO_PULLUP;
	gRSPin.Speed 		= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &gRSPin);
	WritePin(GPIOB, GPIO_PIN_8, 0);
}

void init_blank_pin()
{
	// Blank pin setup
	gBlankPin.Pin 		= GPIO_PIN_0;
	gBlankPin.Mode 		= GPIO_MODE_OUTPUT_PP;
	gBlankPin.Pull 		= GPIO_PULLDOWN;
	gBlankPin.Speed 	= GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &gBlankPin);
	WritePin(GPIOB, GPIO_PIN_0, 0);
}

void init_display()
{
	init_reset_pin();
	init_reg_pin();
	init_blank_pin();

	// Setup SPI port
	SPI_Initialize(&SPI_Bus_1, SPI1, SPI_BAUDRATEPRESCALER_32, SPI_FIRSTBIT_MSB, SPI_POLARITY_LOW);
	SPI_Initialize_CS(SPI1_CS_PORT, SPI1_CS0);
	HAL_NVIC_SetPriority(SPI1_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(SPI1_IRQn);

	set_brightness(10.0f);
}

void update_display()
{
	memcpy(&spi_txBuff[1+0], &all_chars[(uint8_t)character_buff[0]][0], 5);
	memcpy(&spi_txBuff[1+5], &all_chars[(uint8_t)character_buff[1]][0], 5);
	memcpy(&spi_txBuff[1+5+5], &all_chars[(uint8_t)character_buff[2]][0], 5);
	memcpy(&spi_txBuff[1+5+5+5], &all_chars[(uint8_t)character_buff[3]][0], 5);

	SPI_SendReceive(&SPI_Bus_1, SPI1_CS_PORT, SPI1_CS0, &spi_txBuff[1], &spi_rxBuff[1], 20);

	WritePin(GPIOB, GPIO_PIN_8, 1);
	SPI_SendReceive(&SPI_Bus_1, SPI1_CS_PORT, SPI1_CS0, &spi_txBuff[0], &spi_rxBuff[0], 1);
	WritePin(GPIOB, GPIO_PIN_8, 0);
}

void set_brightness(float percent)
{
	int map_idx = 0;
	map_idx = lookup(percent, lookuparr, (sizeof(lookuparr)/sizeof(lookuparr[0]))/2, sizeof(lookuparr)/sizeof(lookuparr[0]));
	spi_txBuff[0] = CTRL_WORD_0_MASK & (NORM_MODE_MASK | peak_arr[map_idx] | pwm_arr[map_idx]);
}


