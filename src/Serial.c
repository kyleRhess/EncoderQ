#include <string.h>
#include <stdio.h>
#include "stm32f4xx.h"
#include "Serial.h"
#include "system.h"
#include "characters.h"
#include "hcms.h"


volatile uint8_t toggle = 0;

static int errorSet = 0;
UART_HandleTypeDef s_UARTHandle = { .Instance = USART1 };

__IO ITStatus UartReady				= SET;
__IO ITStatus UartRxCmdReady 		= RESET;

int InitSerial(uint32_t baudrate, uint32_t stopbits, uint32_t datasize, uint32_t parity)
{
	int rc = HAL_OK;

	s_UARTHandle.Init.BaudRate   	= baudrate;
	s_UARTHandle.Init.WordLength 	= datasize;
	s_UARTHandle.Init.StopBits   	= stopbits;
	s_UARTHandle.Init.Parity     	= parity;
	s_UARTHandle.Init.Mode       	= USART_MODE_TX_RX;
	s_UARTHandle.Init.HwFlowCtl 	= UART_HWCONTROL_NONE;
	s_UARTHandle.Init.OverSampling 	= UART_OVERSAMPLING_16;
	rc = HAL_UART_Init(&s_UARTHandle);

	datMsg.msgCnt 	= 0;
	serialODR 		= 50;
	rxIndexA 		= 0;
	rxIndexB 		= 0;
	rxBufferSwitch  = 0;
	connLoss 		= 0;
	firstSync 		= 1;
	handshakeCMD 	= 0;
	serialMSG 		= 0;

	return rc;
}

void transmitSerialData()
{
	// Test data
	float arrt[7] =
	{
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f,
		0.0f
	};

	static uint32_t chksum32 = 0;
	chksum32 = 0;
	datMsg.start = SERIAL_MSG_START;
	chksum32 += datMsg.start;
	for (int i = 0; i < 7; ++i)
	{
		memcpy(&datMsg.dat[i], &arrt[i], 4);
		chksum32 += (datMsg.dat[i] & 0xff000000) >> 24;
		chksum32 += (datMsg.dat[i] & 0x00ff0000) >> 16;
		chksum32 += (datMsg.dat[i] & 0x0000ff00) >> 8;
		chksum32 += (datMsg.dat[i] & 0x000000ff) >> 0;
	}
	if(datMsg.msgCnt >= 255)
		datMsg.msgCnt = 0;
	else
		datMsg.msgCnt++;

	chksum32 += datMsg.msgCnt;

	datMsg.statusA = datMsg.msgCnt;
	datMsg.statusB = 0;

	chksum32 += datMsg.statusA;
	chksum32 += datMsg.statusB;

	datMsg.cksum = (0xff - (uint8_t)(chksum32 & 0x000000ff));

	uartBuffer = (uint8_t*)&datMsg;
	HAL_UART_Transmit_IT(&s_UARTHandle, uartBuffer, sizeof(struct DataMsg));
}

float toFloat(uint8_t bytes[], int startI)
{
	uint32_t rcUint = (uint32_t)((bytes[3+startI] << 24) |
								 (bytes[2+startI] << 16) |
								 (bytes[1+startI] << 8)	 |
								 (bytes[0+startI] << 0));
	float rcFloat = *((float*)&rcUint);
	return rcFloat;
}

uint8_t calcCRC(uint8_t datArr[], size_t size)
{
	uint32_t crc = 0;
	for(size_t i = 0; i < size-1; i++)
		crc += datArr[i];
	crc = (0xff - (crc & 0x000000ff));
	return (uint8_t)crc;
}






/*
 *
 */
//void USART1_IRQHandler(void)
//{
//	if(s_UARTHandle.gState != HAL_UART_STATE_BUSY_TX)
//	{
//		if(rxBufferSwitch == 0)
//		{
//			// Copy ISR buffer into RX buffer
//			uartRxA[rxIndexA++] = (uint8_t)(s_UARTHandle.Instance->DR & (uint8_t)0x00FF);
//
//			if(rxIndexA >= TX_BUFF_SZ)
//			{
//				memcpy(&uartTx, &uartRxA, TX_BUFF_SZ);
//				HAL_UART_Transmit_IT(&s_UARTHandle, uartTx, TX_BUFF_SZ);
//				rxIndexA = 0;
//				rxBufferSwitch = 1;
//			}
//		}
//		else
//		{
//			// Copy ISR buffer into RX buffer
//			uartRxB[rxIndexB++] = (uint8_t)(s_UARTHandle.Instance->DR & (uint8_t)0x00FF);
//
//			if(rxIndexB >= TX_BUFF_SZ)
//			{
//				memcpy(&uartTx, &uartRxB, TX_BUFF_SZ);
//				HAL_UART_Transmit_IT(&s_UARTHandle, uartTx, TX_BUFF_SZ);
//				rxIndexB = 0;
//				rxBufferSwitch = 0;
//			}
//		}
//
//		add_characters((uint8_t)(s_UARTHandle.Instance->DR & (uint8_t)0x00FF), 1);
//		update_display();
//
//		// Clear out the rx uart register
//		__HAL_UART_FLUSH_DRREGISTER(&s_UARTHandle);
//		__HAL_UART_CLEAR_FEFLAG(&s_UARTHandle);
//	}
//
//	UartRxCmdReady = RESET;
//
//	HAL_UART_IRQHandler(&s_UARTHandle);
//	return;
//}

//void HAL_UART_MspInit(UART_HandleTypeDef* huart)
//{
//	if(huart->Instance == USART1)
//	{
//		GPIO_InitTypeDef GPIO_InitStructureUart = {0};
//
//		// Setup all our peripherals
//		__HAL_RCC_USART1_CLK_ENABLE();
//		__HAL_RCC_GPIOB_CLK_ENABLE();
//
//		GPIO_InitStructureUart.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//		GPIO_InitStructureUart.Mode = GPIO_MODE_AF_PP;
//		GPIO_InitStructureUart.Alternate = GPIO_AF7_USART1;
//		GPIO_InitStructureUart.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//		GPIO_InitStructureUart.Pull = GPIO_PULLUP;
//		HAL_GPIO_Init(GPIOB, &GPIO_InitStructureUart);
//
//		// Enable the UART interrupt
//		__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
//
//		HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
//		HAL_NVIC_EnableIRQ(USART1_IRQn);
//	}
//}
