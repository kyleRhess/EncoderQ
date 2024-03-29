#ifndef SERIAL_H_ /* include guard */
#define SERIAL_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "system.h"

#define MSG_RATE_HZ(xxx) 		(SAMPLE_RATE / xxx)

#define RX_BUFF_SZ 				42
#define TX_BUFF_SZ 				42

#define START_CHAR 				0xAA

#define CMD_SET_OUT_DAT 		0x01
#define CMD_SET_OUT_RATE 		0x02
#define CMD_SET_PID_GAIN 		0x03

#define CMD_MOTOR_NONE			0x00000000
#define CMD_MOTOR_ENABLE		0x00000001
#define CMD_MOTOR_DISABLE		0x00000002
#define CMD_MOTOR_TORQUE		0x00000004
#define CMD_MOTOR_MODE			0x00000008
#define CMD_MOTOR_QUERY			0x00000010
#define CMD_MOTOR_RESET			0x00000020
#define CMD_MOTOR_POSITION		0x00000040
#define CMD_MOTOR_PARMS			0x00000080
#define CMD_MOTOR_HOME			0x00000100
#define CMD_MOTOR_HEARTBEAT		0x00000200
#define CMD_MOTOR_CRUISE		0x00000400
#define CMD_MOTOR_SPEED			0x00000800


typedef struct DataFields
{
	float		torqueValue; 	// 1
	float		speedValue;		// 2
	float		posValue;		// 3
	float		motorCurrent;	// 4
	float		battVoltage;	// 4
	uint32_t	driveMode;		// 5
	uint32_t	millis;			// 6
	uint8_t		parmID;			// 7
	float		parmValue;		// 8
} __attribute__((__packed__)) DataFields;

int Serial_InitPort(uint32_t baudrate, uint32_t stopbits, uint32_t datasize, uint32_t parity);
void Serial_RxData(uint16_t Size);
void Serial_TxData(uint16_t Size);
int Serial_WaitingForAck(void);
DataFields* Serial_GetResponse(void);
DataFields* Serial_GetTransmit(void);
int Serial_SendCommand(uint32_t cmd);


#endif /* SERIAL_H_ */
