/*
 * AsyncSerial.h
 *
 *  Created on: Apr 24, 2013
 *      Author: julien
 *
 *  TODO: Need Device dependent implementation for the pins used by the ports.
 *  for now it mostly support STM32F405/07xx LQPF100.
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "SerialType.h"

#define STRING_BUFFER_LENGTH	50

#define PREEMP_PRIORITY			15
#define USART1_SUBPRIORITY		0
#define USART2_SUBPRIORITY		0
#define USART3_SUBPRIORITY		0
#define USART4_SUBPRIORITY		0
#define USART5_SUBPRIORITY		0
#define USART6_SUBPRIORITY		0

class AsyncSerial
{
	//port configuration
	COMMPort commPort;
	Parity parity;
	StopBits stopBits;
	DataBits dataBits;
	HwFlowCtrl hwFlowCtrl;
	LinkMode linkMode;
	BaudRate baudRate;
	uint8_t preempPriority;
	uint8_t subPriority;
	InterruptSetting interruptSetting;
	SerialError currentStatus;

	//queue handle
	CQueue dataStreamIn;
	CQueue dataStreamOut;

public:
	/* Constructors */
	AsyncSerial(void);
	AsyncSerial(COMMPort iPort, Parity iParityConf, StopBits iStopBitConf,
			DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode,
			BaudRate iBaudRateConf, uint8_t iPreempPriority,
			uint8_t iSubPriority, InterruptSetting iInterruptSetting);

	/* Serial port initializer */
	SerialError usartInit(COMMPort iPort, Parity iParityConf,
			StopBits iStopBitConf, DataBits iDataLengthConf,
			HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRateConf,
			uint8_t iPreempPriority, uint8_t iSubPriority,
			InterruptSetting iInterruptSetting);

	/* Current status getter */
	SerialError getCurrentStatus();

	/* Port reading method */
	SerialError getChar(const int8_t *oCharacter, portTickType iBlockTime);

	/* Port writing methods */
	SerialError putChar(const int8_t iCharacter, portTickType iBlockTime);
	SerialError putString(const int8_t * const iString);

	/* Semaphore method */


	/* Destructor */
	~AsyncSerial();
};


#endif /* SERIAL_H_ */
