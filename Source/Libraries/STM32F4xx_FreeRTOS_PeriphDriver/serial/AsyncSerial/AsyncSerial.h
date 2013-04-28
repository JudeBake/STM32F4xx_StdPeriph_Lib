/*
 * AsyncSerial.h
 *
 *  Created on: Apr 24, 2013
 *      Author: julien
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "AsyncSerialType.h"

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

	//queue handle
	CQueue dataStreamIn;
	CQueue dataStreamOut;

public:
	//constructors
	AsyncSerial(void);
	AsyncSerial(COMMPort iPort, Parity iParityConf, StopBits iStopBitConf,
			DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl, LinkMode iMode,
			BaudRate iBaudRateConf, InterruptSetting iInterruptSetting);

	//init methods
	uint8_t usartInit(COMMPort iPort, Parity iParityConf, StopBits iStopBitConf,
			DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl, LinkMode iMode,
			BaudRate iBaudRateConf, InterruptSetting iInterruptSetting);

	//port reading methods
	uint8_t getChar(const int8_t *oCharacter, portTickType iBlockTime);

	//port writing methods
	uint8_t putChar(const int8_t iCharacter, portTickType iBlockTime);
	uint8_t putString(const int8_t * const iString);

	//semaphore methods


	//destructor
	~AsyncSerial();
};


#endif /* SERIAL_H_ */
