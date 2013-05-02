/*
 * AsyncSerialPort1.h
 *
 *  Created on: May 1, 2013
 *      Author: julien
 *
 *  @brief	This class is the singleton implementation of the higher level of
 *  		the asynchrone serial port 1.
 *
 */

#ifndef ASYNCSERIAL1_H_
#define ASYNCSERIAL1_H_

#ifdef ASYNC_SERIAL_PORT1

#include "SerialType.h"

#define STRING_BUFFER_LENGTH	100

#define PREEMP_PRIORITY			15
#define USART1_SUBPRIORITY		0

class AsyncSerialPort1
{
	/* Port handle */
	static AsyncSerialPort1 *portHandle;

	/* Port configuration */
	SerialStatus currentStatus;
	Parity parity;
	StopBits stopBits;
	DataBits dataBits;
	HwFlowCtrl hwFlowCtrl;
	LinkMode linkMode;
	BaudRate baudRate;
	uint8_t preempPriority;
	uint8_t subPriority;
	InterruptSetting interruptSetting;

	/* Data stream Queues */
	CQueue dataStreamIn;
	CQueue dataStreamOut;

	/* Constructors */
	AsyncSerialPort1(void);
	void AsyncSerialPort1(const AsyncSerialPort1 &);
	void operator=(const AsyncSerialPort1 &);

	/* Destructor */
	~AsyncSerialPort1(void);

public:
	/* Instance getter */
	static AsyncSerialPort1 *getInstance(void);

	/* Serial port initializer */
	SerialStatus usartInit(Parity iParityConf, StopBits iStopBitConf,
			DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl,
			LinkMode iLinkMode, BaudRate iBaudRateConf,
			uint8_t iPreempPriority, uint8_t iSubPriority,
			InterruptSetting iInterruptSetting);

	/* Current status getter */
	SerialStatus getCurrentStatus();

	/* Port reading method */
	SerialStatus getChar(const int8_t *oCharacter, portTickType iBlockTime);

	/* Port writing methods */
	SerialStatus putChar(const int8_t iCharacter, portTickType iBlockTime);
	SerialStatus putString(const int8_t * const iString);

	/* Semaphore method */

	/* Closing port method */
	static void closePort(void);
};


#endif /* ASYNC_SERIAL_PORT1 */
#endif /* ASYNCSERIAL1_H_ */
