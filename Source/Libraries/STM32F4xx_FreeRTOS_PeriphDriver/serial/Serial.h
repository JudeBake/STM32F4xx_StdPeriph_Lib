/*
 * Serial.h
 *
 *  Created on: Apr 24, 2013
 *      Author: julien
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "SerialType.h"

#define STRING_BUFFER_LENGTH	50

class Serial
{
	//port configuration
	COMMPort commPort;
	Parity parity;
	StopBits stopBits;
	DataBits dataBits;
	HwFlowCtrl hwFlowCtrl;
	LinkMode linkMode;
	BaudRate baudRate;

	//queue handle
	CQueue dataStreamIn;
	CQueue dataStreamOut;

public:
	//constructors
	Serial(void);
	Serial(COMMPort iPort, Parity iParityConf, StopBits iStopBitConf,
			DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl, LinkMode iMode,
			BaudRate iBaudRateConf);

	//init methods
	uint8_t usartInit(COMMPort iPort, Parity iParityConf, StopBits iStopBitConf,
			DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl, LinkMode iMode,
			BaudRate iBaudRateConf);

	//port reading methods
	uint8_t getChar(const int8_t *oCharacter, portTickType iBlockTime);

	//port writing methods
	uint8_t putChar(const int8_t iCharacter, portTickType iBlockTime);
	uint8_t putString(const int8_t * const iString);

	//semaphore methods


	//destructor
	~Serial();
};


#endif /* SERIAL_H_ */
