/*
 * AsyncSerialType.h
 *
 *  Created on: Apr 24, 2013
 *      Author: julien
 */

#ifndef ASYNCSERIALTYPE_H_
#define ASYNCSERIALTYPE_H_

typedef enum
{
	NO_PORT,
	COMM_PORT_1,
	COMM_PORT_2,
	COMM_PORT_3,
	COMM_PORT_4,
	COMM_PORT_5,
	COMM_PORT_6
} COMMPort;

typedef enum
{
	NO_PARITY,
	ODD_PARITY,
	EVEN_PARITY
} Parity;

typedef enum
{
	STOP_BIT_1,
	STOP_BIT_2
} StopBits;

typedef enum
{
	DATA_8_BITS,
	DATA_9_BITS
} DataBits;

typedef enum
{
	NO_HW_FLOW_CTRL,
	HW_FLOW_CTRL_RTS,
	HW_FLOW_CTRL_CTS,
	HW_FLOW_CTRL_RTS_CTS
} HwFlowCtrl;

typedef enum
{
	SIMPLEX_RX,
	SIMPLEX_TX,
	FULL_DUPLEX
} LinkMode;

typedef enum
{
	BAUD_1200 = 1200,
	BAUD_2400 = 2400,
	BAUD_9600 = 9600,
	BAUD_19200 = 19200,
	BAUD_38400 = 38400,
	BAUD_57600 = 57600,
	BAUD_115200 = 115200
} BaudRate;

typedef enum
{
	INT_DISEABLE,
	INT_ENABLE
} InterruptSetting;

typedef enum
{
	//TODO: implement error code, may be in a common file for all Serial class
} SerialError;

#endif /* SERIALTYPE_H_ */
