/**
 * \file SerialType.h
 * \brief This file contains every enum used to configure and control serial
 *        ports.
 *
 * \date	Created on: May 1, 2013
 * \date	Last change on: &DATE&
 *
 * \author	Created by: julien
 * \author	Last change by: &AUTHOR&
 *
 * \version Commit Id: &REVISION&
 */

#ifndef SERIALTYPE_H_
#define SERIALTYPE_H_

/**
 * \enum Parity
 * \brief The parity setting for a given port.
 *
 * The choices are:
 * - SERIAL_NO_PARITY
 * - SERIAL_ODD_PARITY
 * - SERIAL_EVEN_PARITY
 */
typedef enum
{
	SERIAL_NO_PARITY,
	SERIAL_ODD_PARITY,
	SERIAL_EVEN_PARITY
} Parity;

/**
 * \enum StopBits
 * \brief The Stop bit(s) setting for a given port.
 *
 * The Choices are:
 * - SERIAL_1_STOP_BIT
 * - SERIAL_2_STOP_BIT
 */
typedef enum
{
	SERIAL_1_STOP_BIT,
	SERIAL_2_STOP_BIT
} StopBits;

/**
 * \enum DataBits
 * \brief The length of the data word setting in bits for a given port.
 *
 * The choices are:
 * - SERIAL_8_BITS_DATA
 * - SERIAL_9_BITS_DATA
 */
typedef enum
{
	SERIAL_8_BITS_DATA,
	SERIAL_9_BITS_DATA
} DataBits;

/**
 * \enum HwFlowCtrl
 * \brief The hardware flow control setting for a given port.
 *
 * The choices are:
 * - SERIAL_NO_HW_FLOW_CTRL
 * - SERIAL_HW_FLOW_CTRL_RTS
 * - SERIAL_HW_FLOW_CTRL_CTS
 * - SERIAL_HW_FLOW_CTRL_RTS_CTS
 */
typedef enum
{
	SERIAL_NO_HW_FLOW_CTRL,
	SERIAL_HW_FLOW_CTRL_RTS,
	SERIAL_HW_FLOW_CTRL_CTS,
	SERIAL_HW_FLOW_CTRL_RTS_CTS
} HwFlowCtrl;

/**
 * \enum LinkMode
 * \brief The link mode setting for a given port.
 *
 * The choices are:
 * - SERIAL_SIMPLEX_RX
 * - SERIAL_SIMPLEX_TX
 * - SERIAL_HALF_DUPLEX
 * - SERIAL_FULL_DUPLEX
 */
typedef enum
{
	SERIAL_SIMPLEX_RX,
	SERIAL_SIMPLEX_TX,
	SERIAL_HALF_DUPLEX,
	SERIAL_FULL_DUPLEX
	//TODO: add LIN, IrDa and Smartcard modes
} LinkMode;

/**
 * \enum BaudRate
 * \brief The baud rate setting for a given port.
 *
 * The choices are:
 * - SERIAL_1200_BAUD
 * - SERIAL_2400_BAUD
 * - SERIAL_9600_BAUD
 * - SERIAL_19200_BAUD
 * - SERIAL_38400_BAUD
 * - SERIAL_57600_BAUD
 * - SERIAL_115200_BAUD
 */
typedef enum
{
	SERIAL_1200_BAUD = 1200,
	SERIAL_2400_BAUD = 2400,
	SERIAL_9600_BAUD = 9600,
	SERIAL_19200_BAUD = 19200,
	SERIAL_38400_BAUD = 38400,
	SERIAL_57600_BAUD = 57600,
	SERIAL_115200_BAUD = 115200
} BaudRate;

/**
 * \enum InterruptSetting
 * \brief The interrupt setting for a given port.
 *
 * The choices are:
 * - SERIAL_INT_DISEABLE
 * - SERIAL_INT_ENABLE
 */
typedef enum
{
	SERIAL_INT_DISABLE = 0,
	SERIAL_INT_ENABLE = 1
} InterruptSetting;

/**
 * \enum SerialStatus
 * \brief The current possible status for a given port
 *
 * The choices are:
 * - SERIAL_INIT_ERROR
 * - SERIAL_OVERRUN_ERROR
 * - SERIAL_NOISE_ERROR
 * - SERIAL_FRAMING_ERROR
 * - SERIAL_PARITY_ERROR
 * - SERIAL_TX_BUFFER_FULL
 * - SERIAL_RX_BUFFER_EMPTY
 * - SERIAL_BUSY
 * - SERIAL_OK
 */
typedef enum
{
	SERIAL_INIT_ERROR,
	SERIAL_OVERRUN_ERROR,
	SERIAL_NOISE_ERROR,
	SERIAL_FRAMING_ERROR,
	SERIAL_PARITY_ERROR,
	SERIAL_TX_BUFFER_FULL,
	SERIAL_RX_BUFFER_EMPTY,
	SERIAL_BUSY,
	SERIAL_OK
} SerialStatus;


#endif /* SERIALTYPE_H_ */
