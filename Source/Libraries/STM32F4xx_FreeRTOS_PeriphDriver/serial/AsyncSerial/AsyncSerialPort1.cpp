/*
 * AsyncSerialPort1.cpp
 *
 *  Created on: May 1, 2013
 *      Author: julien
 */

#include "SystemPeriphConf.h"
#ifdef ASYNC_SERIAL_PORT1

//FreeRTOS includes
#include "CFreeRTOS.h"
#include "CQueue.h"
#include "CMutex.h"

//STM includes
#include "stm32f4xx.h"
#include "misc.h"

//Class include
#include "AsyncSerialPort1.h"
#include "stm32f4xx_USARTPinAss.h"

/*
 * Initializing the port handle
 */
AsyncSerialPort1 *AsyncSerialPort1::portHandle = NULL;

/*
 * Interrupt handler
 */
#define usart1InterruptHandler USART1_IRQHandler
void usart1InterruptHandler(void);

/*
 * Private functions
 */
void initPort1(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting);
void setParityPort1(USART_InitTypeDef &iUsartInitStruct, Parity iParity);
void setStopBitPort1(USART_InitTypeDef &iUsartInitStruct, StopBits iStopBit);
void setWordlengthPort1(USART_InitTypeDef &iUsartInitStruct, DataBits iWordLength);
void setLinkModePort1(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, LinkMode iLinkMode);
void setHwFlowCtrlPort1(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, HwFlowCtrl iHwFlowCtrl);
void setNvicPort1(uint8_t iPreempPriority, uint8_t iSubPriority);

/*
 * Constructors
 */
AsyncSerialPort1::AsyncSerialPort1()
{
	currentStatus = SERIAL_INIT_ERROR;
	parity = SERIAL_NO_PARITY;
	stopBits = SERIAL_1_STOP_BIT;
	dataBits = SERIAL_8_BITS_DATA;
	hwFlowCtrl = SERIAL_NO_HW_FLOW_CTRL;
	linkMode = SERIAL_FULL_DUPLEX;
	baudRate = SERIAL_19200_BAUD;
	preempPriority = PREEMP_PRIORITY;
	subPriority = USART1_SUBPRIORITY;
	interruptSetting = SERIAL_INT_DISEABLE;

	dataStreamIn.Create(STRING_BUFFER_LENGTH, sizeof(uint8_t));
	dataStreamOut.Create(STRING_BUFFER_LENGTH, sizeof(uint8_t));
}

void AsyncSerialPort1::AsyncSerialPort1(const AsyncSerialPort1 &)
{

}

void AsyncSerialPort1::operator=(const AsyncSerialPort1 &)
{

}

AsyncSerialPort1::~AsyncSerialPort1()
{
	*AsyncSerialPort1::portHandle = NULL;
}

/*
 * Instance getter
 */
AsyncSerialPort1 *AsyncSerialPort1::getInstance(void)
{
	if (AsyncSerialPort1::portHandle == NULL)
	{
		*AsyncSerialPort1::portHandle = new AsyncSerialPort1;
	}

	return AsyncSerialPort1::portHandle;
}

/*
 * Serial port initializer
 */
SerialStatus AsyncSerialPort1::usartInit(Parity iParityConf,
		StopBits iStopBitConf, DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl,
		LinkMode iLinkMode, BaudRate iBaudRateConf, uint8_t iPreempPriority,
		uint8_t iSubPriority, InterruptSetting iInterruptSetting)
{
	//initialize the class member
	currentStatus = SERIAL_OK;
	parity = iParityConf;
	stopBits = iStopBitConf;
	dataBits = iDataLengthConf;
	hwFlowCtrl = iHwFlowCtrl;
	linkMode = iLinkMode;
	baudRate = iBaudRateConf;
	preempPriority = iPreempPriority;
	subPriority = iSubPriority;
	interruptSetting = iInterruptSetting;

	dataStreamIn.Create(STRING_BUFFER_LENGTH, sizeof(uint8_t));
	dataStreamOut.Create(STRING_BUFFER_LENGTH, sizeof(uint8_t));

	if ((dataStreamIn.IsValid()) && (dataStreamOut.IsValid()))
	{
		initPort1(parity, stopBits, dataBits, hwFlowCtrl, linkMode, baudRate,
				preempPriority, subPriority, interruptSetting);
	}

	else
	{
		currentStatus = SERIAL_INIT_ERROR;
	}
	return currentStatus;
}

/*
 * Current status getter
 */
SerialStatus AsyncSerialPort1::getCurrentStatus()
{
	return currentStatus;
}

/*
 * Port reading method
 */
SerialStatus AsyncSerialPort1::getChar(const int8_t *oCharacter,
		portTickType iBlockTime)
{
	return currentStatus;
}

/*
 * Port writing methods
 */
SerialStatus AsyncSerialPort1::putChar(const int8_t iCharacter,
		portTickType iBlockTime)
{
	return currentStatus;
}

SerialStatus AsyncSerialPort1::putString(const int8_t *const iString)
{
	return currentStatus;
}

/*
 * Closing port method
 */
void closePort(void)
{
	delete AsyncSerialPort1::portHandle;
}

/*
 * Private functions
 */
void initPort1(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	GPIO_InitTypeDef gpioInitStruct;

	//enable USART clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//init the gpio init struct with common values
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	//set up the link mode
	setLinkModePort1(gpioInitStruct, usartInitStruct, iLinkMode);

	//set up the hardware flow control
	setHwFlowCtrlPort1(gpioInitStruct, usartInitStruct, iHwFlowCtrl);

	//set up the parity
	setParityPort1(usartInitStruct, iParity);

	//set up thestop bit config
	setStopBitPort1(usartInitStruct, iStopBit);

	//set up the word length
	setWordlengthPort1(usartInitStruct, iDataLength);

	//set the baud rate
	usartInitStruct.USART_BaudRate = iBaudRate;

	//set up the usart clock
	usartClockInitStruct.USART_Clock = USART_Clock_Disable;
	usartClockInitStruct.USART_CPOL = USART_CPOL_Low;
	usartClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	usartClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	//init the usart
	USART_Init(USART1, &usartInitStruct);
	USART_ClockInit(USART1, &usartClockInitStruct);

	//set the interrupt if enabled
	if (iInterruptSetting == SERIAL_INT_ENABLE)
	{
		setNvic(COMM_PORT_1, iPreempPriority, iSubPriority);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART1, ENABLE);
	}
}

void setParityPort1(USART_InitTypeDef &iUsartInitStruct, Parity iParity)
{
	switch (iParity)
	{
	case SERIAL_NO_PARITY:
		iUsartInitStruct.USART_Parity = USART_Parity_No;
		break;
	case SERIAL_ODD_PARITY:
		iUsartInitStruct.USART_Parity = USART_Parity_Odd;
		break;
	case SERIAL_EVEN_PARITY:
		iUsartInitStruct.USART_Parity = USART_Parity_Even;
		break;
	}
}

void setStopBitPort1(USART_InitTypeDef &iUsartInitStruct, StopBits iStopBit)
{
	switch (iStopBit)
	{
	case SERIAL_1_STOP_BIT:
		iUsartInitStruct.USART_StopBits = USART_StopBits_1;
		break;
	case SERIAL_2_STOP_BIT:
		iUsartInitStruct.USART_StopBits = USART_StopBits_2;
		break;
	}
}

void setWordlengthPort1(USART_InitTypeDef &iUsartInitStruct, DataBits iWordLength)
{
	switch (iWordLength)
	{
	case SERIAL_8_BITS_DATA:
		iUsartInitStruct.USART_WordLength = USART_WordLength_8b;
		break;
	case SERIAL_9_BITS_DATA:
		iUsartInitStruct.USART_WordLength = USART_WordLength_9b;
		break;
	}
}

void setLinkModePort1(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, LinkMode iLinkMode)
{
	switch (iLinkMode)
	{
	case SERIAL_SIMPLEX_RX:
		iGpioInitStruct.GPIO_Pin = USART1_RX;
		GPIO_Init(UART4_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART1_GPIO_PORT, USART1_RX_SOURCE, GPIO_AF_USART1);
		iUsartInitStruct.USART_Mode = USART_Mode_Rx;
		break;
	case SERIAL_SIMPLEX_TX:
		iGpioInitStruct.GPIO_Pin = USART1_TX;
		GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART1_GPIO_PORT, USART1_TX_SOURCE, GPIO_AF_USART1);
		iUsartInitStruct.USART_Mode = USART_Mode_Tx;
		break;
	case SERIAL_FULL_DUPLEX:
		iGpioInitStruct.GPIO_Pin = USART1_RX | USART1_TX;
		GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART1_GPIO_PORT, USART1_RX_SOURCE | USART1_TX_SOURCE,
				GPIO_AF_USART1);
		iUsartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		break;
	case SERIAL_HALF_DUPLEX:
		//not implemented in this class. See HalfDuplexSerialPortX class
		break;
	}
}

void setHwFlowCtrlPort1(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, HwFlowCtrl iHwFlowCtrl)
{
	switch (iHwFlowCtrl)
	{
	case SERIAL_NO_HW_FLOW_CTRL:
		iGpioInitStruct.GPIO_Pin = USART1_RTS;
		GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART1_GPIO_PORT, USART1_RTS_SOURCE, GPIO_AF_USART1);
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_None;
		break;
	case SERIAL_HW_FLOW_CTRL_RTS:
		iGpioInitStruct.GPIO_Pin = USART1_CTS;
		GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART1_GPIO_PORT, USART1_CTS_SOURCE, GPIO_AF_USART1);
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_RTS;
		break;
	case SERIAL_HW_FLOW_CTRL_CTS:
		iGpioInitStruct.GPIO_Pin = USART1_CTS;
		GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART1_GPIO_PORT, USART1_CTS_SOURCE, GPIO_AF_USART1);
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_CTS;
		break;
	case SERIAL_HW_FLOW_CTRL_RTS_CTS:
		iGpioInitStruct.GPIO_Pin = USART1_RTS | USART1_CTS;
		GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART1_GPIO_PORT,
				USART1_RTS_SOURCE | USART1_CTS_SOURCE, GPIO_AF_USART1);
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_RTS_CTS;
		break;
	}
}

void setNvicPort1(uint8_t iPreempPriority, uint8_t iSubPriority)
{
	NVIC_InitTypeDef nvicInitStruct;

	nvicInitStruct.NVIC_IRQChannel = USART1_IRQn;
	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = iPreempPriority;
	nvicInitStruct.NVIC_IRQChannelSubPriority = iSubPriority;
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvicInitStruct);
}

/*
 * Interrupt handler
 */
void usart1InterruptHandler(void)
{

}

#endif
