/**
 * \file AsyncSerialPort2.cpp
 *
 * \date Created on: Dec 9, 2013
 * \date Last change on: &DATE&
 *
 * \author Created by: julien 
 * \author Last change by: &AUTHOR&
 *
 * \version Commit Id: &REVISION&
 */

#include "SerialPortConf.h"
#ifdef ASYNC_SERIAL_PORT2

//FreeRTOS includes
#include "CFreeRTOS.h"

//STM includes
#include "stm32f4xx.h"
#include "misc.h"

//Class include
#include "AsyncSerialPort2.h"
#include "stm32f4xx_USARTPinAss.h"

//Constants definition
#define SER_NO_BLOCK			((portTickType) 0)

/*
 * Initializing the port handle
 */
AsyncSerialPort2* AsyncSerialPort2::portHandle = NULL;

/*
 * Interrupt handler
 */
#ifdef __cplusplus
extern "C"
{
#endif
void USART2_IRQHandler(void);
#ifdef __cplusplus
}
#endif

/*
 * Private functions
 */
void initPort2(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting);
void setParityPort2(USART_InitTypeDef& iUsartInitStruct, Parity iParity);
void setStopBitPort2(USART_InitTypeDef& iUsartInitStruct, StopBits iStopBit);
void setWordlengthPort2(USART_InitTypeDef& iUsartInitStruct, DataBits iWordLength);
void setLinkModePort2(GPIO_InitTypeDef& iGpioInitStruct,
		USART_InitTypeDef& iUsartInitStruct, LinkMode iLinkMode);
void setHwFlowCtrlPort2(GPIO_InitTypeDef& iGpioInitStruct,
		USART_InitTypeDef& iUsartInitStruct, HwFlowCtrl iHwFlowCtrl);
void setNvicPort2(uint8_t iPreempPriority, uint8_t iSubPriority);

/*
 * Constructors
 */
AsyncSerialPort2::AsyncSerialPort2(void)
{
	currentStatus = SERIAL_INIT_ERROR;
	parity = SERIAL_NO_PARITY;
	stopBits = SERIAL_1_STOP_BIT;
	dataBits = SERIAL_8_BITS_DATA;
	hwFlowCtrl = SERIAL_NO_HW_FLOW_CTRL;
	linkMode = SERIAL_FULL_DUPLEX;
	baudRate = SERIAL_19200_BAUD;
	preempPriority = SERIAL_PORT2_PREEMP_PRIORITY;
	subPriority = SERIAL_PORT2_SUBPRIORITY;
	interruptSetting = SERIAL_INT_DISABLE;

	dataStreamIn.Create(SERIAL_PORT2_BUFFERS_LENGTH, sizeof(uint8_t));
	dataStreamOut.Create(SERIAL_PORT2_BUFFERS_LENGTH, sizeof(uint8_t));

	portMutex.Create();
}

AsyncSerialPort2::AsyncSerialPort2(const AsyncSerialPort2&)
{
	//just to shut down warning
	currentStatus = SERIAL_INIT_ERROR;
	parity = SERIAL_NO_PARITY;
	stopBits = SERIAL_1_STOP_BIT;
	dataBits = SERIAL_8_BITS_DATA;
	hwFlowCtrl = SERIAL_NO_HW_FLOW_CTRL;
	linkMode = SERIAL_FULL_DUPLEX;
	baudRate = SERIAL_19200_BAUD;
	preempPriority = SERIAL_PORT2_PREEMP_PRIORITY;
	subPriority = SERIAL_PORT2_SUBPRIORITY;
	interruptSetting = SERIAL_INT_DISABLE;
}

void AsyncSerialPort2::operator=(const AsyncSerialPort2&)
{

}

/*
 * Instance getter
 */
AsyncSerialPort2 *AsyncSerialPort2::getInstance(void)
{
	if (AsyncSerialPort2::portHandle == NULL)
	{
		AsyncSerialPort2::portHandle = new AsyncSerialPort2;
	}

	return AsyncSerialPort2::portHandle;
}

/*
 * Serial port initializer
 */
SerialStatus AsyncSerialPort2::portInit(Parity iParityConf,
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

	if ((dataStreamIn.IsValid()) && (dataStreamOut.IsValid()) &&
			(portMutex.IsValid()))
	{
		initPort2(parity, stopBits, dataBits, hwFlowCtrl, linkMode, baudRate,
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
SerialStatus AsyncSerialPort2::getCurrentStatus(void)
{
	return currentStatus;
}

/* Current status setter
 * ***USED ONLY IN INTERRUPT HANDLER***
 */
void AsyncSerialPort2::setCurrentStatus(SerialStatus iStatus)
{
	currentStatus = iStatus;
}

/*
 * Stream getters
 * ***USED ONLY IN INTERRUPT HANDLER***
 */
CQueue& AsyncSerialPort2::getOutStream(void)
{
	return dataStreamOut;
}

CQueue& AsyncSerialPort2::getInStream(void)
{
	return dataStreamIn;
}

/*
 * Port reading method
 */
SerialStatus AsyncSerialPort2::getChar(const int8_t* oCharacter,
		portTickType iBlockTime)
{
	SerialStatus oReturn = currentStatus;

	if (currentStatus == SERIAL_OK)
	{
		if (!dataStreamIn.Receive((void*)oCharacter, iBlockTime))
		{
			oReturn = SERIAL_RX_BUFFER_EMPTY;
		}
	}

	return oReturn;
}

/*
 * Port writing methods
 */
SerialStatus AsyncSerialPort2::putChar(const int8_t iCharacter,
		portTickType iBlockTime)
{
	if (currentStatus == SERIAL_OK)
	{
		if (dataStreamOut.Send((void*)&iCharacter, iBlockTime))
		{
			USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
		}

		else
		{
			currentStatus = SERIAL_TX_BUFFER_FULL;
		}
	}

	return currentStatus;
}

SerialStatus AsyncSerialPort2::putString(const int8_t* const iString,
		uint32_t* oNbCharSent)
{
	int8_t* nextChar = (int8_t*)iString;
	uint32_t nbCharSent = 0;

	while (*nextChar && (currentStatus == SERIAL_OK))
	{
		currentStatus = putChar(*nextChar, SER_NO_BLOCK);
		nextChar++;

		if (currentStatus == SERIAL_OK)
		{
			nbCharSent++;
		}
	}

	*oNbCharSent = nbCharSent;

	return currentStatus;
}

/*
 * Mutex method
 */
portBASE_TYPE AsyncSerialPort2::takeMutex(portTickType iBlockTime)
{
	return portMutex.Take(iBlockTime);
}

portBASE_TYPE AsyncSerialPort2::giveMutex(void)
{
	return portMutex.Give();
}
/*
 * Closing port method
 */
void AsyncSerialPort2::closePort(void)
{
	delete AsyncSerialPort2::portHandle;
	AsyncSerialPort2::portHandle = NULL;
}

/*
 * Private functions
 */
void initPort2(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	GPIO_InitTypeDef gpioInitStruct;

	//enable USART clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	//init the gpio init struct with common values
	gpioInitStruct.GPIO_Pin = 0;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	//set up the link mode
	setLinkModePort2(gpioInitStruct, usartInitStruct, iLinkMode);

	//set up the hardware flow control
	setHwFlowCtrlPort2(gpioInitStruct, usartInitStruct, iHwFlowCtrl);

	//set up the parity
	setParityPort2(usartInitStruct, iParity);

	//set up thestop bit config
	setStopBitPort2(usartInitStruct, iStopBit);

	//set up the word length
	setWordlengthPort2(usartInitStruct, iDataLength);

	//set the baud rate
	usartInitStruct.USART_BaudRate = iBaudRate;

	//set up the usart clock
	usartClockInitStruct.USART_Clock = USART_Clock_Disable;
	usartClockInitStruct.USART_CPOL = USART_CPOL_Low;
	usartClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	usartClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	//init the gpio and the usart
	GPIO_Init(USART2_GPIO_PORT, &gpioInitStruct);
	USART_Init(USART2, &usartInitStruct);
	USART_ClockInit(USART2, &usartClockInitStruct);

	//set the interrupt if enabled
	if (iInterruptSetting == SERIAL_INT_ENABLE)
	{
		setNvicPort2(iPreempPriority, iSubPriority);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

		if ((iParity == SERIAL_ODD_PARITY) || (iParity == SERIAL_EVEN_PARITY))
		{
			USART_ITConfig(USART2, USART_IT_PE, ENABLE);
		}

		if (iHwFlowCtrl == SERIAL_HW_FLOW_CTRL_CTS ||
				iHwFlowCtrl == SERIAL_HW_FLOW_CTRL_RTS_CTS)
		{
			USART_ITConfig(USART2, USART_IT_CTS, ENABLE);
		}

		USART_Cmd(USART2, ENABLE);
	}
}

void setParityPort2(USART_InitTypeDef& iUsartInitStruct, Parity iParity)
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

void setStopBitPort2(USART_InitTypeDef& iUsartInitStruct, StopBits iStopBit)
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

void setWordlengthPort2(USART_InitTypeDef& iUsartInitStruct, DataBits iWordLength)
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

void setLinkModePort2(GPIO_InitTypeDef& iGpioInitStruct,
		USART_InitTypeDef& iUsartInitStruct, LinkMode iLinkMode)
{
	switch (iLinkMode)
	{
	case SERIAL_SIMPLEX_RX:
		iGpioInitStruct.GPIO_Pin |= USART2_RX;
		GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_RX_SOURCE, GPIO_AF_USART2);
		iUsartInitStruct.USART_Mode = USART_Mode_Rx;
		break;
	case SERIAL_SIMPLEX_TX:
		iGpioInitStruct.GPIO_Pin |= USART2_TX;
		GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_TX_SOURCE, GPIO_AF_USART2);
		iUsartInitStruct.USART_Mode = USART_Mode_Tx;
		break;
	case SERIAL_FULL_DUPLEX:
		iGpioInitStruct.GPIO_Pin |= USART2_RX | USART2_TX;
		GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_RX_SOURCE, GPIO_AF_USART2);
		GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_TX_SOURCE, GPIO_AF_USART2);
		iUsartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		break;
	case SERIAL_HALF_DUPLEX:
		//not implemented in this class. See HalfDuplexSerialPortX class
		break;
	}
}

void setHwFlowCtrlPort2(GPIO_InitTypeDef& iGpioInitStruct,
		USART_InitTypeDef& iUsartInitStruct, HwFlowCtrl iHwFlowCtrl)
{
	switch (iHwFlowCtrl)
	{
	case SERIAL_NO_HW_FLOW_CTRL:
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_None;
		break;
	case SERIAL_HW_FLOW_CTRL_RTS:
		iGpioInitStruct.GPIO_Pin |= USART2_RTS;
		GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_RTS_SOURCE, GPIO_AF_USART2);
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_RTS;
		break;
	case SERIAL_HW_FLOW_CTRL_CTS:
		iGpioInitStruct.GPIO_Pin |= USART2_CTS;
		GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_CTS_SOURCE, GPIO_AF_USART2);
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_CTS;
		break;
	case SERIAL_HW_FLOW_CTRL_RTS_CTS:
		iGpioInitStruct.GPIO_Pin |= USART2_RTS | USART2_CTS;
		GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_RTS_SOURCE, GPIO_AF_USART2);
		GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_CTS_SOURCE, GPIO_AF_USART2);
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_RTS_CTS;
		break;
	}
}

void setNvicPort2(uint8_t iPreempPriority, uint8_t iSubPriority)
{
	NVIC_InitTypeDef nvicInitStruct;

	nvicInitStruct.NVIC_IRQChannel = USART2_IRQn;
	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = iPreempPriority;
	nvicInitStruct.NVIC_IRQChannelSubPriority = iSubPriority;
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvicInitStruct);
}

/*
 * Interrupt handler
 */
void USART2_IRQHandler(void)
{
	static AsyncSerialPort2* portHandle = AsyncSerialPort2::getInstance();
	static CQueue outStream = portHandle->getOutStream();
	static CQueue inStream = portHandle->getInStream();

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	portCHAR cChar;

	if (USART_GetITStatus(USART2, USART_IT_TXE) == SET)
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
			more characters to transmit? */
		if (outStream.ReceiveFromISR(&cChar, &xHigherPriorityTaskWoken))
		{
			/* A character was retrieved from the queue so can be sent to the
				THR now. */
			USART_SendData(USART2, cChar);

			if (portHandle->getCurrentStatus() == SERIAL_TX_BUFFER_FULL)
			{
				portHandle->setCurrentStatus(SERIAL_OK);
			}
		}
		else
		{
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		}
	}

	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		cChar = USART_ReceiveData(USART2);

		if (!inStream.SendFromISR(&cChar, &xHigherPriorityTaskWoken))
		{
			portHandle->setCurrentStatus(SERIAL_OVERRUN_ERROR);
		}
	}

	if (USART_GetITStatus(USART2, USART_IT_PE) == SET)
	{
		portHandle->setCurrentStatus(SERIAL_PARITY_ERROR);
	}

	if (USART_GetITStatus(USART2, USART_IT_ORE_RX) == SET)
	{
		portHandle->setCurrentStatus(SERIAL_OVERRUN_ERROR);
	}

	if (USART_GetITStatus(USART2, USART_IT_CTS) == SET)
	{
		if (GPIO_ReadInputDataBit(USART2_GPIO_PORT, USART2_CTS))
		{
			portHandle->setCurrentStatus(SERIAL_BUSY);
		}
		else
		{
			portHandle->setCurrentStatus(SERIAL_OK);
		}
	}

	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken );
}

#endif
