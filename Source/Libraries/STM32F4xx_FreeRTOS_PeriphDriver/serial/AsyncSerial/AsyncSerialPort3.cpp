/**
 * \file AsyncSerialPort3.cpp
 *
 * \date Created on: Dec 7, 2013
 * \date Last change on: &DATE&
 *
 * \author Created by: julien 
 * \author Last change by: &AUTHOR&
 *
 * \version Commit Id: &REVISION&
 */

#include "SerialPortConf.h"
#ifdef ASYNC_SERIAL_PORT3

//FreeRTOS includes
#include "CFreeRTOS.h"

//STM includes
#include "stm32f4xx.h"
#include "misc.h"

//Class include
#include "AsyncSerialPort3.h"
#include "stm32f4xx_USARTPinAss.h"

//Constants definition
#define SER_NO_BLOCK			((portTickType) 0)

/*
 * Initializing the port handle
 */
AsyncSerialPort3* AsyncSerialPort3::portHandle = NULL;

/*
 * Interrupt handler
 */
#ifdef __cplusplus
extern "C"
{
#endif
void USART3_IRQHandler(void);
#ifdef __cplusplus
}
#endif

/*
 * Private functions
 */
void initPort3(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting);
void setParityPort3(USART_InitTypeDef& iUsartInitStruct, Parity iParity);
void setStopBitPort3(USART_InitTypeDef& iUsartInitStruct, StopBits iStopBit);
void setWordlengthPort3(USART_InitTypeDef& iUsartInitStruct, DataBits iWordLength);
void setLinkModePort3(GPIO_InitTypeDef& iGpioInitStruct,
		USART_InitTypeDef& iUsartInitStruct, LinkMode iLinkMode);
void setHwFlowCtrlPort3(GPIO_InitTypeDef& iGpioInitStruct,
		USART_InitTypeDef& iUsartInitStruct, HwFlowCtrl iHwFlowCtrl);
void setNvicPort3(uint8_t iPreempPriority, uint8_t iSubPriority);

/*
 * Constructors
 */
AsyncSerialPort3::AsyncSerialPort3(void)
{
	currentStatus = SERIAL_INIT_ERROR;
	parity = SERIAL_NO_PARITY;
	stopBits = SERIAL_1_STOP_BIT;
	dataBits = SERIAL_8_BITS_DATA;
	hwFlowCtrl = SERIAL_NO_HW_FLOW_CTRL;
	linkMode = SERIAL_FULL_DUPLEX;
	baudRate = SERIAL_19200_BAUD;
	preempPriority = SERIAL_PORT3_PREEMP_PRIORITY;
	subPriority = SERIAL_PORT3_SUBPRIORITY;
	interruptSetting = SERIAL_INT_DISABLE;

	dataStreamIn.Create(SERIAL_PORT3_BUFFERS_LENGTH, sizeof(uint8_t));
	dataStreamOut.Create(SERIAL_PORT3_BUFFERS_LENGTH, sizeof(uint8_t));

	portMutex.Create();
}

AsyncSerialPort3::AsyncSerialPort3(const AsyncSerialPort3&)
{
	//just to shut down warning
	currentStatus = SERIAL_INIT_ERROR;
	parity = SERIAL_NO_PARITY;
	stopBits = SERIAL_1_STOP_BIT;
	dataBits = SERIAL_8_BITS_DATA;
	hwFlowCtrl = SERIAL_NO_HW_FLOW_CTRL;
	linkMode = SERIAL_FULL_DUPLEX;
	baudRate = SERIAL_19200_BAUD;
	preempPriority = SERIAL_PORT3_PREEMP_PRIORITY;
	subPriority = SERIAL_PORT3_SUBPRIORITY;
	interruptSetting = SERIAL_INT_DISABLE;
}

void AsyncSerialPort3::operator=(const AsyncSerialPort3&)
{

}

/*
 * Instance getter
 */
AsyncSerialPort3 *AsyncSerialPort3::getInstance(void)
{
	if (AsyncSerialPort3::portHandle == NULL)
	{
		AsyncSerialPort3::portHandle = new AsyncSerialPort3;
	}

	return AsyncSerialPort3::portHandle;
}

/*
 * Serial port initializer
 */
SerialStatus AsyncSerialPort3::portInit(Parity iParityConf,
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
		initPort3(parity, stopBits, dataBits, hwFlowCtrl, linkMode, baudRate,
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
SerialStatus AsyncSerialPort3::getCurrentStatus(void)
{
	return currentStatus;
}

/* Current status setter
 * ***USED ONLY IN INTERRUPT HANDLER***
 */
void AsyncSerialPort3::setCurrentStatus(SerialStatus iStatus)
{
	currentStatus = iStatus;
}

/*
 * Stream getters
 * ***USED ONLY IN INTERRUPT HANDLER***
 */
CQueue& AsyncSerialPort3::getOutStream(void)
{
	return dataStreamOut;
}

CQueue& AsyncSerialPort3::getInStream(void)
{
	return dataStreamIn;
}

/*
 * Port reading method
 */
SerialStatus AsyncSerialPort3::getChar(const int8_t* oCharacter,
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
SerialStatus AsyncSerialPort3::putChar(const int8_t iCharacter,
		portTickType iBlockTime)
{
	if (currentStatus == SERIAL_OK)
	{
		if (dataStreamOut.Send((void*)&iCharacter, iBlockTime))
		{
			USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
		}

		else
		{
			currentStatus = SERIAL_TX_BUFFER_FULL;
		}
	}

	return currentStatus;
}

SerialStatus AsyncSerialPort3::putString(const int8_t* const iString,
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
portBASE_TYPE AsyncSerialPort3::takeMutex(portTickType iBlockTime)
{
	return portMutex.Take(iBlockTime);
}

portBASE_TYPE AsyncSerialPort3::giveMutex(void)
{
	return portMutex.Give();
}
/*
 * Closing port method
 */
void AsyncSerialPort3::closePort(void)
{
	delete AsyncSerialPort3::portHandle;
	AsyncSerialPort3::portHandle = NULL;
}

/*
 * Private functions
 */
void initPort3(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	GPIO_InitTypeDef gpioInitStruct;

	//enable USART clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//init the gpio init struct with common values
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	//set up the link mode
	setLinkModePort3(gpioInitStruct, usartInitStruct, iLinkMode);

	//set up the hardware flow control
	setHwFlowCtrlPort3(gpioInitStruct, usartInitStruct, iHwFlowCtrl);

	//set up the parity
	setParityPort3(usartInitStruct, iParity);

	//set up thestop bit config
	setStopBitPort3(usartInitStruct, iStopBit);

	//set up the word length
	setWordlengthPort3(usartInitStruct, iDataLength);

	//set the baud rate
	usartInitStruct.USART_BaudRate = iBaudRate;

	//set up the usart clock
	usartClockInitStruct.USART_Clock = USART_Clock_Disable;
	usartClockInitStruct.USART_CPOL = USART_CPOL_Low;
	usartClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	usartClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	//init the usart
	USART_Init(USART3, &usartInitStruct);
	USART_ClockInit(USART3, &usartClockInitStruct);

	//set the interrupt if enabled
	if (iInterruptSetting == SERIAL_INT_ENABLE)
	{
		setNvicPort3(iPreempPriority, iSubPriority);
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

		if ((iParity == SERIAL_ODD_PARITY) || (iParity == SERIAL_EVEN_PARITY))
		{
			USART_ITConfig(USART3, USART_IT_PE, ENABLE);
		}

		if (iHwFlowCtrl == SERIAL_HW_FLOW_CTRL_CTS ||
				iHwFlowCtrl == SERIAL_HW_FLOW_CTRL_RTS_CTS)
		{
			USART_ITConfig(USART3, USART_IT_CTS, ENABLE);
		}

		USART_Cmd(USART3, ENABLE);
	}
}

void setParityPort3(USART_InitTypeDef& iUsartInitStruct, Parity iParity)
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

void setStopBitPort3(USART_InitTypeDef& iUsartInitStruct, StopBits iStopBit)
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

void setWordlengthPort3(USART_InitTypeDef& iUsartInitStruct, DataBits iWordLength)
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

void setLinkModePort3(GPIO_InitTypeDef& iGpioInitStruct,
		USART_InitTypeDef& iUsartInitStruct, LinkMode iLinkMode)
{
	switch (iLinkMode)
	{
	case SERIAL_SIMPLEX_RX:
		iGpioInitStruct.GPIO_Pin = USART3_RX;
		GPIO_Init(UART4_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART3_GPIO_PORT, USART3_RX_SOURCE, GPIO_AF_USART3);
		iUsartInitStruct.USART_Mode = USART_Mode_Rx;
		break;
	case SERIAL_SIMPLEX_TX:
		iGpioInitStruct.GPIO_Pin = USART3_TX;
		GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART3_GPIO_PORT, USART3_TX_SOURCE, GPIO_AF_USART3);
		iUsartInitStruct.USART_Mode = USART_Mode_Tx;
		break;
	case SERIAL_FULL_DUPLEX:
		iGpioInitStruct.GPIO_Pin = USART3_RX | USART3_TX;
		GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART3_GPIO_PORT, USART3_RX_SOURCE | USART3_TX_SOURCE,
				GPIO_AF_USART3);
		iUsartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		break;
	case SERIAL_HALF_DUPLEX:
		//not implemented in this class. See HalfDuplexSerialPortX class
		break;
	}
}

void setHwFlowCtrlPort3(GPIO_InitTypeDef& iGpioInitStruct,
		USART_InitTypeDef& iUsartInitStruct, HwFlowCtrl iHwFlowCtrl)
{
	switch (iHwFlowCtrl)
	{
	case SERIAL_NO_HW_FLOW_CTRL:
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_None;
		break;
	case SERIAL_HW_FLOW_CTRL_RTS:
		iGpioInitStruct.GPIO_Pin = USART3_RTS;
		GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART3_GPIO_PORT, USART3_CTS_SOURCE, GPIO_AF_USART3);
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_RTS;
		break;
	case SERIAL_HW_FLOW_CTRL_CTS:
		iGpioInitStruct.GPIO_Pin = USART3_CTS;
		GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART3_GPIO_PORT, USART3_CTS_SOURCE, GPIO_AF_USART3);
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_CTS;
		break;
	case SERIAL_HW_FLOW_CTRL_RTS_CTS:
		iGpioInitStruct.GPIO_Pin = USART3_RTS | USART3_CTS;
		GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART3_GPIO_PORT,
				USART3_RTS_SOURCE | USART3_CTS_SOURCE, GPIO_AF_USART3);
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_RTS_CTS;
		break;
	}
}

void setNvicPort3(uint8_t iPreempPriority, uint8_t iSubPriority)
{
	NVIC_InitTypeDef nvicInitStruct;

	nvicInitStruct.NVIC_IRQChannel = USART3_IRQn;
	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = iPreempPriority;
	nvicInitStruct.NVIC_IRQChannelSubPriority = iSubPriority;
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvicInitStruct);
}

/*
 * Interrupt handler
 */
void USART3_IRQHandler(void)
{
	static AsyncSerialPort3* portHandle = AsyncSerialPort3::getInstance();
	static CQueue outStream = portHandle->getOutStream();
	static CQueue inStream = portHandle->getInStream();

	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	portCHAR cChar;

	if (USART_GetITStatus(USART3, USART_IT_TXE) == SET)
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
			more characters to transmit? */
		if (outStream.ReceiveFromISR(&cChar, &xHigherPriorityTaskWoken))
		{
			/* A character was retrieved from the queue so can be sent to the
				THR now. */
			USART_SendData(USART3, cChar);

			if (portHandle->getCurrentStatus() == SERIAL_TX_BUFFER_FULL)
			{
				portHandle->setCurrentStatus(SERIAL_OK);
			}
		}
		else
		{
			USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
		}
	}

	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		cChar = USART_ReceiveData(USART3);

		inStream.SendFromISR(&cChar, &xHigherPriorityTaskWoken);
	}

	if (USART_GetITStatus(USART3, USART_IT_PE) == SET)
	{
		portHandle->setCurrentStatus(SERIAL_PARITY_ERROR);
	}

	if (USART_GetITStatus(USART3, USART_IT_ORE_RX) == SET)
	{
		portHandle->setCurrentStatus(SERIAL_OVERRUN_ERROR);
	}

	if (USART_GetITStatus(USART3, USART_IT_CTS) == SET)
	{
		if (GPIO_ReadInputDataBit(USART3_GPIO_PORT, USART3_CTS))
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

