/*
 * AsyncSerial.cpp
 *
 *  Created on: Apr 24, 2013
 *      Author: julien
 *
 *  TODO: Need Device dependent implementation for the pins used by the ports.
 *  for now it mostly support STM32F405/07xx LQPF100.
 */

//FreeRTOS includes
#include "CFreeRTOS.h"
#include "CQueue.h"
#include "CMutex.h"

//STM includes
#include "stm32f4xx.h"
#include "misc.h"

//Class include
#include "AsyncSerial.h"
#include "stm32f4xx_USARTPinAss.h"

//private function
void initPort1(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting);
void initPort2(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting);
void initPort3(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting);
void initPort4(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting);
void initPort5(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting);
void initPort6(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting);
void setAsSimplexRx(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
void setAsSimplexTx(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
void setAsFullDuplex(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
void setAsNoHwFlowCtrl(USART_InitTypeDef &iUsartInitStruct);
void setAsHwFlowCtrlRTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
void setAsHwFlowCtrlCTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
void setAsHwFlowCtrlRTSCTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
void setWordLength(USART_InitTypeDef &iUsartInitStruct, DataBits iWordLength);
void setStopBits(USART_InitTypeDef &iUsartInitStruct, StopBits iStopBit);
void setParity(USART_InitTypeDef &iUsartInitStruct, Parity iParity);
void setNvic(COMMPort iPort, uint8_t iPreempPriority, uint8_t iSubPriority);

AsyncSerial::AsyncSerial()
{
	//init the port setting
	commPort = NO_PORT;
	parity = NO_PARITY;
	stopBits = STOP_BIT_1;
	dataBits = DATA_8_BITS;
	hwFlowCtrl = NO_HW_FLOW_CTRL;
	linkMode = FULL_DUPLEX;
	baudRate = BAUD_19200;
	preempPriority = 0;
	subPriority = 0;
	interruptSetting = INT_DISEABLE;
}

AsyncSerial::AsyncSerial(COMMPort iPort, Parity iParityConf,
		StopBits iStopBitConf, DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl,
		LinkMode iLinkMode, BaudRate iBaudRateConf, uint8_t iPreempPriority,
		uint8_t iSubPriority, InterruptSetting iInterruptSetting)
{
	//init the port setting
	commPort = iPort;
	parity = iParityConf;
	stopBits = iStopBitConf;
	dataBits = iDataLengthConf;
	hwFlowCtrl = iHwFlowCtrl;
	linkMode = iLinkMode;
	baudRate = iBaudRateConf;
	preempPriority = iPreempPriority;
	subPriority = iSubPriority;
	interruptSetting = iInterruptSetting;

	//init the port queues
	dataStreamIn.Create(STRING_BUFFER_LENGTH, sizeof(int8_t));
	dataStreamOut.Create(STRING_BUFFER_LENGTH, sizeof(int8_t));

	if ((dataStreamIn != NULL) && (dataStreamOut != NULL))
	{
		switch (iPort)
		{
			case COMM_PORT_1:
				initPort1(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate, preempPriority, subPriority,
						interruptSetting);
				break;
			case COMM_PORT_2:
				initPort2(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate, preempPriority, subPriority,
						interruptSetting);
				break;
			case COMM_PORT_3:
				initPort3(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate, preempPriority, subPriority,
						interruptSetting);
				break;
			case COMM_PORT_4:
				initPort4(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate, preempPriority, subPriority,
						interruptSetting);
				break;
			case COMM_PORT_5:
				initPort5(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate, preempPriority, subPriority,
						interruptSetting);
				break;
			case COMM_PORT_6:
				initPort6(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate, preempPriority, subPriority,
						interruptSetting);
				break;
			case NO_PORT:
				break;
		}
	}
}

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

	//setting up the link mode
	switch (iLinkMode)
	{
	case SIMPLEX_RX:
		setAsSimplexRx(gpioInitStruct, usartInitStruct, COMM_PORT_1);
		break;
	case SIMPLEX_TX:
		setAsSimplexTx(gpioInitStruct, usartInitStruct, COMM_PORT_1);
		break;
	case FULL_DUPLEX:
		setAsFullDuplex(gpioInitStruct, usartInitStruct, COMM_PORT_1);
		break;
	}

	//setting up the hardware flow control
	switch (iHwFlowCtrl)
	{
	case NO_HW_FLOW_CTRL:
		setAsNoHwFlowCtrl(usartInitStruct);
		break;
	case HW_FLOW_CTRL_RTS:
		setAsHwFlowCtrlRTS(gpioInitStruct, usartInitStruct, COMM_PORT_1);
		break;
	case HW_FLOW_CTRL_CTS:
		setAsHwFlowCtrlCTS(gpioInitStruct, usartInitStruct, COMM_PORT_1);
		break;
	case HW_FLOW_CTRL_RTS_CTS:
		setAsHwFlowCtrlRTSCTS(gpioInitStruct, usartInitStruct, COMM_PORT_1);
		break;
	}

	//setting up the baud rate, the word length the stop bit and the parity
	usartInitStruct.USART_BaudRate = iBaudRate;
	setWordLength(usartInitStruct, iDataLength);
	setStopBits(usartInitStruct, iStopBit);
	setParity(usartInitStruct, iParity);

	//set up the usart clock
	usartClockInitStruct.USART_Clock = USART_Clock_Disable;
	usartClockInitStruct.USART_CPOL = USART_CPOL_Low;
	usartClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	usartClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	//init the usart
	USART_Init(USART1, &usartInitStruct);
	USART_ClockInit(USART1, &usartClockInitStruct);

	//set the interrupt if enabled
	if (iInterruptSetting == INT_ENABLE)
	{
		setNvic(COMM_PORT_1, iPreempPriority, iSubPriority);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART1, ENABLE);
	}
}

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
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	//setting up the link mode
	switch (iLinkMode)
	{
	case SIMPLEX_RX:
		setAsSimplexRx(gpioInitStruct, usartInitStruct, COMM_PORT_2);
		break;
	case SIMPLEX_TX:
		setAsSimplexTx(gpioInitStruct, usartInitStruct, COMM_PORT_2);
		break;
	case FULL_DUPLEX:
		setAsFullDuplex(gpioInitStruct, usartInitStruct, COMM_PORT_2);
		break;
	}

	//setting up the hardware flow control
	switch (iHwFlowCtrl)
	{
	case NO_HW_FLOW_CTRL:
		setAsNoHwFlowCtrl(usartInitStruct);
		break;
	case HW_FLOW_CTRL_RTS:
		setAsHwFlowCtrlRTS(gpioInitStruct, usartInitStruct, COMM_PORT_2);
		break;
	case HW_FLOW_CTRL_CTS:
		setAsHwFlowCtrlCTS(gpioInitStruct, usartInitStruct, COMM_PORT_2);
		break;
	case HW_FLOW_CTRL_RTS_CTS:
		setAsHwFlowCtrlRTSCTS(gpioInitStruct, usartInitStruct, COMM_PORT_2);
		break;
	}

	//setting up the baud rate, the word length the stop bit and the parity
	usartInitStruct.USART_BaudRate = iBaudRate;
	setWordLength(usartInitStruct, iDataLength);
	setStopBits(usartInitStruct, iStopBit);
	setParity(usartInitStruct, iParity);

	//set up the usart clock
	usartClockInitStruct.USART_Clock = USART_Clock_Disable;
	usartClockInitStruct.USART_CPOL = USART_CPOL_Low;
	usartClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	usartClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	//init the usart
	USART_Init(USART2, &usartInitStruct);
	USART_ClockInit(USART2, &usartClockInitStruct);

	//set the interrupt if enabled
	if (iInterruptSetting == INT_ENABLE)
	{
		setNvic(COMM_PORT_2, iPreempPriority, iSubPriority);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART2, ENABLE);
	}
}

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

	//setting up the link mode
	switch (iLinkMode)
	{
	case SIMPLEX_RX:
		setAsSimplexRx(gpioInitStruct, usartInitStruct, COMM_PORT_3);
		break;
	case SIMPLEX_TX:
		setAsSimplexTx(gpioInitStruct, usartInitStruct, COMM_PORT_3);
		break;
	case FULL_DUPLEX:
		setAsFullDuplex(gpioInitStruct, usartInitStruct, COMM_PORT_3);
		break;
	}

	//setting up the hardware flow control
	switch (iHwFlowCtrl)
	{
	case NO_HW_FLOW_CTRL:
		setAsNoHwFlowCtrl(usartInitStruct);
		break;
	case HW_FLOW_CTRL_RTS:
		setAsHwFlowCtrlRTS(gpioInitStruct, usartInitStruct, COMM_PORT_3);
		break;
	case HW_FLOW_CTRL_CTS:
		setAsHwFlowCtrlCTS(gpioInitStruct, usartInitStruct, COMM_PORT_3);
		break;
	case HW_FLOW_CTRL_RTS_CTS:
		setAsHwFlowCtrlRTSCTS(gpioInitStruct, usartInitStruct, COMM_PORT_3);
		break;
	}

	//setting up the baud rate, the word length the stop bit and the parity
	usartInitStruct.USART_BaudRate = iBaudRate;
	setWordLength(usartInitStruct, iDataLength);
	setStopBits(usartInitStruct, iStopBit);
	setParity(usartInitStruct, iParity);

	//set up the usart clock
	usartClockInitStruct.USART_Clock = USART_Clock_Disable;
	usartClockInitStruct.USART_CPOL = USART_CPOL_Low;
	usartClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	usartClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	//init the usart
	USART_Init(USART3, &usartInitStruct);
	USART_ClockInit(USART3, &usartClockInitStruct);

	//set the interrupt if enabled
	if (iInterruptSetting == INT_ENABLE)
	{
		setNvic(COMM_PORT_3, iPreempPriority, iSubPriority);
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART3, ENABLE);
	}
}

void initPort4(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	GPIO_InitTypeDef gpioInitStruct;

	//enable USART clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//init the gpio init struct with common values
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	//setting up the link mode
	switch (iLinkMode)
	{
	case SIMPLEX_RX:
		setAsSimplexRx(gpioInitStruct, usartInitStruct, COMM_PORT_4);
		break;
	case SIMPLEX_TX:
		setAsSimplexTx(gpioInitStruct, usartInitStruct, COMM_PORT_4);
		break;
	case FULL_DUPLEX:
		setAsFullDuplex(gpioInitStruct, usartInitStruct, COMM_PORT_4);
		break;
	}

	//set the Hardware control flow (none for UART 4)
	setAsNoHwFlowCtrl(usartInitStruct);

	//setting up the baud rate, the word length the stop bit and the parity
	usartInitStruct.USART_BaudRate = iBaudRate;
	setWordLength(usartInitStruct, iDataLength);
	setStopBits(usartInitStruct, iStopBit);
	setParity(usartInitStruct, iParity);

	//set up the usart clock
	usartClockInitStruct.USART_Clock = USART_Clock_Disable;
	usartClockInitStruct.USART_CPOL = USART_CPOL_Low;
	usartClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	usartClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	//init the usart
	USART_Init(UART4, &usartInitStruct);
	USART_ClockInit(UART4, &usartClockInitStruct);

	//set the interrupt if enabled
	if (iInterruptSetting == INT_ENABLE)
	{
		setNvic(COMM_PORT_4, iPreempPriority, iSubPriority);
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
		USART_Cmd(UART4, ENABLE);
	}
}

void initPort5(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	GPIO_InitTypeDef gpioInitStruct;

	//enable USART clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//init the gpio init struct with common values
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_UP;

	//setting up the link mode
	switch (iLinkMode)
	{
	case SIMPLEX_RX:
		setAsSimplexRx(gpioInitStruct, usartInitStruct, COMM_PORT_5);
		break;
	case SIMPLEX_TX:
		setAsSimplexTx(gpioInitStruct, usartInitStruct, COMM_PORT_5);
		break;
	case FULL_DUPLEX:
		setAsFullDuplex(gpioInitStruct, usartInitStruct, COMM_PORT_5);
		break;
	}

	//set the Hardware control flow (none for UART 4)
	setAsNoHwFlowCtrl(usartInitStruct);

	//setting up the baud rate, the word length the stop bit and the parity
	usartInitStruct.USART_BaudRate = iBaudRate;
	setWordLength(usartInitStruct, iDataLength);
	setStopBits(usartInitStruct, iStopBit);
	setParity(usartInitStruct, iParity);

	//set up the usart clock
	usartClockInitStruct.USART_Clock = USART_Clock_Disable;
	usartClockInitStruct.USART_CPOL = USART_CPOL_Low;
	usartClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	usartClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	//init the usart
	USART_Init(UART5, &usartInitStruct);
	USART_ClockInit(UART5, &usartClockInitStruct);

	//set the interrupt if enabled
	if (iInterruptSetting == INT_ENABLE)
	{
		setNvic(COMM_PORT_5, iPreempPriority, iSubPriority);
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
		USART_Cmd(UART5, ENABLE);
	}
}

void initPort6(Parity iParity, StopBits iStopBit, DataBits iDataLength,
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

	//setting up the link mode
	switch (iLinkMode)
	{
	case SIMPLEX_RX:
		setAsSimplexRx(gpioInitStruct, usartInitStruct, COMM_PORT_6);
		break;
	case SIMPLEX_TX:
		setAsSimplexTx(gpioInitStruct, usartInitStruct, COMM_PORT_6);
		break;
	case FULL_DUPLEX:
		setAsFullDuplex(gpioInitStruct, usartInitStruct, COMM_PORT_6);
		break;
	}

	//setting up the hardware flow control
	switch (iHwFlowCtrl)
	{
	case NO_HW_FLOW_CTRL:
		setAsNoHwFlowCtrl(usartInitStruct);
		break;
	case HW_FLOW_CTRL_RTS:
		setAsHwFlowCtrlRTS(gpioInitStruct, usartInitStruct, COMM_PORT_6);
		break;
	case HW_FLOW_CTRL_CTS:
		setAsHwFlowCtrlCTS(gpioInitStruct, usartInitStruct, COMM_PORT_6);
		break;
	case HW_FLOW_CTRL_RTS_CTS:
		setAsHwFlowCtrlRTSCTS(gpioInitStruct, usartInitStruct, COMM_PORT_6);
		break;
	}

	//setting up the baud rate, the word length the stop bit and the parity
	usartInitStruct.USART_BaudRate = iBaudRate;
	setWordLength(usartInitStruct, iDataLength);
	setStopBits(usartInitStruct, iStopBit);
	setParity(usartInitStruct, iParity);

	//set up the usart clock
	usartClockInitStruct.USART_Clock = USART_Clock_Disable;
	usartClockInitStruct.USART_CPOL = USART_CPOL_Low;
	usartClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	usartClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	//init the usart
	USART_Init(USART6, &usartInitStruct);
	USART_ClockInit(USART6, &usartClockInitStruct);

	//set the interrupt if enabled
	if (iInterruptSetting == INT_ENABLE)
	{
		setNvic(COMM_PORT_6, iPreempPriority, iSubPriority);
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART6, ENABLE);
	}
}

void setAsSimplexRx(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
	switch (iPort)
	{
	case COMM_PORT_1:
		iGpioInitStruct.GPIO_Pin = USART1_RX;
		GPIO_Init(UART4_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART1_GPIO_PORT, USART1_RX_SOURCE, GPIO_AF_USART1);
		break;
	case COMM_PORT_2:
		iGpioInitStruct.GPIO_Pin = USART2_RX;
		GPIO_Init(USART2_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_RX_SOURCE, GPIO_AF_USART2);
		break;
	case COMM_PORT_3:
		iGpioInitStruct.GPIO_Pin = USART3_RX;
		GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART3_GPIO_PORT, USART3_RX_SOURCE, GPIO_AF_USART3);
		break;
	case COMM_PORT_4:
		iGpioInitStruct.GPIO_Pin = UART4_RX;
		GPIO_Init(UART4_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(UART4_GPIO_PORT, UART4_RX_SOURCE, GPIO_AF_UART4);
		break;
	case COMM_PORT_5:
		iGpioInitStruct.GPIO_Pin = UART5_RX;
		GPIO_Init(UART5_GPIO_PORT_RX, &iGpioInitStruct);
		GPIO_PinAFConfig(UART5_GPIO_PORT_RX, UART5_RX_SOURCE, GPIO_AF_UART5);
		break;
	case COMM_PORT_6:
		iGpioInitStruct.GPIO_Pin = USART6_RX;
		GPIO_Init(USART6_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART6_GPIO_PORT, USART6_RX_SOURCE, GPIO_AF_USART6);
		break;
	case NO_PORT:
		break;
	}

	iUsartInitStruct.USART_Mode = USART_Mode_Rx;
}

void setAsSimplexTx(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
	switch (iPort)
	{
	case COMM_PORT_1:
		iGpioInitStruct.GPIO_Pin = USART1_TX;
		GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART1_GPIO_PORT, USART1_TX_SOURCE, GPIO_AF_USART1);
		break;
	case COMM_PORT_2:
		iGpioInitStruct.GPIO_Pin = USART2_TX;
		GPIO_Init(USART2_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_TX_SOURCE, GPIO_AF_USART2);
		break;
	case COMM_PORT_3:
		iGpioInitStruct.GPIO_Pin = USART3_TX;
		GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART3_GPIO_PORT, USART3_TX_SOURCE, GPIO_AF_USART3);
		break;
	case COMM_PORT_4:
		iGpioInitStruct.GPIO_Pin = UART4_TX;
		GPIO_Init(UART4_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(UART4_GPIO_PORT, UART4_TX_SOURCE, GPIO_AF_UART4);
		break;
	case COMM_PORT_5:
		iGpioInitStruct.GPIO_Pin = UART5_TX;
		GPIO_Init(UART5_GPIO_PORT_TX, &iGpioInitStruct);
		GPIO_PinAFConfig(UART5_GPIO_PORT_TX, UART5_TX_SOURCE, GPIO_AF_UART5);
		break;
	case COMM_PORT_6:
		iGpioInitStruct.GPIO_Pin = USART6_TX;
		GPIO_Init(USART6_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART6_GPIO_PORT, USART6_TX_SOURCE, GPIO_AF_USART6);
		break;
	case NO_PORT:
		break;
	}

	iUsartInitStruct.USART_Mode = USART_Mode_Tx;
}

void setAsFullDuplex(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
	switch (iPort)
	{
	case COMM_PORT_1:
		iGpioInitStruct.GPIO_Pin = USART1_RX | USART1_TX;
		GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART1_GPIO_PORT, USART1_RX_SOURCE | USART1_TX_SOURCE,
				GPIO_AF_USART1);
		break;
	case COMM_PORT_2:
		iGpioInitStruct.GPIO_Pin = USART2_RX | USART2_TX;
		GPIO_Init(USART2_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_RX_SOURCE | USART2_TX_SOURCE,
				GPIO_AF_USART2);
		break;
	case COMM_PORT_3:
		iGpioInitStruct.GPIO_Pin = USART3_RX | USART3_TX;
		GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART3_GPIO_PORT, USART3_RX_SOURCE | USART3_TX_SOURCE,
				GPIO_AF_USART3);
		break;
	case COMM_PORT_4:
		iGpioInitStruct.GPIO_Pin = UART4_RX | UART4_TX;
		GPIO_Init(UART4_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(UART4_GPIO_PORT, UART4_RX_SOURCE | UART4_TX_SOURCE,
				GPIO_AF_UART4);
		break;
	case COMM_PORT_5:
		iGpioInitStruct.GPIO_Pin = UART5_RX;
		GPIO_Init(UART5_GPIO_PORT_RX, &iGpioInitStruct);
		GPIO_PinAFConfig(UART5_GPIO_PORT_RX, UART5_RX_SOURCE, GPIO_AF_UART5);
		iGpioInitStruct.GPIO_Pin = UART5_TX;
		GPIO_Init(UART5_GPIO_PORT_TX, &iGpioInitStruct);
		GPIO_PinAFConfig(UART5_GPIO_PORT_TX, UART5_TX_SOURCE, GPIO_AF_UART5);
		break;
	case COMM_PORT_6:
		iGpioInitStruct.GPIO_Pin = USART6_RX | USART6_TX;
		GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
		GPIO_PinAFConfig(USART6_GPIO_PORT, USART6_RX_SOURCE | USART6_TX_SOURCE,
				GPIO_AF_USART6);
		break;
	case NO_PORT:
		break;
	}

	iUsartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
}

void setAsNoHwFlowCtrl(USART_InitTypeDef &iUsartInitStruct)
{
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_None;
}

void setAsHwFlowCtrlRTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
		switch (iPort)
		{
		case COMM_PORT_1:
			iGpioInitStruct.GPIO_Pin = USART1_RTS;
			GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART1_GPIO_PORT, USART1_RTS_SOURCE, GPIO_AF_USART1);
			break;
		case COMM_PORT_2:
			iGpioInitStruct.GPIO_Pin = USART2_RTS;
			GPIO_Init(USART2_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_RTS_SOURCE, GPIO_AF_USART2);
			break;
		case COMM_PORT_3:
			iGpioInitStruct.GPIO_Pin = USART3_RTS;
			GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART3_GPIO_PORT, USART3_RTS_SOURCE, GPIO_AF_USART3);
			break;
		case COMM_PORT_4:
			//TODO: may need implementation for other than STM32F405/07xx
			break;
		case COMM_PORT_5:
			//TODO: may need implementation for other than STM32F405/07xx
			break;
		case COMM_PORT_6:
			iGpioInitStruct.GPIO_Pin = USART6_RTS;
			GPIO_Init(USART6_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART6_GPIO_PORT, USART6_RTS_SOURCE, GPIO_AF_USART6);
			break;
		case NO_PORT:
			break;
		}

		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_RTS;
}

void setAsHwFlowCtrlCTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
		switch (iPort)
		{
		case COMM_PORT_1:
			iGpioInitStruct.GPIO_Pin = USART1_CTS;
			GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART1_GPIO_PORT, USART1_CTS_SOURCE, GPIO_AF_USART1);
			break;
		case COMM_PORT_2:
			iGpioInitStruct.GPIO_Pin = USART2_CTS;
			GPIO_Init(USART2_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART2_GPIO_PORT, USART2_CTS_SOURCE, GPIO_AF_USART2);
			break;
		case COMM_PORT_3:
			iGpioInitStruct.GPIO_Pin = USART3_CTS;
			GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART3_GPIO_PORT, USART3_CTS_SOURCE, GPIO_AF_USART3);
			break;
		case COMM_PORT_4:
			//TODO: may need implementation for other than STM32F405/07xx
			break;
		case COMM_PORT_5:
			//TODO: may need implementation for other than STM32F405/07xx
			break;
		case COMM_PORT_6:
			iGpioInitStruct.GPIO_Pin = USART6_CTS;
			GPIO_Init(USART6_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART6_GPIO_PORT, USART6_CTS_SOURCE, GPIO_AF_USART6);
			break;
		case NO_PORT:
			break;
		}

		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_CTS;
}

void setAsHwFlowCtrlRTSCTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
		switch (iPort)
		{
		case COMM_PORT_1:
			iGpioInitStruct.GPIO_Pin = USART1_RTS | USART1_CTS;
			GPIO_Init(USART1_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART1_GPIO_PORT,
					USART1_RTS_SOURCE | USART1_CTS_SOURCE, GPIO_AF_USART1);
			break;
		case COMM_PORT_2:
			iGpioInitStruct.GPIO_Pin = USART2_RTS | USART2_CTS;
			GPIO_Init(USART2_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART2_GPIO_PORT,
					USART2_RTS_SOURCE | USART2_CTS_SOURCE, GPIO_AF_USART2);
			break;
		case COMM_PORT_3:
			iGpioInitStruct.GPIO_Pin = USART3_RTS | USART3_CTS;
			GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART3_GPIO_PORT,
					USART3_RTS_SOURCE | USART3_CTS_SOURCE, GPIO_AF_USART3);
			break;
		case COMM_PORT_4:
			//TODO: may need implementation for other than STM32F405/07xx
			break;
		case COMM_PORT_5:
			//TODO: may need implementation for other than STM32F405/07xx
			break;
		case COMM_PORT_6:
			iGpioInitStruct.GPIO_Pin = USART6_RTS | USART6_CTS;
			GPIO_Init(USART3_GPIO_PORT, &iGpioInitStruct);
			GPIO_PinAFConfig(USART3_GPIO_PORT,
					USART3_RTS_SOURCE | USART3_CTS_SOURCE, GPIO_AF_USART3);
			break;
			case NO_PORT:
				break;
		}

		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_RTS_CTS;
}

void setWordLength(USART_InitTypeDef &iUsartInitStruct, DataBits iWordLength)
{
	switch (iWordLength)
	{
		case DATA_8_BITS:
			iUsartInitStruct.USART_WordLength = USART_WordLength_8b;
			break;
		case DATA_9_BITS:
			iUsartInitStruct.USART_WordLength = USART_WordLength_9b;
			break;
	}
}

void setStopBits(USART_InitTypeDef &iUsartInitStruct, StopBits iStopBit)
{
	switch (iStopBit)
	{
		case STOP_BIT_1:
			iUsartInitStruct.USART_StopBits = USART_StopBits_1;
			break;
		case STOP_BIT_2:
			iUsartInitStruct.USART_StopBits = USART_StopBits_2;
			break;
	}
}

void setParity(USART_InitTypeDef &iUsartInitStruct, Parity iParity)
{
	switch (iParity)
	{
		case NO_PARITY:
			iUsartInitStruct.USART_Parity = USART_Parity_No;
			break;
		case ODD_PARITY:
			iUsartInitStruct.USART_Parity = USART_Parity_Odd;
			break;
		case EVEN_PARITY:
			iUsartInitStruct.USART_Parity = USART_Parity_Even;
			break;
	}
}

void setNvic(COMMPort iPort, uint8_t iPreempPriority, uint8_t iSubPriority)
{
	NVIC_InitTypeDef nvicInitStruct;

	switch(iPort)
	{
		case COMM_PORT_1:
			nvicInitStruct.NVIC_IRQChannel = USART1_IRQn;
			break;
		case COMM_PORT_2:
			nvicInitStruct.NVIC_IRQChannel = USART2_IRQn;
			break;
		case COMM_PORT_3:
			nvicInitStruct.NVIC_IRQChannel = USART3_IRQn;
			break;
		case COMM_PORT_4:
			nvicInitStruct.NVIC_IRQChannel = UART4_IRQn;
			break;
		case COMM_PORT_5:
			nvicInitStruct.NVIC_IRQChannel = UART5_IRQn;
			break;
		case COMM_PORT_6:
			nvicInitStruct.NVIC_IRQChannel = USART6_IRQn;
			break;
		case NO_PORT:
			break;
	}

	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = iPreempPriority;
	nvicInitStruct.NVIC_IRQChannelSubPriority = iSubPriority;
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvicInitStruct);
}
