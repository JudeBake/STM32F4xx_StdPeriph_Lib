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
#include "SerialPinAssignment.h"

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
uint16_t setAsSimplexRx(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
uint16_t setAsSimplexTx(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
uint16_t setAsFullDuplex(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
void setAsNoHwFlowCtrl(USART_InitTypeDef &iUsartInitStruct);
uint16_t setAsHwFlowCtrlRTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
uint16_t setAsHwFlowCtrlCTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
uint16_t setAsHwFlowCtrlRTSCTS(GPIO_InitTypeDef &iGpioInitStruct,
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
	uint16_t pinSourceUsed = 0;

	//enable USART clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//setting up the link mode
	gpioInitStruct.GPIO_Pin = 0;
	switch (iLinkMode)
	{
		case SIMPLEX_RX:
			pinSourceUsed = pinSourceUsed | setAsSimplexRx(gpioInitStruct,
					usartInitStruct, COMM_PORT_1);
			break;
		case SIMPLEX_TX:
			pinSourceUsed = pinSourceUsed | setAsSimplexTx(gpioInitStruct,
					usartInitStruct, COMM_PORT_1);
			break;
		case FULL_DUPLEX:
			pinSourceUsed = pinSourceUsed | setAsFullDuplex(gpioInitStruct,
					usartInitStruct, COMM_PORT_1);
			break;
	}

	//setting up the hardware flow control
	switch (iHwFlowCtrl)
	{
		case NO_HW_FLOW_CTRL:
			setAsNoHwFlowCtrl(usartInitStruct);
			break;
		case HW_FLOW_CTRL_RTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlRTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_1);
			break;
		case HW_FLOW_CTRL_CTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlCTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_1);
			break;
		case HW_FLOW_CTRL_RTS_CTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlRTSCTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_1);
			break;
	}

	//init the gpio
	GPIO_Init(GPIOA, &gpioInitStruct);
	GPIO_PinAFConfig(GPIOA, pinSourceUsed, GPIO_AF_USART1);

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
	uint16_t pinSourceUsed = 0;

	//enable USART clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	//setting up the link mode
	gpioInitStruct.GPIO_Pin = 0;
	switch (iLinkMode)
	{
		case SIMPLEX_RX:
			pinSourceUsed = pinSourceUsed | setAsSimplexRx(gpioInitStruct,
					usartInitStruct, COMM_PORT_2);
			break;
		case SIMPLEX_TX:
			pinSourceUsed = pinSourceUsed | setAsSimplexTx(gpioInitStruct,
					usartInitStruct, COMM_PORT_2);
			break;
		case FULL_DUPLEX:
			pinSourceUsed = pinSourceUsed | setAsFullDuplex(gpioInitStruct,
					usartInitStruct, COMM_PORT_2);
			break;
	}

	//setting up the hardware flow control
	switch (iHwFlowCtrl)
	{
		case NO_HW_FLOW_CTRL:
			setAsNoHwFlowCtrl(usartInitStruct);
			break;
		case HW_FLOW_CTRL_RTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlRTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_2);
			break;
		case HW_FLOW_CTRL_CTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlCTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_2);
			break;
		case HW_FLOW_CTRL_RTS_CTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlRTSCTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_2);
			break;
	}

	//init the gpio
	GPIO_Init(GPIOD, &gpioInitStruct);
	GPIO_PinAFConfig(GPIOD, pinSourceUsed, GPIO_AF_USART2);

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
	uint16_t pinSourceUsed = 0;

	//enable USART clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//setting up the link mode
	gpioInitStruct.GPIO_Pin = 0;
	switch (iLinkMode)
	{
		case SIMPLEX_RX:
			pinSourceUsed = pinSourceUsed | setAsSimplexRx(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
		case SIMPLEX_TX:
			pinSourceUsed = pinSourceUsed | setAsSimplexTx(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
		case FULL_DUPLEX:
			pinSourceUsed = pinSourceUsed | setAsFullDuplex(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
	}

	//setting up the hardware flow control
	switch (iHwFlowCtrl)
	{
		case NO_HW_FLOW_CTRL:
			setAsNoHwFlowCtrl(usartInitStruct);
			break;
		case HW_FLOW_CTRL_RTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlRTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
		case HW_FLOW_CTRL_CTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlCTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
		case HW_FLOW_CTRL_RTS_CTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlRTSCTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
	}

	//init the gpio
	GPIO_Init(GPIOD, &gpioInitStruct);
	GPIO_PinAFConfig(GPIOB, pinSourceUsed, GPIO_AF_USART3);

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
	uint16_t pinSourceUsed = 0;

	//enable USART clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//setting up the link mode
	gpioInitStruct.GPIO_Pin = 0;
	switch (iLinkMode)
	{
		case SIMPLEX_RX:
			pinSourceUsed = pinSourceUsed | setAsSimplexRx(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
		case SIMPLEX_TX:
			pinSourceUsed = pinSourceUsed | setAsSimplexTx(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
		case FULL_DUPLEX:
			pinSourceUsed = pinSourceUsed | setAsFullDuplex(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
	}

	//setting up the hardware flow control
	switch (iHwFlowCtrl)
	{
		case NO_HW_FLOW_CTRL:
			setAsNoHwFlowCtrl(usartInitStruct);
			break;
		case HW_FLOW_CTRL_RTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlRTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
		case HW_FLOW_CTRL_CTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlCTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
		case HW_FLOW_CTRL_RTS_CTS:
			pinSourceUsed = pinSourceUsed | setAsHwFlowCtrlRTSCTS(gpioInitStruct,
					usartInitStruct, COMM_PORT_3);
			break;
	}

	//init the gpio
	GPIO_Init(GPIOD, &gpioInitStruct);
	GPIO_PinAFConfig(GPIOB, pinSourceUsed, GPIO_AF_USART3);

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

void initPort5(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting)
{

}

void initPort6(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate,
		uint8_t iPreempPriority, uint8_t iSubPriority,
		InterruptSetting iInterruptSetting)
{

}

uint16_t setAsSimplexRx(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
	uint16_t pinSourceUsed = 0;

	switch (iPort)
	{
		case COMM_PORT_1:
			iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | USART1_RX;
			pinSourceUsed = USART1_RX_SOURCE;
			break;
		case COMM_PORT_2:
			iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_6;
			pinSourceUsed = GPIO_PinSource6;
			break;
		case COMM_PORT_3:
			iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_11;
			pinSourceUsed = GPIO_PinSource11;
			break;
		case COMM_PORT_4:
			iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_11;
			pinSourceUsed = GPIO_PinSource11;
			break;
		case COMM_PORT_5:
			iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_2;
			pinSourceUsed = GPIO_PinSource2;
			break;
		case COMM_PORT_6:
			iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_7;
			pinSourceUsed = GPIO_PinSource7;
			break;
		case NO_PORT:
			break;
	}

	iUsartInitStruct.USART_Mode = iUsartInitStruct.USART_Mode | USART_Mode_Rx;

	return pinSourceUsed;
}

uint16_t setAsSimplexTx(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
	uint16_t pinSourceUsed = 0;

	switch (iPort)
	{
	case COMM_PORT_1:
		iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | USART1_TX;
		pinSourceUsed = USART1_TX_SOURCE;
		break;
	case COMM_PORT_2:
		iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_5;
		pinSourceUsed = GPIO_PinSource5;
		break;
	case COMM_PORT_3:
		iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_10;
		pinSourceUsed = GPIO_PinSource10;
		break;
	case COMM_PORT_4:
		iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_10;
		pinSourceUsed = GPIO_PinSource10;
		break;
	case COMM_PORT_5:
		iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_12;
		pinSourceUsed = GPIO_PinSource12;
		break;
	case COMM_PORT_6:
		iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_6;
		pinSourceUsed = GPIO_PinSource6;
		break;
	case NO_PORT:
		break;
	}

	iUsartInitStruct.USART_Mode = iUsartInitStruct.USART_Mode | USART_Mode_Tx;

	return pinSourceUsed;
}

uint16_t setAsFullDuplex(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
	uint16_t pinSourceUsed = 0;

	switch (iPort)
	{
	case COMM_PORT_1:
		iGpioInitStruct.GPIO_Pin =
				iGpioInitStruct.GPIO_Pin | USART1_RX | USART1_TX;
		pinSourceUsed = USART1_RX_SOURCE | USART1_TX_SOURCE;
		break;
	case COMM_PORT_2:
		iGpioInitStruct.GPIO_Pin =
				iGpioInitStruct.GPIO_Pin | GPIO_Pin_6 | GPIO_Pin_5;
		pinSourceUsed = GPIO_PinSource6 | GPIO_PinSource5;
		break;
	case COMM_PORT_3:
		iGpioInitStruct.GPIO_Pin =
				iGpioInitStruct.GPIO_Pin | GPIO_Pin_11 | GPIO_Pin_10;
		pinSourceUsed = GPIO_PinSource11 | GPIO_PinSource10;
		break;
	case COMM_PORT_4:
		iGpioInitStruct.GPIO_Pin =
				iGpioInitStruct.GPIO_Pin | GPIO_Pin_11 | GPIO_Pin_10;
		pinSourceUsed = GPIO_PinSource11 | GPIO_PinSource10;
		break;
	case COMM_PORT_5:
		break;
	case COMM_PORT_6:
		iGpioInitStruct.GPIO_Pin =
				iGpioInitStruct.GPIO_Pin | GPIO_Pin_7 | GPIO_Pin_6;
		pinSourceUsed = GPIO_PinSource7 | GPIO_PinSource6;
		break;
	case NO_PORT:
		break;
	}

	iUsartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	return pinSourceUsed;
}

void setAsNoHwFlowCtrl(USART_InitTypeDef &iUsartInitStruct)
{
		iUsartInitStruct.USART_HardwareFlowControl =
				USART_HardwareFlowControl_None;
}

uint16_t setAsHwFlowCtrlRTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
	uint16_t pinSourceUsed = 0;

		switch (iPort)
		{
			case COMM_PORT_1:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | USART1_RTS;
				pinSourceUsed = USART1_RTS_SOURCE;
				break;
			case COMM_PORT_2:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_4;
				pinSourceUsed = GPIO_PinSource4;
				break;
			case COMM_PORT_3:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_14;
				pinSourceUsed = GPIO_PinSource14;
				break;
			case COMM_PORT_4:
				//TODO: may need implementation for other than STM32F405/07xx
				break;
			case COMM_PORT_5:
				//TODO: may need implementation for other than STM32F405/07xx
				break;
			case COMM_PORT_6:
				//TODO: need device dependent implementation
				break;
			case NO_PORT:
				break;
		}

		iUsartInitStruct.USART_HardwareFlowControl =
				iUsartInitStruct.USART_HardwareFlowControl |
				USART_HardwareFlowControl_RTS;

		return pinSourceUsed;
}

uint16_t setAsHwFlowCtrlCTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
	uint16_t pinSourceUsed = 0;

		switch (iPort)
		{
			case COMM_PORT_1:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | USART1_CTS;
				pinSourceUsed = USART1_CTS_SOURCE;
				break;
			case COMM_PORT_2:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_3;
				pinSourceUsed = GPIO_PinSource3;
				break;
			case COMM_PORT_3:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_13;
				pinSourceUsed = GPIO_PinSource13;
				break;
			case COMM_PORT_4:
				//TODO: may need implementation for other than STM32F405/07xx
				break;
			case COMM_PORT_5:
				//TODO: may need implementation for other than STM32F405/07xx
				break;
			case COMM_PORT_6:
				//TODO: need implementation relative to the device
				break;
			case NO_PORT:
				break;
		}

		iUsartInitStruct.USART_HardwareFlowControl =
				iUsartInitStruct.USART_HardwareFlowControl |
				USART_HardwareFlowControl_CTS;

		return pinSourceUsed;
}

uint16_t setAsHwFlowCtrlRTSCTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
	uint16_t pinSourceUsed;

		switch (iPort)
		{
			case COMM_PORT_1:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_12 | GPIO_Pin_11;
				pinSourceUsed = GPIO_PinSource12 | GPIO_PinSource11;
				break;
			case COMM_PORT_2:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_4 | GPIO_Pin_3;
				pinSourceUsed = GPIO_PinSource4 | GPIO_PinSource3;
				break;
			case COMM_PORT_3:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_14 | GPIO_Pin_13;
				pinSourceUsed = GPIO_PinSource14 | GPIO_PinSource13;
				break;
			case COMM_PORT_4:
				//TODO: may need implementation for other than STM32F405/07xx
				break;
			case COMM_PORT_5:
				//TODO: may need implementation for other than STM32F405/07xx
				break;
			case COMM_PORT_6:
				//TODO: need implementation relative to the device
				break;
			case NO_PORT:
				break;
		}

		iUsartInitStruct.USART_HardwareFlowControl =
				iUsartInitStruct.USART_HardwareFlowControl |
				USART_HardwareFlowControl_RTS_CTS;

		return pinSourceUsed;
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
