/*
 * Serial.cpp
 *
 *  Created on: Apr 24, 2013
 *      Author: julien
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

//private function
void initPort1(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate);
void initPort2(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate);
void initPort3(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate);
void initPort4(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate);
void initPort5(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate);
void initPort6(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate);
uint16_t setAsSimplexRx(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
uint16_t setAsSimplexTx(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
uint16_t setAsFullDuplex(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
void setAsNoHwFlowCtrl(USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
uint16_t setAsHwFlowCtrlRTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
uint16_t setAsHwFlowCtrlCTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
uint16_t setAsHwFlowCtrlRTSCTS(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort);
void setWordLength(USART_InitTypeDef &iUsartInitStruct, DataBits iWordLength);
void setStopBits(USART_InitTypeDef &iUsartInitStruct, StopBits iStopBit);
void setParity(USART_InitTypeDef &iUsartInitStruct, Parity iParity);
void setNvic(COMMPort iPort, uint8_t iPreampPriority, uint8_t iSubPriority);

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
}

AsyncSerial::AsyncSerial(COMMPort iPort, Parity iParityConf,
		StopBits iStopBitConf, DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl,
		LinkMode iLinkMode, BaudRate iBaudRateConf)
{
	//init the port setting
	commPort = iPort;
	parity = iParityConf;
	stopBits = iStopBitConf;
	dataBits = iDataLengthConf;
	hwFlowCtrl = iHwFlowCtrl;
	linkMode = iLinkMode;
	baudRate = iBaudRateConf;

	//init the port queues
	dataStreamIn.Create(STRING_BUFFER_LENGTH, sizeof(int8_t));
	dataStreamOut.Create(STRING_BUFFER_LENGTH, sizeof(int8_t));

	if ((dataStreamIn != NULL) && (dataStreamOut != NULL))
	{
		switch (iPort)
		{
			case COMM_PORT_1:
				initPort1(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate);
				break;
			case COMM_PORT_2:
				initPort2(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate);
				break;
			case COMM_PORT_3:
				initPort3(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate);
				break;
			case COMM_PORT_4:
				initPort4(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate);
				break;
			case COMM_PORT_5:
				initPort5(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate);
				break;
			case COMM_PORT_6:
				initPort6(parity, stopBits, dataBits, hwFlowCtrl, linkMode,
						baudRate);
				break;
			case NO_PORT:
				break;
		}
	}
}

void initUsartStruct(USART_InitTypeDef &iUsartInitStruct)
{

}

void nvicInit(void)
{
	NVIC_InitTypeDef nvicInitStruct;

	nvicInitStruct.NVIC_IRQChannel = USART1_IRQn;
	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = configLIBRARY_LOWEST_INTERRUPT_PRIORITY;
	nvicInitStruct.NVIC_IRQChannelSubPriority = 0;
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicInitStruct);
}

void initPort1(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	GPIO_InitTypeDef gpioInitStruct;

	//enable USART clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//setting up the GPIOs
	gpioInitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9;

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	//setting up the USART
	usartInitStruct.USART_BaudRate = iBaudRate;
	setWordLength(usartInitStruct, iDataLength);
	setStopBits(usartInitStruct, iStopBit);
	setParity(usartInitStruct, iParity);

	usartClockInitStruct.USART_Clock = USART_Clock_Disable;
	usartClockInitStruct.USART_CPOL = USART_CPOL_Low;
	usartClockInitStruct.USART_CPHA = USART_CPHA_2Edge;
	usartClockInitStruct.USART_LastBit = USART_LastBit_Disable;

	USART_Init(USART1, &usartInitStruct);
	USART_ClockInit(USART1, &usartClockInitStruct);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART1, ENABLE);
}

void initPort2(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	GPIO_InitTypeDef gpioInitStruct;
}

void initPort3(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	GPIO_InitTypeDef gpioInitStruct;
}

void initPort4(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	GPIO_InitTypeDef gpioInitStruct;
}

void initPort5(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	GPIO_InitTypeDef gpioInitStruct;
}

void initPort6(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	NVIC_InitTypeDef nvicInitStruct;

}

uint16_t setAsSimplexRx(GPIO_InitTypeDef &iGpioInitStruct,
		USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
{
	uint16_t pinSourceUsed = 0;

	switch (iPort)
	{
		case COMM_PORT_1:
			iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_10;
			pinSourceUsed = GPIO_PinSource10;
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
		iGpioInitStruct.GPIO_Pin = iGpioInitStruct.GPIO_Pin | GPIO_Pin_9;
		pinSourceUsed = GPIO_PinSource9;
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
				iGpioInitStruct.GPIO_Pin | GPIO_Pin_10 | GPIO_Pin_9;
		pinSourceUsed = GPIO_PinSource10 | GPIO_PinSource9;
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
	case COMM_PORT_3:
		iGpioInitStruct.GPIO_Pin =
				iGpioInitStruct.GPIO_Pin | GPIO_Pin_11 | GPIO_Pin_10;
		pinSourceUsed = GPIO_PinSource11 | GPIO_PinSource10;
		break;
	case COMM_PORT_5:
		break;
	case COMM_PORT_6:
	case COMM_PORT_3:
		iGpioInitStruct.GPIO_Pin =
				iGpioInitStruct.GPIO_Pin | GPIO_Pin_11 | GPIO_Pin_10;
		pinSourceUsed = GPIO_PinSource11 | GPIO_PinSource10;
		break;
	case NO_PORT:
		break;
	}

	iUsartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	return pinSourceUsed;
}

void setAsNoHwFlowCtrl(USART_InitTypeDef &iUsartInitStruct, COMMPort iPort)
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
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_12;
				pinSourceUsed = GPIO_PinSource12;
				break;
			case COMM_PORT_2:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_4;
				pinSourceUsed = GPIO_PinSource4;
				break;
			case COMM_PORT_3:
				//TODO: may need implementation for other than STM32F405/07xx
				break;
			case COMM_PORT_4:
				//TODO: may need implementation for other than STM32F405/07xx
				break;
			case COMM_PORT_5:
				//TODO: may need implementation for other than STM32F405/07xx
				break;
			case COMM_PORT_6:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_8;
				pinSourceUsed = GPIO_PinSource8;
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
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_11;
				pinSourceUsed = GPIO_PinSource11;
				break;
			case COMM_PORT_2:
				iGpioInitStruct.GPIO_Pin =
						iGpioInitStruct.GPIO_Pin | GPIO_Pin_3;
				pinSourceUsed = GPIO_PinSource3;
				break;
			case COMM_PORT_3:
				//TODO: may need implementation for other than STM32F405/07xx
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
				//TODO: may need implementation for other than STM32F405/07xx
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

void setNvic(COMMPort iPort, uint8_t iPreampPriority, uint8_t iSubPriority)
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

	nvicInitStruct.NVIC_IRQChannelPreemptionPriority = iPreampPriority;
	nvicInitStruct.NVIC_IRQChannelSubPriority = iSubPriority;
	nvicInitStruct.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvicInitStruct);
}
