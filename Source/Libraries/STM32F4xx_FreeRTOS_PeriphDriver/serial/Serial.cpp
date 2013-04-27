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
#include "Serial.h"

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
void setGpio(HwFlowCtrl iHwFlowCtrl, LinkMode iMode, COMMPort iPort);
void setNvic(COMMPort iPort, uint8_t iPrempPriority);
void setWordLength(USART_InitTypeDef &iUsartInitStruct, DataBits iWordLength);
void setStopBits(USART_InitTypeDef &iUsartInitStruct, StopBits iStopBit);
void setParity(USART_InitTypeDef &iUsartInitStruct, Parity iParity);
void setHwFlowCtrl(USART_InitTypeDef &iUsartInitStruct, HwFlowCtrl iHwFlowCtrl);
void setLinkMode(USART_InitTypeDef &iUsartInitStruct, LinkMode iLinkMode);

Serial::Serial()
{
	//init the port setting
	commPort = NO_PORT;
	parity = NO_PARITY;
	stopBits = STOP_BIT_1;
	dataBits = DATA_8_BITS;
	hwFlowCtrl = NO_HW_FLOW_CTRL;
	baudRate = BAUD_19200;
}

Serial::Serial(COMMPort iPort, Parity iParityConf, StopBits iStopBitConf,
		DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode,
		BaudRate iBaudRateConf)
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
	NVIC_InitTypeDef nvicInitStruct;
	GPIO_InitTypeDef gpioInitStruct;

	//enable USART clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//setting up the GPIOs
	gpioInitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_9;
	gpioInitStruct.GPIO_Mode = GPIO_Mode_AF;
	gpioInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	gpioInitStruct.GPIO_OType = GPIO_OType_PP;
	gpioInitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &gpioInitStruct);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	//setting up the USART
	usartInitStruct.USART_BaudRate = iBaudRate;
	setWordLength(usartInitStruct, iDataLength);
	setStopBits(usartInitStruct, iStopBit);
	setParity(usartInitStruct, iParity);
	usartInitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usartInitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
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
	NVIC_InitTypeDef nvicInitStruct;
	GPIO_InitTypeDef gpioInitStruct;
}

void initPort3(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	NVIC_InitTypeDef nvicInitStruct;
	GPIO_InitTypeDef gpioInitStruct;
}

void initPort4(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	NVIC_InitTypeDef nvicInitStruct;
	GPIO_InitTypeDef gpioInitStruct;
}

void initPort5(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	NVIC_InitTypeDef nvicInitStruct;
	GPIO_InitTypeDef gpioInitStruct;
}

void initPort6(Parity iParity, StopBits iStopBit, DataBits iDataLength,
		HwFlowCtrl iHwFlowCtrl, LinkMode iLinkMode, BaudRate iBaudRate)
{
	USART_InitTypeDef usartInitStruct;
	USART_ClockInitTypeDef usartClockInitStruct;
	NVIC_InitTypeDef nvicInitStruct;
	GPIO_InitTypeDef gpioInitStruct;
}

void setGpio(HwFlowCtrl iHwFlowCtrl, LinkMode iMode, COMMPort iPort)
{

}

void setNvic(COMMPort iPort, uint8_t iPrempPriority)
{

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

void setHwFlowCtrl(USART_InitTypeDef &iUsartInitStruct, HwFlowCtrl iHwFlowCtrl)
{

}

void setLinkMode(USART_InitTypeDef &iUsartInitStruct, LinkMode iLinkMode)
{

}
