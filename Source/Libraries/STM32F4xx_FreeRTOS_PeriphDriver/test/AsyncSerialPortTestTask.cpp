/**
 * \file AsyncSerialPortTestTask.cpp
 *
 * \date Created on: Sep 13, 2013
 * \date Last change on: &DATE&
 *
 * \author Created by: julien 
 * \author Last change by: &AUTHOR&
 *
 * \version Commit Id: &REVISION&
 */

#include "AsyncSerialPortTestTask.h"

typedef enum
{
	CHAR_READ_WRITE,
	STRING_READ_WRITE,
	STRING_OVERLOAD,
} TestingPhase;

void initTestString(int8_t* testString, uint32_t stringLength);
void initOverloadTestStr(int8_t* ovldTestStr, uint32_t stringLength);

AsyncSerialPortTestTask::AsyncSerialPortTestTask()
{
	portInstance = AsyncSerialPort3::getInstance();
	currentStatus = SERIAL_INIT_ERROR;
	testParity = SERIAL_NO_PARITY;
	testStopBit = SERIAL_1_STOP_BIT;
	testDataBit = SERIAL_8_BITS_DATA;
	testHwFlowCtrl = SERIAL_NO_HW_FLOW_CTRL;
	testLinkMode = SERIAL_FULL_DUPLEX;
	testBaudRate = SERIAL_9600_BAUD;
	testInterSetting = SERIAL_INT_ENABLE;

	character = '\0';
	nbCharSent = 0;
}

AsyncSerialPortTestTask::~AsyncSerialPortTestTask()
{
	//close port
	if (portInstance != NULL)
	{
		portInstance->closePort();
	}

	if (IsValid())
	{
		Delete();
	}
}

bool AsyncSerialPortTestTask::HardwareInit()
{
	bool opResult = pdFALSE;

	currentStatus = portInstance->portInit(testParity, testStopBit,
			testDataBit, testHwFlowCtrl, testLinkMode,
			testBaudRate, SERIAL_PORT3_PREEMP_PRIORITY,
			SERIAL_PORT3_SUBPRIORITY, testInterSetting);

	if (currentStatus == SERIAL_OK)
	{
		opResult = pdTRUE;
	}

	return opResult;
}

portBASE_TYPE AsyncSerialPortTestTask::OnCreate(const portCHAR * const pcName,
		unsigned portSHORT usStackDepth,
		unsigned portBASE_TYPE uxPriority)
{
	portBASE_TYPE opResult = pdFALSE;

	if (portInstance->getCurrentStatus() == SERIAL_OK)
	{
		opResult = pdTRUE;
	}

	return opResult;
}


void AsyncSerialPortTestTask::Run(void)
{
	TestingPhase testPhase = CHAR_READ_WRITE;
	uint32_t i;

	initTestString(testTxString, SERIAL_PORT3_BUFFERS_LENGTH);
	initOverloadTestStr(overloadTestStr, SERIAL_PORT3_BUFFERS_LENGTH + 1);

	while (1)
	{
		while (testPhase == CHAR_READ_WRITE)
		{
			for (i = 0; i < 3; i++)
			{
				currentStatus = portInstance->putChar(testTxString[i],
						portMAX_DELAY);
				Delay(100 / portTICK_RATE_MS);
				currentStatus = portInstance->getChar(&character,
						portMAX_DELAY);
			}
		}

		while (testPhase == STRING_READ_WRITE)
		{
			currentStatus = portInstance->putString(testTxString, &nbCharSent);
			Delay(100 / portTICK_RATE_MS);
			i = 0;
			while ((currentStatus = portInstance->getChar(&character, 10)) ==
					SERIAL_OK)
			{
				testRxString[i] = character;
				i++;
			}
		}

		while (testPhase == STRING_OVERLOAD)
		{

		}
	}
}

void initTestString(int8_t* testString, uint32_t stringLength)
{
	int8_t character = 1;
	uint32_t i;

	for (i = 0; i < stringLength - 1; i++)
	{
		testString[i] = character;
		character++;
	}

	testString[i] = '\0';
}

void initOverloadTestStr(int8_t* ovldTestStr, uint32_t stringLength)
{
	int8_t character = 1;
	uint32_t i;

	for (i = 0; i < stringLength - 1; i++)
	{
		ovldTestStr[i] = character;
		character++;
	}

	ovldTestStr[i] = '\0';
}

