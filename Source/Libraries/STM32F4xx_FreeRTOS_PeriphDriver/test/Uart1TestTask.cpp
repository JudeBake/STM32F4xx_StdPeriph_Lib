/**
 * \file Uart1TestTask.cpp
 *
 * \date Created on: Sep 13, 2013
 * \date Last change on: &DATE&
 *
 * \author Created by: julien 
 * \author Last change by: &AUTHOR&
 *
 * \version Commit Id: &REVISION&
 */

#include "Uart1TestTask.h"

Uart1TestTask::Uart1TestTask()
{
	portInstance = NULL;
	currentStatus = SERIAL_INIT_ERROR;
	testParity = SERIAL_NO_PARITY;
	testStopBit = SERIAL_1_STOP_BIT;
	testDataBit = SERIAL_8_BITS_DATA;
	testHwFlowCtrl = SERIAL_NO_HW_FLOW_CTRL;
	testLinkMode = SERIAL_SIMPLEX_RX;
	testBaudRate = SERIAL_9600_BAUD;
	testInterSetting = SERIAL_INT_ENABLE;

	character = "\0";
}

Uart1TestTask::~Uart1TestTask()
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

bool Uart1TestTask::HardwareInit()
{
	bool opResult = pdFALSE;

	currentStatus = portInstance->portInit(testParity, testStopBit,
			testDataBit, testHwFlowCtrl, testLinkMode,
			testBaudRate, SERIAL_PORT1_PREEMP_PRIORITY,
			SERIAL_PORT1_SUBPRIORITY, testInterSetting);

	if (currentStatus == SERIAL_OK)
	{
		opResult = pdTRUE;
	}

	return opResult;
}

portBASE_TYPE Uart1TestTask::OnCreate(const portCHAR * const pcName,
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


void Uart1TestTask::Run(void)
{
	while (1)
	{

	}
}

