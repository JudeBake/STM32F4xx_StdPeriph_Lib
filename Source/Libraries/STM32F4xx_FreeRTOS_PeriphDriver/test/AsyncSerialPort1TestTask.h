/**
 * \file AsyncSerialPort1TestTask.h
 *
 * \brief This managed task is used to test the asynchronous serial port 1.
 * 		  Modify and use this task to test the uart 1.
 *
 * \date Created on: Sep 13, 2013
 * \date Last change on: &DATE&
 *
 * \author Created by: julien 
 * \author Last change by: &AUTHOR&
 *
 * \version Commit Id: &REVISION&
 */

#ifndef UART1TESTTASK_H_
#define UART1TESTTASK_H_

#include "AManagedTask.h"
#include "AsyncSerialPort1.h"
#include "CQueue.h"
#include "SerialType.h"

class AsyncSerialPort1TestTask: public AManagedTask
{
private:
	AsyncSerialPort1* portInstance;
	SerialStatus currentStatus;
	Parity testParity;
	StopBits testStopBit;
	DataBits testDataBit;
	HwFlowCtrl testHwFlowCtrl;
	LinkMode testLinkMode;
	BaudRate testBaudRate;
	InterruptSetting testInterSetting;

	int8_t character;
	int8_t string[SERIAL_PORT1_BUFFERS_LENGTH];

public:
	/*
	 * \brief Default Constructor.
	 */
	AsyncSerialPort1TestTask(void);
	AsyncSerialPort1TestTask(const AsyncSerialPort1TestTask&);
	AsyncSerialPort1TestTask& operator=(const AsyncSerialPort1TestTask&);

	/*
	 * \brief Default Destructor.
	 */
	virtual ~AsyncSerialPort1TestTask(void);

	/*
	 * \brief The hardware initialization of the test task. The input and
	 * 		  output stream buffer are also created here.
	 */
	bool HardwareInit(void);

	/*
	 * \brief The verification that everything is good before the task is
	 * 		  created.
	 *
	 * \param pcName specifies the task name.
	 * \param usStackDepth specifies the task stack depth.
	 * \param uxPriority specifies the task priority.
	 *
	 * \return pdTRUE if success, pdFALSE otherwise. If the method return
	 * 		   pdFALSE the task creation process is stopped and no FreeRTOS
	 * 		   resource are allocated.
	 */
	portBASE_TYPE OnCreate(const portCHAR * const pcName,
						   unsigned portSHORT usStackDepth,
						   unsigned portBASE_TYPE uxPriority);

	/*
	 * \brief The task process.
	 */
	void Run(void);
};

#endif /* UART1TESTTASK_H_ */
