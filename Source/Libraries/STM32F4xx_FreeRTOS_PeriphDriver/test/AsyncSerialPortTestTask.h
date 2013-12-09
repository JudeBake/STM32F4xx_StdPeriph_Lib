/**
 * \file AsyncSerialPortTestTask.h
 * \class AsyncSerialPort1TestTask
 * \brief This managed task is used to test the asynchronous serial ports.
 * 		  Modify and use this task to test the uarts.
 *
 * \date Created on: Sep 13, 2013
 * \date Last change on: &DATE&
 *
 * \author Created by: julien 
 * \author Last change by: &AUTHOR&
 *
 * \version Commit Id: &REVISION&
 */

#ifndef ASYNCSERIALPORTTESTTASK_H_
#define ASYNCSERIALPORTTESTTASK_H_

#include "AManagedTask.h"
#include "AsyncSerialPort3.h"
#include "CQueue.h"
#include "SerialType.h"

class AsyncSerialPortTestTask: public AManagedTask
{
private:
	AsyncSerialPort3* portInstance;
	SerialStatus currentStatus;
	Parity testParity;
	StopBits testStopBit;
	DataBits testDataBit;
	HwFlowCtrl testHwFlowCtrl;
	LinkMode testLinkMode;
	BaudRate testBaudRate;
	InterruptSetting testInterSetting;

	int8_t character;
	int8_t testTxString[SERIAL_PORT3_BUFFERS_LENGTH];
	int8_t testRxString[SERIAL_PORT3_BUFFERS_LENGTH];
	int8_t overloadTestStr[SERIAL_PORT3_BUFFERS_LENGTH + 2];
	uint32_t nbCharSent;

public:
	/*
	 * \brief Default Constructor.
	 */
	AsyncSerialPortTestTask(void);
	AsyncSerialPortTestTask(const AsyncSerialPortTestTask&);
	AsyncSerialPortTestTask& operator=(const AsyncSerialPortTestTask&);

	/*
	 * \brief Default Destructor.
	 */
	virtual ~AsyncSerialPortTestTask(void);

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

#endif /* ASYNCSERIALPORTTESTTASK_H_ */
