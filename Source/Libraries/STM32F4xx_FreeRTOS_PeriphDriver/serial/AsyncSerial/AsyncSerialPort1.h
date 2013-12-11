/**
 * \file AsyncSerialPort1.h
 * \class AsyncSerialPort1
 * \brief Singleton of the asynchronous port 1.
 *
 * This class is the second level driver of the serial port 1 in asynchronous
 * mode. It uses the CQueue framework of the FreeRTOS and FreeRTOS C++ wrapper
 * as Rx/Tx buffers. It uses the CMutex framework of the FreeRTOS and FreeRTOS
 * C++ wrapper to ensures that the port will not be used by more than one task
 * at the time.
 *
 * \date	Created on: May 1, 2013
 * \date	Last change on: &DATE&
 *
 * \author	Created by: julien
 * \author	Last change by: &AUTHOR&
 *
 * \version	Commit Id: &REVISION&
 */

#ifndef ASYNCSERIAL1_H_
#define ASYNCSERIAL1_H_

#include "CQueue.h"
#include "CMutex.h"
#include "SerialType.h"

/**
 * \def SERIAL_PORT1_BUFFERS_LENGTH
 * \brief The length of the Rx/Tx buffers of the port.
 */
#define SERIAL_PORT1_BUFFERS_LENGTH		100

/**
 * \def SERIAL_PORT1_PREEMP_PRIORITY
 * \brief The preemptive priority of the port.
 */
#define SERIAL_PORT1_PREEMP_PRIORITY	15

/**
 * \def SERIAL_PORT1_SUBPRIORITY
 * \brief The sub-priority of the port.
 */
#define SERIAL_PORT1_SUBPRIORITY		0

class AsyncSerialPort1
{
public:
	/* Port handle */
	static AsyncSerialPort1* portHandle; /**<Handle of the port. */

private:
	/* Port configuration */
	SerialStatus currentStatus;
	Parity parity;
	StopBits stopBits;
	DataBits dataBits;
	HwFlowCtrl hwFlowCtrl;
	LinkMode linkMode;
	BaudRate baudRate;
	uint8_t preempPriority;
	uint8_t subPriority;
	InterruptSetting interruptSetting;

	/* Data stream Queues */
	CQueue dataStreamIn;
	CQueue dataStreamOut;

	/* Port Mutex */
	CMutex portMutex;

	/* Constructors */
	AsyncSerialPort1(void);
	AsyncSerialPort1(const AsyncSerialPort1&);
	void operator=(const AsyncSerialPort1&);

	/* Destructor */
	~AsyncSerialPort1(void){}

public:
	/* Instance getter */
	/**
	 * \brief This method give access to the port by returning its handle.
	 *
	 * If the port is not already created, this method creates it. At the port
	 * creation, the port is not initialized. So it is a good practice to verify
	 * its status after getting its instance.
	 *
	 * \return The port handle.
	 */
	static AsyncSerialPort1* getInstance(void);

	/* Serial port initializer */
	/**
	 * \brief This method initialize the port according to the desired settings
	 *
	 * \param iParityConf The parity configuration of the port.
	 * \param iStopBitConf The stop bit configuration of the port.
	 * \param iDataLengthConf The length of the word used by the port.
	 * \param iHwFlowCtrl The hardware flow control configuration of the port.
	 * \param iLinkMode The type of link of the port.
	 * \param iBaudRateConf The baud rate of the port.
	 * \param iPreempPriority The preemptive priority of the port.
	 * \param iSubPriority The sub-priority of the port.
	 * \param iInterruptSetting If the interrupt should be enabled or disabled.
	 */
	SerialStatus portInit(Parity iParityConf, StopBits iStopBitConf,
			DataBits iDataLengthConf, HwFlowCtrl iHwFlowCtrl,
			LinkMode iLinkMode, BaudRate iBaudRateConf,
			uint8_t iPreempPriority, uint8_t iSubPriority,
			InterruptSetting iInterruptSetting);

	/* Current status getter */
	/**
	 * \brief This method enable the user to interrogate the port for its
	 *        current status.
	 *
	 * \return The port current status.
	 */
	SerialStatus getCurrentStatus(void);

	/* Current status setter ***USED ONLY IN INTERRUPT HANDLER*** */
	/**
	 * \brief ***This method is USED ONLY BY THE INTERRUPT HANDLER***
	 */
	void setCurrentStatus(SerialStatus iStatus);

	/* Stream getters ***USED ONLY IN INTERRUPT HANDLER*** */
	/**
	 * \brief ***This method is USED ONLY BY THE INTERRUPT HANDLER***
	 */
	CQueue& getOutStream(void);

	/**
	 * \brief ***This method is USED ONLY BY THE INTERRUPT HANDLER***
	 */
	CQueue& getInStream(void);

	/* Port reading method */
	/**
	 * \brief This method enables to read the received data on the port.
	 *
	 * The reading is made one character at the time.
	 *
	 * \param oCharacter The pointer of the buffer used to read the received
	 * 					 character (return of the received character).
	 * \param iBlockTime The time the Rx buffer of the port will block tasks if
	 * 					 it is empty.
	 *
	 * \return The current status of the port.
	 */
	SerialStatus getChar(const int8_t* oCharacter, portTickType iBlockTime);

	/* Port writing methods */
	/**
	 * \brief This method enables to write data on the port.
	 *
	 * The writing is made one character at the time.
	 *
	 * \param iCharacter The character to write on the port.
	 * \param iBlockTime The time the Tx buffer of the port will block tasks if
	 * 					 it is full.
	 *
	 * \return The current status of the port.
	 */
	SerialStatus putChar(const int8_t iCharacter, portTickType iBlockTime);

	/**
	 * \brief This method enables to write data on the port.
	 *
	 * The writing is made one string at the time.
	 *
	 * \param iString The pointer of the beginning of the string to write.
	 * \param oNbCharSent The pointer of the variable used to store the number
	 * 					  of characters written in case the Tx buffer fills up
	 * 					  before the end of the string (return the number of
	 * 					  characters sent).
	 *
	 * \return The current status of the port.
	 */
	SerialStatus putString(const int8_t* const iString, uint32_t* oNbCharSent);

	/* Mutex method */
	/**
	 * \brief This method enables to take the mutex of the port.
	 *
	 * \param iBlockTime The time the port will block tasks until the mutex is
	 * 					 freed.
	 *
	 * \return 1 if the mutex was taken, 0 otherwise.
	 */
	portBASE_TYPE takeMutex(portTickType iBlockTime);

	/**
	 * \brief This method enables to give the mutex of the port.
	 *
	 * \return 1 if the mutex was given, 0 otherwise.
	 */
	portBASE_TYPE giveMutex(void);

	/* Closing port method */
	/**
	 * \brief This method close the port.
	 *
	 * Closing the port will delete it and all its resources.
	 */
	static void closePort(void);
};

#endif /* ASYNCSERIAL1_H_ */
