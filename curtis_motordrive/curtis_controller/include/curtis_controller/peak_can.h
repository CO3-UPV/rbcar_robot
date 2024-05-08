/*! \class PCan
 *  \file peak_can.h
 *	\author Robotnik Automation S.L.L
 *	\version 1.0
 *	\date 2010
 *  \brief Class to manage the CAN communication using a PEAK CAN device and the CANOpen protocol
 * (C) 2012 Robotnik Automation, SLL
 *  All rights reserved.
 */

#ifndef __PCAN_H
	#define __PCAN_H

#include <pcan.h>
#include <libpcan.h> 
#include <ros/ros.h>

//#define DEFAULT_NODE "/dev/pcanusb0"
#define DEFAULT_NODE "/dev/pcan32"

//extern HANDLE h;
typedef unsigned char byte;

using namespace std;

//! Defines return values for methods and functions
enum ReturnValue{
	OK = 0,
	INITIALIZED,
	THREAD_RUNNING,
	ERROR = -1,
	NOT_INITIALIZED = -2,
	THREAD_NOT_RUNNING = -3,
	COM_ERROR = -4,
	NOT_ERROR = -5
};


//! Class to communicate with an Peak CAN device
class PCan{

	private:
    		//! CAN device handle
		HANDLE h;
		//! Mutex to manage the access to the device
		pthread_mutex_t mutexCAN;
	    //! Baud rate used in the communication
		uint16_t wBTR0BTR1;
		//! Description of the device status
		string sStatus;
		//! Version string
		char txt[VERSIONSTRING_LEN];
		//! Device
		string sDevice;
		//! Flag to control the initialization
		bool initialized;

	public:
    		//! constructor
		PCan(uint16_t baudrate);
		//! constructor
		PCan(string dev, uint16_t baudrate);
		//! Destructor
		virtual ~PCan();
		//! Sends CAN messages
		int send(TPCANMsg *tpcmsg);
		//! Read CAN messages
		int read(TPCANRdMsg *tpcmsg);
		//! Setups the component (mandatory to work with it)
		int setup();
		//! Shutdowns the component
		int shutdown();


	protected:
		
		//! Open device to be used 
		int open();
		//! Closes used devices
	    int close();	
		//! Read Diagnostics of the PCan board
		int readDiagnostics();
	
};

#endif



