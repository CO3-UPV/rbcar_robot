/*! \class PCan
 *  \file peak_can.cc
 *	\author Robotnik Automation S.L.L
 *	\version 1.0
 *	\date 2014
 *  \brief Class to manage the CAN communication using an PEAK CAN device and the CANOpen protocol
 * (C) 2014 Robotnik Automation, SLL
 *  All rights reserved.
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>   // exit
#include <signal.h>
#include <string.h>
#include <stdlib.h>   // strtoul
#include <fcntl.h>    // O_RDWR
#include <unistd.h>

#include <libpcan.h>
#include <ctype.h>

#include <curtis_controller/peak_can.h>

//****************************************************************************
// CODE

/*! \fn PCan::PCan(NetChannel net_channel, BaudRate baudrate)
 	* Constructor by default
    \param net_channel as int, CAN channel
    \param baudrate as int, baudrate for the communication
*/
PCan::PCan(uint16_t baudrate){
	wBTR0BTR1 = baudrate;
	//iSentMsgs = iReadMsgs = 0;
	sDevice.assign(DEFAULT_NODE);
	// mutex intitialization
	pthread_mutex_init(&mutexCAN, NULL);
	
	initialized = false;
};

/*! \fn PCan::PCan(uint16_t baudrate, const char *dev)
 	* Constructor by default
    \param dev as char *, name of the CAN device
    \param baudrate as int, baudrate for the communication
*/
PCan::PCan(string dev, uint16_t baudrate){

	wBTR0BTR1 = baudrate;
	//iSentMsgs = iReadMsgs = 0;
	sDevice = dev;
	// mutex intitialization
	pthread_mutex_init(&mutexCAN, NULL);
	
	initialized = false;
};


/*! \fn PCan::~PCan()
  * Destructor by default
*/
PCan::~PCan(){
        pthread_mutex_destroy(&mutexCAN);
}


/*! \fn int PCan::setup()
 * 
 * \return OK
 * \return ERROR
 * \return INITIALIZED
*/
int PCan::setup(){
	if(initialized){
		ROS_INFO("PCAN::Setup: Already initialized");
		
		return INITIALIZED;
	}
	
	int ret = OK;
	//
	///////////////////////////////////////////////////
	// Setups the component or another subcomponents if it's necessary //
	///////////////////////////////////////////////////
	pthread_mutex_lock(&mutexCAN);
	
	if(open() == OK)
		initialized = true;
	else
		ret = ERROR;
	
	pthread_mutex_unlock(&mutexCAN);
	
	return ret;
}


/*! \fn int PCan::shutDown()
 * Closes and frees the reserved resources
 * \return OK
 * \return ERROR if fails when closes the devices
 * \return NOT_INITIALIZED if the component is not initialized
*/
int PCan::shutdown(){

	if(!initialized){
		ROS_INFO("PCAN::Shutdown: Impossible because of it's not initialized");
		return NOT_INITIALIZED;
	}
	int ret = OK;
	
	//
	///////////////////////////////////////////////////////
	// ShutDowns another subcomponents if it's necessary //
	///////////////////////////////////////////////////////
	pthread_mutex_lock(&mutexCAN);
		
		if(shutdown() == OK)
			initialized = false;
		else
			ret = ERROR;
			
	pthread_mutex_unlock(&mutexCAN);
	
	

	return ret;
}


/*! \fn int PCan::open()
 * open device to be used
 * \return OK
 * \return ERROR
*/
int PCan::open(){

	// open CAN port
	//const char *szDevNode = DEFAULT_NODE;
	// O_NONBLOCK - non blocking read 
	// O_RDWR - blocking read - function returns only if there is something in the buffer to read
	h = LINUX_CAN_Open(sDevice.c_str(), O_RDWR | O_NONBLOCK);
	if (!h) {
		ROS_ERROR("PCan::open: can't open %s\n", sDevice.c_str());
		return ERROR;
	}
	// clear status
	CAN_Status(h);
	// get version info
	errno = CAN_VersionInfo(h, txt);
	if (!errno) {
		ROS_INFO("PCan::open: driver version = %s", txt);
	}else {
		ROS_ERROR("PCan::open: CAN_VersionInfo() failed");
		return ERROR;
	}

	// init to a user defined bit rate
	errno = CAN_Init(h, wBTR0BTR1, CAN_INIT_TYPE_ST); // _ST:standard frames, _EX:extended frames 
	if (errno) {
		ROS_ERROR("PCan::configure: CAN_Init() failed");
		return ERROR;
	}
  
	
  
	ROS_INFO("PCan::open: Opened successfully at %s", sDevice.c_str());
	
	return OK;
}



/*! \fn int PCan::close()
 * Closes used devices
 * \return OK
 * \return ERROR
*/
int PCan::close(){

	
	
	if(h)
	{
		CAN_Close(h);
	}
	
	ROS_INFO("PCan:close finished.");
	sStatus.assign("CLOSED");
	
	
	return OK;
}

/*! \fn int PCan::send(TPCANMsg *cmsg)
 * Sends CAN messages
 * \param cmsg as TPCANMsg*, pointer to data buffer of messages
 * \param len as int*, IN: number of messages to be transmitted
 *                     OUT: number of transmitted messages
 * \return OK
 * \return ERROR
*/
int PCan::send(TPCANMsg *tpcmsg){

	static bool bPrintSend = false;
	
	if(!initialized){
		if(!bPrintSend){
			ROS_ERROR("PCan::Read: Impossible to read because of it's not initialized");
			bPrintSend = true;
		}
		return NOT_INITIALIZED;
	}

	pthread_mutex_lock(&mutexCAN);

	// Write the message to the CAN bus
	CAN_Write( h, tpcmsg );
	int iRet = CAN_Status(h);
	if(iRet < 0)
	{
		// 
		int err = nGetLastError();
		ROS_ERROR("PCan::send error in CAN_Write() errno=%d iRet=%d nGetLastError=%d", errno, iRet, err );
		pthread_mutex_unlock(&mutexCAN);
		return ERROR;
	}

	// ROS_INFO("PCan::send WRITE OK !!!!!!!!!!!!!!! ");

	// Possible wait between consecutive sends
	pthread_mutex_unlock(&mutexCAN);
	bPrintSend = false;
	return OK;
}



/*! \fn int PCan::read(TPCANMsg *tpcmsg)
 * Read CAN messages
 * \param tpcmsg as TPCANRdMsg*, pointer to data buffer of messages
 * \param len as int*, IN: size of the TPCANRdMsg buffer
 *                     OUT: number of read messages
 * \return OK
 * \return ERROR
*/
int PCan::read(TPCANRdMsg *tpcmsg){

    int last_err = 0;

    if(!initialized){
		ROS_INFO("PCan::Read: Impossible to read because of it's not initialized");
		return NOT_INITIALIZED;
	}
	int ret = OK;
	
    pthread_mutex_lock(&mutexCAN);

    int iRet = LINUX_CAN_Read(h, tpcmsg);
    
    if (iRet == CAN_ERR_OK){  // Read OK
	
        if (tpcmsg->Msg.MSGTYPE != MSGTYPE_STANDARD) {
			//ROS_INFO("PCan::Read: 0");
			ret = NOT_ERROR;
		}
	}else if(tpcmsg->Msg.MSGTYPE == MSGTYPE_STATUS){  
		if (iRet < 0) {
			last_err = nGetLastError();
			ROS_ERROR("PCan::Read: error in LINUX_CAN_Read() - MSGTYPE_STATUS - err=%x", last_err);
			
			ret = ERROR;                 
		}else	if (iRet & CAN_ERR_QRCVEMPTY){
			// Error (Don't output error of empty queue) (not an error)              
			// ROS_INFO((char *) "PCan::Read: empty queue ");
			 
			// TODO add input in int Component.h ??
			ret = NOT_ERROR; // Buffer empty 
			//ROS_INFO("PCan::Read: 1");
		}
	}else{

		// Este caso no esta claro : last_err da 11, iRet -1 y son tramas de longitud 4, todo a 0
		//  id=1, len=4, data[ 0 0 0 0 0 0 0 0 ]
		if (iRet & CAN_ERR_QRCVEMPTY) {
		   ret = NOT_ERROR;   
		   //ROS_INFO("PCan::Read: 2. iret = %d", iRet);
		}else{

			last_err = nGetLastError();
			ROS_ERROR("PCan::Read: error in LINUX_CAN_Read, iRet=%d  err=%d", iRet, last_err);       
			// For DEBUG
			readDiagnostics(); 
			
			ret = ERROR;
			//ROS_INFO("PCan::Read: 3");
		}
	}
	
	pthread_mutex_unlock(&mutexCAN);
	
	return ret;


}

/*! \fn int PCan::readDiagnostics()
 * Call diagnostics function of the CAN device
 *     OUT: number of read messages
 * \return OK
 * \return ERROR
*/
int PCan::readDiagnostics()
{
  int err;
  TPDIAG diag;
  
  err = LINUX_CAN_Statistics(h, &diag);
  if (err) {
    ROS_ERROR("PCan::LogDiagnostics: can't read diagnostics, error=%d", err);
    return ERROR;
    }
  else {
    ROS_INFO("*****************************************");
    ROS_INFO("Serial Number = 0x%08x  Device Number = %d", diag.dwBase, diag.wIrqLevel);
    ROS_INFO("reads=%d,  writes=%d", diag.dwReadCounter, diag.dwWriteCounter);
    ROS_INFO("errors=%d,  last CAN status = 0x%04x", diag.dwErrorCounter, diag.wErrorFlag);
    // diag.nLastError;
    // diag.nOpenPaths;
    // diag.szVersionString;    
    }  
  return OK;
}

