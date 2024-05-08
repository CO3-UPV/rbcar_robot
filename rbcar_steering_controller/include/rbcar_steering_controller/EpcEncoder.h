/*! \class EpcEncoder
 *  \file EpcEncoder.h
 *	\author Robotnik Automation S.L.L
 *	\version 1.0
 *	\date 2015
 *  \brief Class to manage the driver DPCANTE of AMC
 *
 * (C) 2015 Robotnik Automation, SLL
 *  All rights reserved.
 */

#ifndef __EPCENCODER_H
	#define __EPCENCODER_H

#include <rbcar_steering_controller/EpcMaCanOpen.h>


#define NODEGUARD_TIME			100				// Time (ms) to control the communication with the driver
#define NODEGUARD_FACTOR		5				// factor to mult
#define HEARTBEAT_TIMER			2000			// milliseconds
#define EVENT_RECOVERY_TIME		1000			// milliseconds
#define NMT_TIMEOUT				2				// Max timeout (seconds) on the communication with the drivers


//! Main function to execute into test secondary thread
void *AuxEpcEncoderControlCommunicationThread(void *threadParam);
//! Main function to execute into the third thread
void *AuxEpcEncoderReadSDOMessagesThread(void *threadParam);

//! Class EpcEncoder
class EpcEncoder: public EpcMaCanOpen{
    friend void *AuxEpcEncoderControlCommunicationThread(void *threadParam);
	friend void *AuxEpcEncoderReadSDOMessagesThread(void *threadParam);

public:


protected:
    //! Can identifier
    byte CanId;
    //! CAN communication status
    CommunicationStatus commStatus;
    //! Time of the last NodeGuard reply from the drive
    struct timespec tNodeGuardReply;
    //! Delay between consecutive CAN transmissions (nanoseconds)
    static const unsigned int uiDelayTrans;
    //! Saves the encoder counts
    int encoder_counts;

    
public:
    //! public constructor
    EpcEncoder(byte can_id, PCan *can_device, double hz);
    //! public constructor
    ~EpcEncoder();
    //! Setups the component
    //! @return OK
    //! @return ERROR
    virtual ReturnValue Setup();
    //! Starts the control thread of the component and its subcomponents
    //! @return OK
    //! @return ERROR starting the thread
    //! @return RUNNING if it's already running
    //! @return NOT_INITIALIZED if it's not initialized
    ReturnValue Start();
	//! Gets the communication status
	CommunicationStatus GetCommunicationStatus();
	//! Gets the communication status like a string
	char *GetCommunicationStatusString();
	//! Puts CAN communication in Operational state
	//! @return OK
	//! @return ERROR
	ReturnValue StartCANCommunication();
	//! Resets the communication control state machine
	//! @return OK
	//! @return ERROR
	ReturnValue ResetCANCommunication();
	//! Stops the communication control state machine to STOPPED state
	//! @return OK
	//! @return ERROR
	ReturnValue StopCANCommunication();
	//! Change the communication control state machine to PRE-OPERATIONAL state
	//! @return OK
	//! @return ERROR
	ReturnValue PreoperationalCANCommunication();
	//! Process received CAN messages
	//! @param msg as a TPCANMsg, the message to process
	void ProcessCANMessage(TPCANMsg msg);

	//! Function to send SYNC message by the CAN network
	ReturnValue Syncronize();
	//! Gets the CAN ID
	byte GetCanId();
	//! Gets the current counts the encoder
	int GetEncoderCounts();
	
protected:
	//! Configures devices and performance of the component
	//! @return OK
    //! @return ERROR, if the configuration process fails
    ReturnValue Configure();
	//! Configures CAN TPDOs and RPDOs
	//! @return OK
    //! @return ERROR, if the configuration process fails
    ReturnValue ConfigureCANMsgs();
	//! Actions performed on initial state
	virtual void InitState();
	//! Actions performed on ready state
	virtual void ReadyState();
	//! Actions performed on restart state	
	virtual void RestartState();
	
	//! Actions performed on the emergency state
	virtual void EmergencyState();
	//! Actions performed in all states
	virtual void AllState();
	//! Change the value of the communication status
	void SwitchToCommunicationStatus(CommunicationStatus new_status);
	//! Process received CAN TPDO1 messages (Default COB-ID= 0x180 + node id).
	//! @param msg as a TPCANMsg, the message to process
	void ProcessTPDO1Msg(TPCANMsg msg);
	//! Process received CAN TPDO2 messages (Default COB-ID= 0x280 + node id).
    //! This type of message contains the following data from Motor Drive:
    //! @param msg as a TPCANMsg, the message to process
    void ProcessTPDO2Msg(TPCANMsg msg);
    //! Process received CAN EMCY messages (Default COB-ID= 0x80 + node id).
    //! @param msg as a TPCANMsg, the message to process
    void ProcessEmergencyMsg(TPCANMsg msg);
	//! Actions to control the communication state with the drive
    void ControlCommunicationThread();
    //! Sleeps for a moment
	void Delay();
	void Delay(unsigned int value);
	//! Reset and initialize the status of the drive
    //! @return OK
    //! @return ERROR
    virtual ReturnValue Initialize();
	//! Reads SDO messages from the device
	void ReadSDOMessagesThread();

};

#endif
