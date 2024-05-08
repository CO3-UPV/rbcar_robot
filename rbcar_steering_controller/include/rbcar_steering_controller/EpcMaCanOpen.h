/*! \class EpcMaCanOpen
 *  \file EpcMaCanOpen.h
 *	\author Robotnik Automation S.L.L
 *	\version 1.0
 *	\date 2012
 *  \brief Class to manage the communication with the driver dzcante-020l080 via CANOpen
 * (C) 2012 Robotnik Automation, SLL
 *  All rights reserved.
 */

#ifndef __EPCMACANOPEN_H
	#define __EPCMACANOPEN_H

#include "rbcar_steering_controller/PCan.h"


//! Class Dzcante020l080
class EpcMaCanOpen: public Component{
    public:
    //! Types of CAN messages
    enum CANMessage{
        NODEGUARD_MSG,
        SYNC_MSG,
		START_COMM_MSG,
        RESET_COMM_MSG,
        RESET_NODE_MSG,
		STOP_COMM_MSG,
		PREOPERATIONAL_COMM_MSG,
		GUARD_TIME_MSG,
		HEARTBEAT_CONSUMER_MSG,
		HEARTBEAT_MSG,
	};
	enum PDOTransmissionType{
		SYNC_ACYCLIC = 0,
		SYCN_RTR = 0xFC,
		ASYNC_RTR = 0xFD,
		ASYNC = 0xFE
	};
	// Possible event actions when an error is produced
    protected:

    //! Direction of used TPDO messages
    static const int TPDO1, TPDO2;
    //! MSB of the direction of used TPDO messages
    static const int TPDO1_MSB, TPDO2_MSB;

    //! CAN device
    PCan *canDev;

    public:
    //! public constructor
    EpcMaCanOpen(PCan *can_device);
    //! public constructor
    ~EpcMaCanOpen();
    //! Checks if the can device is initialized
    //! @return OK
    //! @return ERROR
    virtual ReturnValue Setup();
    //! Sends a CAN message
    virtual ReturnValue Send(byte can_id, CANMessage type, AccessMode mode, int param);  // byte can_id

};

#endif

