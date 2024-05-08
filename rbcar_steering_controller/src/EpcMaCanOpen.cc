/*! \class EpcMaCanOpen
 *  \file EpcMaCanOpen.cc
 *	\author Robotnik Automation S.L.L
 *	\version 1.0
 *	\date 2015
 *  \brief Class to manage the communication with the EPC encoders MA family
 * (C) 2012 Robotnik Automation, SLL
 *  All rights reserved.
 */

#include <rbcar_steering_controller/EpcMaCanOpen.h>


const int EpcMaCanOpen::TPDO1        = 0x180;
const int EpcMaCanOpen::TPDO1_MSB    = 0x01;
const int EpcMaCanOpen::TPDO2        = 0x280;
const int EpcMaCanOpen::TPDO2_MSB    = 0x02;



/*! \fn EpcMaCanOpen::EpcMaCanOpen(PCan *can_device)
 * Constructor
*/
EpcMaCanOpen::EpcMaCanOpen(PCan *can_device):Component(10.0){
    // Change the name to the name of the derived class
    sComponentName.assign("EpcMaCanOpen");
    canDev = can_device;
}

/*! \fn EpcMaCanOpen::~EpcMaCanOpen()
 * Destructor
*/
EpcMaCanOpen::~EpcMaCanOpen(){

}

/*! \fn ReturnValue EpcMaCanOpen::Setup()
 * Configures and initializes the component
 * \return OK
 * \return INITIALIZED if the component is already intialized
 * \return ERROR
*/
ReturnValue EpcMaCanOpen::Setup(){

	// Checks if has been initialized
	if(bInitialized){
	   ROS_ERROR( "%s::Setup: Already initialized",sComponentName.c_str() );
	   return INITIALIZED;
	}

	///////////////////////////////////////////////////
	// Setups CAN device							 //
	///////////////////////////////////////////////////
	if(canDev == NULL){
          ROS_ERROR("%s::Setup: can device points to null",sComponentName.c_str());
	  return ERROR;
    	}
    
	if(canDev->Setup() == ERROR){
	  ROS_ERROR("%s::Setup: Error in can device setup",sComponentName.c_str());
	  return ERROR;
	  }

	bInitialized = true;
	return OK;
}

/*! \fn ReturnValue EpcMaCanOpen::Send(byte can_id, CANMessage type, AccessMode mode ,int param = 0)
 * Sends a specific CAN message to the desired device
 * \param can_id as byte, CAN ID of the device
 * \param type as CANMessage, type of message (CAN object) accepted by the device
 * \param mode as AccessMode, defines access mode of the message (READ / WRITE)
 * \param param as int, optional parameter to send with the message
 * \return OK
 * \return ERROR
 * \return NOT_INITIALIZED if the component has not been initialized
*/

ReturnValue EpcMaCanOpen::Send(byte can_id, CANMessage type, AccessMode mode ,int param = 0){
    TPCANMsg msg;
    byte b1 = 0, b2 = 0, b3 = 0, b4 = 0;

    if(!bInitialized){
		ROS_ERROR("%s::Send: Impossible to send because of it's not initialized", sComponentName.c_str());
		return NOT_INITIALIZED;
	}

    msg.ID = 0x600 + can_id;
    msg.LEN = 0x08;         //Message size(8 bytes)
    msg.MSGTYPE = MSGTYPE_STANDARD;
    
    // Command
    if(mode == READ)
        msg.DATA[0]=0x40;       //Command -> 40h
    else
        msg.DATA[0]=0x22;       //Command -> 22h
    // CAN Object
    switch(type){
        
        case START_COMM_MSG:
            msg.ID = 0x00;
            msg.LEN = 0x02;         //Tamaño del mensaje (2 bytes)
            msg.DATA[0]=0x01;       //mode
            msg.DATA[1]= can_id;    //node
            msg.DATA[2]=0;
            msg.DATA[3]=0;
            msg.DATA[4]=0;
            msg.DATA[5]=0;
            msg.DATA[6]=0;
            msg.DATA[7]=0;
        break;
		case STOP_COMM_MSG:
            msg.ID = 0x00;
            msg.LEN = 0x02;         //Tamaño del mensaje (2 bytes)
            msg.DATA[0]=0x02;       //mode
            msg.DATA[1]= can_id;    //node
            msg.DATA[2]=0;
            msg.DATA[3]=0;
            msg.DATA[4]=0;
            msg.DATA[5]=0;
            msg.DATA[6]=0;
            msg.DATA[7]=0;
        break;
        case RESET_COMM_MSG:
            msg.ID = 0x00;
            msg.LEN = 0x02;         //Tamaño del mensaje (2 bytes)
            msg.DATA[0]=0x82;       //mode
            msg.DATA[1]= can_id;    //node
            msg.DATA[2]=0;
            msg.DATA[3]=0;
            msg.DATA[4]=0;
            msg.DATA[5]=0;
            msg.DATA[6]=0;
            msg.DATA[7]=0;
        break;
		case RESET_NODE_MSG:
            msg.ID = 0x00;
            msg.LEN = 0x02;         //Tamaño del mensaje (2 bytes)
            msg.DATA[0]=0x81;       //mode
            msg.DATA[1]= can_id;    //node
            msg.DATA[2]=0;
            msg.DATA[3]=0;
            msg.DATA[4]=0;
            msg.DATA[5]=0;
            msg.DATA[6]=0;
            msg.DATA[7]=0;
        break;
		case PREOPERATIONAL_COMM_MSG:
            msg.ID = 0x00;
            msg.LEN = 0x02;         	// Message size (2 bytes)
            msg.DATA[0]=0x80;       	// mode
            msg.DATA[1]= can_id;    	// node
            msg.DATA[2]=0;
            msg.DATA[3]=0;
            msg.DATA[4]=0;
            msg.DATA[5]=0;
            msg.DATA[6]=0;
            msg.DATA[7]=0;
        break;
        case NODEGUARD_MSG:    
            msg.ID = 0x700 + can_id;
            msg.MSGTYPE = MSGTYPE_RTR;   // 
            msg.LEN = 0x08;              // 
        break;
		case HEARTBEAT_MSG:
            msg.ID = 0x700;              //+ can_id;
            msg.LEN = 0x01;              // 
			msg.DATA[0]=0x00;
            msg.DATA[1]=0x0;
            msg.DATA[2]=0x0;
            msg.DATA[3]=0x0;
            msg.DATA[4]=0x0;
            msg.DATA[5]=0x0;
            msg.DATA[6]=0x0;
            msg.DATA[7]=0x0;
        break;
        case SYNC_MSG:
            msg.ID = 0x80;
            msg.LEN = 0x00;
        break;
        
		case GUARD_TIME_MSG:
			if(mode == WRITE){
                b1 = param & 0xFF;
                b2 = (param>>8) & 0xFF;
            }
			msg.DATA[1]=0x0C;       // Object Index(LSB)
            msg.DATA[2]=0x10;       // Object Index(MSB)
            msg.DATA[3]=0x00;		// Sub-Index 
            msg.DATA[4]=b1;			// Byte 4 (LSB)
            msg.DATA[5]=b2;			// Byte 6
            msg.DATA[6]=0x00;       // Byte 7
            msg.DATA[7]=0x00;		// Byte 8 (LSB)
		break;
		
		case HEARTBEAT_CONSUMER_MSG:
            if(mode == WRITE){
                b1 = param & 0xFF;
                b2 = (param>>8) & 0xFF;
                b3 = (param>>16) & 0xFF;
                b4 = (param>>24) & 0xFF;
            }
            msg.DATA[1]=0x16;       // Object Index(LSB)
            msg.DATA[2]=0x10;       // Object Index(MSB)
            msg.DATA[3]=0x01;       // Sub-Index
            msg.DATA[4]=b1;       // Byte 5
            msg.DATA[5]=b2;       // Byte 6
            msg.DATA[6]=b3;       // Byte 7
            msg.DATA[7]=b4;       // Byte 8
        break;
	
	default:
            ROS_ERROR("%s::Send: Unknown message", sComponentName.c_str());
            return ERROR;
        break;

    }

    return canDev->Send(&msg);
}
