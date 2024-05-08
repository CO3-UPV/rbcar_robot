/*! \class MasterDrive
 *  \file master_drive.cc
 *	\author Robotnik Automation S.L.L
 *	\version 1.0
 *	\date 2014
 *  \brief
 * (C) 2014 Robotnik Automation, SLL
 *  All rights reserved.
 */
 
#include <curtis_controller/master_drive.h>
 
 
const unsigned int MasterDrive::PDO1_MISO_MtoC = 0x1A6; 
const unsigned int MasterDrive::PDO1_MOSI_MtoS = 0x227; 
const unsigned int MasterDrive::PDO2_MISO_MtoC = 0x2A6;
const unsigned int MasterDrive::PDO2_MISO_StoC = 0x2A7;
const unsigned int MasterDrive::PDO1_MISO_StoM = 0x1A7;
const unsigned int MasterDrive::PDO1_MOSI_CtoM = 0x226;

#define CURTIS_SPEED_DIVISOR	10.0
#define KMH_TO_MPS				0.277777778
 
/*! \fn MasterDrive::MasterDrive(PCan *dev, byte id)
 	* Constructor by default
    \param dev as PCan *, CAN object device
    \param id as byte, CAN bus id
*/
MasterDrive::MasterDrive(PCan *dev){
	can_dev = dev;
	
	initialized = false;
	
	data_master.type = "MASTER";
	data_slave.type = "SLAVE";
	motor_direction = MOTOR_DIRECTION;
	
}


/*! \fn MasterDrive::~MasterDrive()
 	* Destructor by default
*/
MasterDrive::~MasterDrive(){
	
}


/*! \fn int MasterDrive::setup()
 * Setups the component
 * \return OK
 * \return ERROR
 * \return INITIALIZED
*/
int MasterDrive::setup(){
	if(initialized){
		ROS_INFO("MasterDrive::Setup: Already initialized");
		
		return INITIALIZED;
	}
	
	int ret = OK;
	//
	///////////////////////////////////////////////////
	// Setups the component or another subcomponents if it's necessary //
	//////////////////////////////////////////////////
	if(can_dev->setup() != ERROR){
		initialized = true;
	}else
		ret = ERROR;
	
	return ret;
}


/*! \fn int MasterDrive::shutdown()
 * Closes and frees the reserved resources
 * \return OK
 * \return ERROR if fails when closes the devices
 * \return NOT_INITIALIZED if the component is not initialized
*/
int MasterDrive::shutdown(){

	if(!initialized){
		ROS_INFO("MasterDrive::shutdown: Impossible because of it's not initialized");
		return NOT_INITIALIZED;
	}
	
	int ret = OK;
	
	//
	///////////////////////////////////////////////////////
	// ShutDowns another subcomponents if it's necessary //
	///////////////////////////////////////////////////////

		
	if(can_dev->shutdown() != ERROR)
		initialized = false;
	else
		ret = ERROR;


	return ret;
}


/*! \fn void MasterDrive::processCANMessage(TPCANMsg msg)
 * Process the received CAN msg
*/
void MasterDrive::processCANMessage(TPCANMsg msg){
	bool master_received = false, slave_received = false;
	
	if(msg.ID == PDO1_MISO_MtoC){
		
		//ROS_INFO("MasterDrive::processCANMessage: PDO1_MISO_MtoC");
		processPDO1_MISO_MtoC(msg);
		master_received = true;
		
	}else if(msg.ID == PDO2_MISO_MtoC){
		processPDO2_MISO_MtoC(msg);
		master_received = true;
		//ROS_INFO("MasterDrive::processCANMessage: PDO2_MISO_MtoC");
		
	}else if(msg.ID == PDO1_MOSI_MtoS){
		processPDO1_MOSI_MtoS(msg);
		master_received = true;
		
		
	}else if(msg.ID == PDO2_MISO_StoC){
		processPDO2_MISO_StoC(msg);
		slave_received = true;
		
	
	}else if(msg.ID == PDO1_MISO_StoM){
		processPDO1_MISO_StoM(msg);
		slave_received = true;
		
	}
	
	if(master_received)
		data_master.header.stamp = ros::Time::now();
	if(slave_received)
		data_slave.header.stamp = ros::Time::now();
}


/*! \fn void MasterDrive::processTPDO1_MISO_MtoC(TPCANMsg msg)
 * Process the received PDO1 msg (MASTER TO COMPUTER)
 * Values received:
 * 1	Current (RMS)			LO
 * 2	Current (RMS)			HI
 * 3	Battery Current			LO
 * 4	Battery Current			HI
 * 5 	Keyswitch voltage		LO
 * 6 	Keyswitch voltage		HI
 * 7 	BDI percentage			LO
 * 8 	BDI percentage			HI
*/
void MasterDrive::processPDO1_MISO_MtoC(TPCANMsg msg){
	int16_t current = 0, battery_current = 0, keyswitch_voltage = 0, bdi_percentage = 0;
	

	//Extract the current rms value
	current = msg.DATA[1] << 8;
	current |= msg.DATA[0];
	
	//Extract the battery_current value
	battery_current = msg.DATA[3] << 8;
	battery_current |= msg.DATA[2];
	//Extract the keyswitch_voltage value
	keyswitch_voltage = msg.DATA[5] << 8;
	keyswitch_voltage |= msg.DATA[4];
	//Extract the bdi_percentage value
	bdi_percentage = msg.DATA[7] << 8;
	bdi_percentage |= msg.DATA[6];
	
	data_master.current_rms = (float)current/10.0;
	data_master.battery_current =  (float)battery_current/10.0;
	data_master.keyswitch_voltage = (float)keyswitch_voltage/100.0;
	data_master.bdi_percentage = (float)bdi_percentage;
	
	
	//ROS_INFO("MasterDrive::processTPDO1_MISO_MtoC: id = %x, current = %f, battery_current = %f, keyswitch_voltage = %f, bdi_percentage = %f",
	// msg.ID, data_master.current_rms, data_master.battery_current, data_master.keyswitch_voltage, data_master.bdi_percentage);
}


/*! \fn void MasterDrive::processTPDO2_MISO_MtoC(TPCANMsg msg)
 * Process the received PDO1 msg (MASTER TO COMPUTER)
 * Values received:
 * 1	Motor RPM				LO
 * 2	Motor RPM				HI
 * 3	Vehicle Speed			LO
 * 4	Vehicle Speed			HI
 * 5 	Master Controller Temp	LO
 * 6 	Master Controller Temp	HI
 * 7 	Master Motor Temp		LO
 * 8 	Master Motor Temp		HI
*/
void MasterDrive::processPDO2_MISO_MtoC(TPCANMsg msg){
	int16_t rpm = 0, speed = 0, control_temp = 0, motor_temp = 0;

	//Extract the RPM value
	rpm = msg.DATA[1] << 8;
	rpm |= msg.DATA[0];
	
	//Extract the speed value
	speed = msg.DATA[3] << 8;
	speed |= msg.DATA[2];
	//Extract the controller temp value
	control_temp = msg.DATA[5] << 8;
	control_temp |= msg.DATA[4];
	//Extract the controller temp value
	motor_temp = msg.DATA[7] << 8;
	motor_temp |= msg.DATA[6];
	
	
	data_master.motor_rpm = motor_direction * rpm;
	data_master.speed = motor_direction * (float)(speed/CURTIS_SPEED_DIVISOR)*KMH_TO_MPS;
	data_master.controller_temp = (float)control_temp/10.0;
	data_master.motor_temp = (float)motor_temp/10.0;
	
	//ROS_INFO("MasterDrive::processTPDO2_MISO_MtoC: id = %x, rpm = %d, speed = %f, control_temp = %f, motor_temp = %f", msg.ID, rpm, data_master.speed,
	// data_master.controller_temp,  data_master.motor_temp);
}


/*! \fn void MasterDrive::processPDO1_MOSI_MtoS(TPCANMsg msg)
 * Process the received msg (MASTER TO SLAVE)
 * Values received:
 * 1	CAN_Master_Commands				bit
 * 			CAN_Master_interlock_state	  0
 * 			CAN_Master_fault_status		  1	
 * 			CAN_Master_Mode_Auto		  2
 * 			CAN_Master_Mode_Manual        3
 * 2	CAN_Master_fault_code
 * 3	
 * 4	
 * 5 	
 * 6 	
 * 7 
 * 8 	
*/
void MasterDrive::processPDO1_MOSI_MtoS(TPCANMsg msg){
	//int16_t master_interlock = 0, master_fault = 0, fault_code = 0;
	
	data_master.interlock  = msg.DATA[0] & 0x01;
	
	data_master.on_fault = (msg.DATA[0] & 0x02) >> 1;
	
	data_master.fault_code = msg.DATA[1];
	
	data_master.mode_auto = (msg.DATA[0] & 0x04) >> 2;
	
	data_master.mode_manual = (msg.DATA[0] & 0x08) >> 3;
	
	//ROS_INFO("MasterDrive::processPDO1_MOSI_MtoS: id = %x, master_interlock = %d, master_fault = %d, fault_code = %d (%d)",
	// msg.ID, master_interlock, master_fault, fault_code, msg.DATA[0]);
}

/*! \fn void MasterDrive::processPDO2_MISO_StoC(TPCANMsg msg)
 * Process the received PDO1 msg (SLAVE TO COMPUTER)
 * Values received:
 * 1 	Slave Controller Temp	LO
 * 2 	Slave Controller Temp	HI
 * 3 	Slave Motor Temp		LO
 * 4 	Slave Motor Temp		HI
 * 5	Current (RMS)			LO
 * 6	Current (RMS)			HI
 * 7	Battery Current			LO
 * 8	Battery Current			HI
*/
void MasterDrive::processPDO2_MISO_StoC(TPCANMsg msg){
	int16_t current = 0, battery_current = 0, control_temp = 0, motor_temp = 0;

	//Extract the controller temp value
	control_temp = msg.DATA[1] << 8;
	control_temp |= msg.DATA[0];
	//Extract the controller temp value
	motor_temp = msg.DATA[3] << 8;
	motor_temp |= msg.DATA[2];
	
	//Extract the current value
	current = msg.DATA[5] << 8;
	current |= msg.DATA[4];
	
	//Extract the speed value
	battery_current = msg.DATA[7] << 8;
	battery_current |= msg.DATA[6];
	
	
	
	data_slave.current_rms = (float)current/10.0;
	data_slave.battery_current = (float)battery_current/10.0;
	data_slave.controller_temp = (float)control_temp/10.0;
	data_slave.motor_temp = (float)motor_temp/10.0;
	
	//ROS_INFO("MasterDrive::processPDO2_MISO_StoC: id = %x, current = %f, battery_current = %f, control_temp = %f, motor_temp = %f", msg.ID, data_slave.current_rms, data_slave.current_battery,
	// data_slave.controller_temp,  data_slave.motor_temp);
}


/*! \fn void MasterDrive::processPDO1_MISO_StoM(TPCANMsg msg)
 * Process the received msg (SLAVE TO MASTER)
 * Values received:
 * 1	CAN_Master_Commands				bit
 * 			CAN_Master_interlock_state	  0
 * 			CAN_Master_falut_status		  1
 * 2	CAN_Master_fault_code
 * 3	
 * 4	
 * 5 	
 * 6 	
 * 7 	
 * 8 	
*/
void MasterDrive::processPDO1_MISO_StoM(TPCANMsg msg){
	//int16_t master_interlock = 0, master_fault = 0, fault_code = 0;

	data_slave.fault_code = msg.DATA[1];
	
	data_slave.interlock = msg.DATA[0] & 0x01;
	
	data_slave.on_fault = (msg.DATA[0] & 0x02) >> 1;
	
	//ROS_INFO("MasterDrive::processPDO1_MISO_StoM: id = %x, master_interlock = %d, master_fault = %d, fault_code = %d (%d)",
	// msg.ID, master_interlock, master_fault, fault_code, msg.DATA[0]);
}

/*! \fn curtis_msgs::DriveData MasterDrive::getDataMaster()
 * Gets the current data of the master drive
*/
curtis_msgs::DriveData MasterDrive::getDataMaster(){
	return data_master;
}


/*! \fn curtis_msgs::DriveData MasterDrive::getDataSlave()
 * Gets the current data of the slave drive
*/
curtis_msgs::DriveData MasterDrive::getDataSlave(){
	return data_slave;
}

/*! \fn curtis_msgs::DriveData MasterDrive::getFaultDescription()
 * Return the describing string given a fault number
*/
char * MasterDrive::getFaultString(int fault_code){

	switch(fault_code){
		case 38:
			return (char *)"MAIN_CONTACTOR_WELDED";
		break;
		case 39:
			return (char *)"MAIN_CONTACTOR_DID_NOT_CLOSE";
		break;
		case 45:
			return (char *)"POT_LOW_OVERCURRENT";
		break;
		case 42:
			return (char *)"THROTTLE_WIPER_LOW ";
		break;
		case 41:
			return (char *)"THROTTLE_WIPER_HIGH ";
		break;
		case 44:
			return (char *)"POT2_WIPER_LOW ";
		break;
		case 43:
			return (char *)"POT2_WIPER_HIGH";
		break;
		case 46:
			return (char *)"EEPROM_FAILURE";
		break;
		case 47:
			return (char *)"HPD_SEQUENCING";
		break;
		case 17:
			return (char *)"SEVERE_UNDERVOLTAGE";
		break;
		case 18:
			return (char *)"SEVERE_OVERVOLTAGE";
		break;		
		case 23:
			return (char *)"UNDERVOLTAGE_CUTBACK";
		break;
		case 24:
			return (char *)"OVERVOLTAGE_CUTBACK";
		break;
		case 21:
			return (char *)"NOT_KNOWDN";
		break;
		case 22:
			return (char *)"CONTROLLER_OVERTEMP_CUTBACK";
		break;
		case 15:
			return (char *)"CONTROLLER_SEVERE_UNDERTEMP";
		break;
		case 16:
			return (char *)"CONTROLLER_SEVERE_OVERTEMP";
		break;
		case 33:
			return (char *)"COIL3_DRIVER_OPEN_SHORT";
		break;
		case 34:
			return (char *)"COIL4_DRIVER_OPEN_SHORT";
		break;
		case 35:
			return (char *)"PD_OPEN_SHORT";
		break;
		case 31:
			return (char *)"MAIN_OPEN_SHORT";
		break;
		case 32:
			return (char *)"EMBRAKE_OPEN_SHORT";
		break;
		case 14:
			return (char *)"PRECHARGE_FAILED";
		break;
		case 26:
			return (char *)"DIGITAL_OUT_6_OVERCURRENT";
		break;
		case 27:
			return (char *)"DIGITAL_OUT_7_OVERCURRENT";
		break;
		case 12:
			return (char *)"DIGITAL_OUT_6_OVERCURRENT";
		break;				
		case 13:
			return (char *)"DIGITAL_OUT_6_OVERCURRENT";
		break;
		case 28:
			return (char *)"MOTOR_TEMP_HOT_CUTBACK";
		break;		
		case 49:
			return (char *)"PARAMETER_CHANGE";
		break;
		case 37:
			return (char *)"MOTOR_OPEN";
		break;			
		
		/* USER FAULTS */		
		case 51:
			return (char *)"SLAVE_TIMEOUT_FAULT";
		break;
		case 52:
			return (char *)"CUSTOM_HPD_FAULT";
		break;
		case 53:
			return (char *)"SEQUENCING_FAULT";
		break;
		case 54:
			return (char *)"SLAVE_FAULTED";
		break;
		case 55:
			return (char *)"COMPUTER_TIMEOUT_FAULT";
		break;
		/* /USER FAULTS */
		
		case 69:
			return (char *)"EXTERNAL_SUPPLY_OUT_OF_RANGE";
		break;
		case 29:
			return (char *)"MOTOR_TEMP_SENSOR ";
		break;				
		case 68:
			return (char *)"VCL_RUN_TIME_ERROR";
		break;
		case 25:
			return (char *)"EXTERNAL_5V";
		break;		
		case 71:
			return (char *)"OS_GENERAL";
		break;
		case 72:
			return (char *)"PDO_TIMEOUT";
		break;			
		case 36:
			return (char *)"ENCODER";
		break;
		case 73:
			return (char *)"STALL_DETECTED ";
		break;				
		case 89:
			return (char *)"MOTOR_TYPE_ERROR";
		break;		
		case 87:
			return (char *)"MOTOR_CHARACTERIZATION";
		break;
		case 97:
			return (char *)"PUMP_HARDWARE";
		break;		
		case 91:
			return (char *)"VCL_OS_MISMATCH";
		break;
		case 92:
			return (char *)"EM_BRAKE_FAILED_TO_SET";
		break;				
		case 93:
			return (char *)"ENCODER_LOS";
		break;
		case 94:
			return (char *)"EMER_REV_TIMEOUT";
		break;		
		case 75:
			return (char *)"DUAL_SEVERE";
		break;
		case 74:
			return (char *)"FAULT_ON_OTHER_TRACTION_CONTROLLER ";
		break;			
		case 98:
			return (char *)"ILLEGAL_MODEL_NUMBER";
		break;				
		case 95:
			return (char *)"PUMP_OVERCURRENT";
		break;
		case 96:
			return (char *)"PUMP_BDI";
		break;
		case 99:
			return (char *)"DUALMOTOR_PARAMETER_MISMATCH";
		break;			
		default:
			return (char *)"";
		break;
	}
}


/*! \fn int MasterDrive::sendThrottle(int value, bool reset)
 * Sends a CAN command to move the motor/reset the driver
 * \param value as int, -100 to +100 (%)
*/
int MasterDrive::sendThrottle(int value, bool reset){
	TPCANMsg msg;
    
    if(!initialized){
		ROS_ERROR("MasterDrive::sendThrottle: Impossible to send because of it's not initialized");
		return NOT_INITIALIZED;
	}

    msg.ID = PDO1_MOSI_CtoM;
    msg.LEN = 0x04;         //Message size(8 bytes)
    msg.MSGTYPE = MSGTYPE_STANDARD;
    
    // Throttle
    msg.DATA[0] = value & 0xFF;
    msg.DATA[1] = (value>>8) & 0xFF;
    
    // Reset
    if (reset != 0){
		msg.DATA[2] = RESET_CODE & 0xFF;
		msg.DATA[3] = (RESET_CODE>>8) & 0xFF;
	}else{
		msg.DATA[2] = 0;
		msg.DATA[3] = 0;
	}
    
    //ROS_INFO("MESSAGE SENT: [%x] [%x] [%x] [%x]", msg.DATA[0], msg.DATA[1], msg.DATA[2], msg.DATA[3]);
    
    return can_dev->send(&msg);
}


/*! \fn void MasterDrive::setMotorDirection(int value)
 * Sets the motor encoder direction
*/
void MasterDrive::setMotorDirection(int value){
	motor_direction = value;
}
