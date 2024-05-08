/*! \class MasterDrive
 *  \file master_drive.h
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date 2014
 *  \brief 
 * 
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR -PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include <curtis_controller/peak_can.h>
#include <curtis_msgs/DriveData.h>

#define MOTOR_DIRECTION				-1	// +1 REGULAR
										// -1 INVERTED

#define RESET_CODE					0x4850								
										

//! Class to process (and send) the CAN msgs from Curtis motordrives
//! Allows configuration of master/slave
class MasterDrive{
	public:
		
		
	protected:
		//! Saves the motor direction configuration
		int motor_direction;
		//! Pointer to a shared PCan device
		PCan *can_dev;
		//! Flag to control that the component is initialized
		bool initialized;
		
		//! PDO id for CAN msgs
		static const unsigned int PDO1_MISO_MtoC;	// Master to Computer
		static const unsigned int PDO1_MISO_StoM;	// Slave to Master
		static const unsigned int PDO1_MOSI_MtoS;	// Master to Slave
		static const unsigned int PDO1_MOSI_CtoM;	// Computer to Master
		
		static const unsigned int PDO2_MISO_StoC;	// Slave to Computer
		static const unsigned int PDO2_MISO_MtoC;	// Master to Computer
		
		
		//! Current values of the Drive
		curtis_msgs::DriveData data_master;
		curtis_msgs::DriveData data_slave;
		
	public:
		//! constructor
		MasterDrive(PCan *dev);
		//! destructor
		~MasterDrive();
		
		//! Setup
		int setup();
		//! Shutdow
		int shutdown();
		//! Process the received CAN msg
		void processCANMessage(TPCANMsg msg);
		
		//! Gets the current data (processed from CANmsgs) of the drive
		curtis_msgs::DriveData getDataMaster();
		curtis_msgs::DriveData getDataSlave();
		char * getFaultString(int fault_code);
		
		//! Sends a CAN command to move the motor/reset the driver
		int sendThrottle(int value, bool reset);
		//! Sets the motor encoder direction
		void setMotorDirection(int value);
	protected:
		void processPDO1_MISO_MtoC(TPCANMsg msg);
		void processPDO2_MISO_MtoC(TPCANMsg msg);
		void processPDO1_MOSI_MtoS(TPCANMsg msg);
		void processPDO2_MISO_StoC(TPCANMsg msg);
		void processPDO1_MISO_StoM(TPCANMsg msg);
	
};
