/*! \class CurtisController
 *  \file curtis_controller.h
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
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef __CURTISCONTROLLER_H
#define __CURTISCONTROLLER_H

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <std_srvs/Empty.h>
#include <robotnik_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <curtis_controller/peak_can.h>
#include <curtis_controller/master_drive.h>
#include <curtis_msgs/Throttle.h>
#include <curtis_msgs/DriveData.h>

//! Size of string for logging
#define DEFAULT_THREAD_DESIRED_HZ	40.0
#define DEFAULT_CAN_PORT			"/dev/pcan32"
#define DEFAULT_WATCHDOG_COMMAND	true	
#define DEFAULT_MODE				0
#define WATCHDOG_TIMEOUT			0.2 // secs
#define RECOVER_TIMER				5.0	// secs

//! Motor parametrization
#define WHEEL_DIAMETER_M			0.5
#define MAX_RPM						4300.0
#define GEAR_RATIO					16.0
#define MAX_VEL_AUTO				2.5

//! Motor params
struct curtis_motor_params{
	double wheel_diameter;
	double max_rpm;
	double max_rps;
	double gear_ratio;
	double max_v_mps;
	double max_v_auto;
	int motor_direction;
};

using namespace std;

//! Class Rcomponent
class CurtisController{
	protected:
		//! Controls if has been initialized succesfully
		bool initialized, ros_initialized;
		//! Controls the execution of the CurtisController's thread
		bool running;
		
		//! State of the CurtisController
		int state;
		//! State before
		int previous_state;
		//!	Saves the name of the component
		string component_name;
		//! ROS node handle
		ros::NodeHandle nh_;
		//! Private ROS node handle
		ros::NodeHandle pnh_;
		//! Desired loop frequency
		double desired_freq_, real_freq;
		//! Name of can port
		string can_port_;
		
		// PUBLISHERS
		//! Publish the component state
		ros::Publisher state_publisher;
		//! Publish data from master drive
		ros::Publisher master_drive_data_publisher;
		ros::Publisher slave_drive_data_publisher;
		
		// SUBSCRIBERS
		//! Receives throttle commands
		ros::Subscriber throttle_command_subscriber;
		//! Receives Twist commands
		ros::Subscriber twist_command_subscriber;
		
		// SERVICE
		ros::ServiceServer reset_service_caller; 
		
		// TIMERS
		ros::Time last_command_time;
		ros::Time failure_recover_time;
		
		//! Object to send can msgs
		PCan *can_dev;
		//! Object to manage comm with master drive
		MasterDrive *master_drive;
		//! Sets the desired throttle
		double desired_throttle;
		
		//! Flag containing the interlock state of the master drive
		bool interlock_, last_interlock_;
		
		//! Flag active if we want watchdog command control
		bool watchdog_command_;
		
		//! Work mode
		int mode_; 
		
		//! Motor direction
		int motor_direction_;
		
		//! Reset flag variable
		bool reset_;
		
		//! Motor parametrization
		curtis_motor_params motor_params_;
		
	public:
		//! Public constructor
		CurtisController(ros::NodeHandle h);
		//! Public destructor
		virtual ~CurtisController();
		
		//! Starts the control loop of the component and its subcomponents
		//! @return OK
		//! @return ERROR starting the thread
		//! @return RUNNING if it's already running
		//! @return NOT_INITIALIZED if it's not initialized
		virtual int start();
		//! Stops the main control loop of the component and its subcomponents
		//! @return OK
		//! @return ERROR if any error has been produced
		//! @return NOT_RUNNING if the main thread isn't running
		virtual int stop();
		//! Returns the general state of the CurtisController
		int getState();
		//! Returns the general state of the CurtisController as string
		char *getStateString();
		//! Returns the general state as string
		char *getStateString(int state);
		//! Method to get current update rate of the thread
		//! @return pthread_hz
		double getUpdateRate();
		
	protected:
		//! Configures and initializes the component
		//! @return OK
		//! @return INITIALIZED if the component is already intialized
		//! @return ERROR
		virtual int setup();
		//! Closes and frees the reserved resources
		//! @return OK
		//! @return ERROR if fails when closes the devices
		//! @return RUNNING if the component is running
		//! @return NOT_INITIALIZED if the component is not initialized
		virtual int shutdown();
		//! All core component functionality is contained in this thread.
		//!	All of the CurtisController component state machine code can be found here.
		virtual void controlLoop();
		//! Actions performed on initial state
		virtual void initState();
		//! Actions performed on standby state
		virtual void standbyState();
		//! Actions performed on ready state
		virtual void readyState();
		//! Actions performed on the emergency state
		virtual void emergencyState();
		//! Actions performed on Failure state
		virtual void failureState();
		//! Actions performed on Shudown state
		virtual void shutdownState();
		//! Actions performed in all states
		virtual void allState();
		//! Switches between states
		virtual void switchToState(int new_state);
		//! Setups all the ROS' stuff
		virtual int rosSetup();
		//! Shutdowns all the ROS' stuff
		virtual int rosShutdown();
		//! Reads data a publish several info into different topics
		virtual void rosPublish();
		//! Reads params from params server
		virtual void rosReadParams();
		
		//! Reads CAN msgs from bus and process them
		int readCANMessages();
		//! Callback executed to process throttle msgs
		void cmdThrottleCb(const curtis_msgs::Throttle::ConstPtr& cmd);
		//! Callback executed to process Twist msgs
		void cmdTwistCb(const geometry_msgs::Twist::ConstPtr& cmd);
		//! Callback executed to process the reset service
		bool resetSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
};

#endif
