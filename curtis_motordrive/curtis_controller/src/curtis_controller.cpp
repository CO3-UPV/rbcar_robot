/*! \class CurtisController
 *  \file CurtisController.cpp
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date 2014
 *  \brief Class to define a standard and shared structure (attributes & methods) for all the components
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

#include <curtis_controller/curtis_controller.h>


/*! \fn CurtisController::CurtisController()
 *  \brief Constructor by default
 *	\param hz as double, sets the desired frequency of the controlthread
 *	\param h as ros::NodeHandle, ROS node handle
*/
CurtisController::CurtisController(ros::NodeHandle h):nh_(h), pnh_("~"){
	// Set main flags to false
	ros_initialized = initialized = running = false;
	// reads params from server
	rosReadParams();
	
	if(desired_freq_ <= 0.0)
		desired_freq_ = DEFAULT_THREAD_DESIRED_HZ;
		
	state = robotnik_msgs::State::INIT_STATE;
	// Realizar para cada una de las clases derivadas
	component_name.assign("CurtisController");
	
	can_dev = new PCan(can_port_, CAN_BAUD_125K);
	master_drive = new MasterDrive(can_dev);
	master_drive->setMotorDirection(motor_params_.motor_direction);
	
	desired_throttle = 0.0;
}

/*! \fn CurtisController::~CurtisController()
 * Destructor by default
*/
CurtisController::~CurtisController(){
	
	delete master_drive;
	delete can_dev;
}

/*! \fn int CurtisController::setup()
 * Configures and initializes the component
 * \return OK
 * \return INITIALIZED if the component is already intialized
 * \return ERROR
*/
int CurtisController::setup(){
	// Checks if has been initialized
	if(initialized){
		ROS_INFO("%s::Setup: Already initialized",component_name.c_str());
		
		return INITIALIZED;
	}
	
	//
	///////////////////////////////////////////////////
	// Setups the component or another subcomponents if it's necessary //
	///////////////////////////////////////////////////
	if(can_dev->setup() != ERROR){
		if(master_drive->setup() != ERROR) 
			initialized = true;
	}else 
		return ERROR;
		
	

	return OK;
}

/*! \fn int CurtisController::shutDown()
 * Closes and frees the reserved resources
 * \return OK
 * \return ERROR if fails when closes the devices
 * \return RUNNING if the component is running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int CurtisController::shutdown(){
	
	if(running){
		ROS_INFO("%s::Shutdown: Impossible while thread running, first must be stopped",component_name.c_str());
		return THREAD_RUNNING;
	}
	if(!initialized){
		ROS_INFO("%s::Shutdown: Impossible because of it's not initialized", component_name.c_str());
		return NOT_INITIALIZED;
	}
	
	//
	///////////////////////////////////////////////////////
	// ShutDowns another subcomponents if it's necessary //
	///////////////////////////////////////////////////////
	can_dev->shutdown();
	master_drive->shutdown();
	
	initialized = false;

	return OK;
}


/*! \fn int CurtisController::start()
 * Starts the control thread of the component and its subcomponents
 * \return OK
 * \return RUNNING if it's already running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int CurtisController::start(){
	// Performs ROS setup
	rosSetup();
	
	if(running){
		ROS_INFO("%s::start: the component's thread is already running", component_name.c_str());
		return THREAD_RUNNING;
	}
	
	ROS_INFO("%s Started", component_name.c_str());
	ROS_INFO("%s CAN port: %s", component_name.c_str(), can_port_.c_str());
	ROS_INFO("%s Desired Frequency: %2.f", component_name.c_str(), desired_freq_);
	ROS_INFO("%s Watchdog: %s", component_name.c_str(), watchdog_command_?"True":"False");
	ROS_INFO("%s Mode: %s", component_name.c_str(), mode_?"MODE_TWIST":"MODE_THROTTLE");
	ROS_INFO("%s Direction: %d", component_name.c_str(), motor_params_.motor_direction);
	ROS_INFO("%s Motor params: wheel diameter %.3lf, max_rpm = %3.lf, gear ratio = %lf, max_v_mps = %.3lf", component_name.c_str(), motor_params_.wheel_diameter, motor_params_.max_rpm,
	 motor_params_.gear_ratio, motor_params_.max_v_mps);
	
	running = true;
	
	// Executes the control loop
	controlLoop();
	
	return OK;

}

/*! \fn int CurtisController::stop()
 * Stops the control thread of the Motors
 * \return OK
 * \return ERROR if it can't be stopped
 * \return THREAD_NOT_RUNNING if the thread is not running
*/
int CurtisController::stop(){
	
	if(!running){
		ROS_INFO("%s::stop: Thread not running", component_name.c_str());
	
		return THREAD_NOT_RUNNING;
	}
	//
	///////////////////////////////////////////////////
	// Stops another subcomponents, if it's necessary //
	///////////////////////////////////////////////////
	//
	ROS_INFO("%s::Stop: Stopping the component", component_name.c_str());
	
	running = false;

	usleep(100000);

	return OK;
}

/*!	\fn void CurtisController::controlLoop()
 *	\brief All core component functionality is contained in this thread.
*/
void CurtisController::controlLoop(){
	ROS_INFO("%s::controlLoop(): Init", component_name.c_str());
	ros::Rate r(desired_freq_);  
	ros::Time t1,t2;
	while(running && ros::ok()) {
		
		t1 = ros::Time::now();
		
		switch(state){
			
			case robotnik_msgs::State::INIT_STATE:
				initState();
			break;
			
			case robotnik_msgs::State::STANDBY_STATE:
				standbyState();
			break;
			
			case robotnik_msgs::State::READY_STATE:
				readyState();
			break;
			
			case robotnik_msgs::State::SHUTDOWN_STATE:
				shutdownState();
			break;
			
			case robotnik_msgs::State::EMERGENCY_STATE:
				emergencyState();
			break;
			
			case robotnik_msgs::State::FAILURE_STATE:
				failureState();
			break;
		
		}
		
		allState();
		
		ros::spinOnce();
		r.sleep();
		
		t2 = ros::Time::now();
		
		real_freq = 1.0/(t2 - t1).toSec();
		
	}
	
	shutdownState();
	// Performs ROS Shutdown
	rosShutdown();

	ROS_INFO("%s::controlLoop(): End", component_name.c_str());

}

/*!	\fn void CurtisController::initState()
 *	\brief Actions performed on initial 
 * 	Setups the component
*/
void CurtisController::initState(){
	// If component setup is successful goes to STANDBY (or READY) state
	if(setup() != ERROR){
		switchToState(robotnik_msgs::State::READY_STATE);
	}
}

/*!	\fn void CurtisController::shutdownState()
 *	\brief Actions performed on Shutdown state
*/
void CurtisController::shutdownState(){
	
	if(shutdown() == OK){
		switchToState(robotnik_msgs::State::INIT_STATE);
	}
}

/*!	\fn void CurtisController::standbyState()
 *	\brief Actions performed on Standby state
*/
void CurtisController::standbyState(){
	readCANMessages();
}

/*!	\fn void CurtisController::readyState()
 *	\brief Actions performed on ready state
*/
void CurtisController::readyState(){
	if(readCANMessages() == ERROR){
		switchToState(robotnik_msgs::State::FAILURE_STATE);
		return;
	}
	
	// Check in each interation the interlock bit. In case of losing it, it is sent a 0 reference to the driver
	// in order to avoid enter in error.
	if(!interlock_){
		//ROS_WARN("CurtisController::readyState: Interlock is not set. Desired throttle = 0");
		desired_throttle = 0;
	}else if(interlock_ && !last_interlock_){
		ROS_WARN("CurtisController::readyState: Interlock recovered. Desired throttle = 0");
		desired_throttle = 0;
	}
	
	if (reset_){
		if(master_drive->sendThrottle(0, true) != OK){
			ROS_ERROR("%s::readyState: Error resetting driver.", component_name.c_str());
			switchToState(robotnik_msgs::State::FAILURE_STATE);
		}
		reset_ = false;
	}else{
		
		/* Limit throttle */
		int iDesiredThrottle = 0;
		if(desired_throttle >= SHRT_MAX){
			iDesiredThrottle = SHRT_MAX;
		}else if (desired_throttle <= -SHRT_MAX){
			iDesiredThrottle = -SHRT_MAX;
		}else{
			iDesiredThrottle = (int)desired_throttle;
		}	
			
		if(master_drive->sendThrottle(iDesiredThrottle, false) != OK){
			ROS_ERROR("%s::readyState: Error setting value", component_name.c_str());
			switchToState(robotnik_msgs::State::FAILURE_STATE);
		}
	}

}

/*!	\fn void CurtisController::EmergencyState()
 *	\brief Actions performed on emergency state
*/
void CurtisController::emergencyState(){

}

/*!	\fn void CurtisController::FailureState()
 *	\brief Actions performed on failure state
*/
void CurtisController::failureState(){
	if( ros::Time::now() > failure_recover_time){
		
		shutdown();
		
		if(setup() != ERROR){
			ROS_INFO("%s::failureState: Recovered", component_name.c_str() );
			switchToState(robotnik_msgs::State::READY_STATE);
		}
	}
}

/*!	\fn void CurtisController::AllState()
 *	\brief Actions performed on all states
*/
void CurtisController::allState(){
	if(watchdog_command_){
		
		int d = (ros::Time::now() - last_command_time).toSec();
				
		if( (ros::Time::now() - last_command_time).toSec() > WATCHDOG_TIMEOUT){
			desired_throttle = 0;
			//ROS_WARN("%s::allStates: Watchdog activated", component_name.c_str());	
		}
	}
	rosPublish();
}

/*!	\fn double CurtisController::getUpdateRate()
 * 	\brief Gets current update rate of the thread
 * 	\return real frequency of the thread
*/
double CurtisController::getUpdateRate(){
	return desired_freq_;
}

/*!	\fn int CurtisController::getState()
 * 	\brief returns the state of the component
*/
int CurtisController::getState(){
	return state;
}

/*!	\fn char *CurtisController::getStateString()
 *	\brief Gets the state of the component as string
*/
char *CurtisController::getStateString(){
	return getStateString(state);
}

/*!	\fn char *CurtisController::getStateString(int state)
 *	\brief Gets the state as a string
*/
char *CurtisController::getStateString(int state){
	switch(state){
		case robotnik_msgs::State::INIT_STATE:
			return (char *)"INIT";
		break;
		case robotnik_msgs::State::STANDBY_STATE:
			return (char *)"STANDBY";
		break;
		case robotnik_msgs::State::READY_STATE:
			return (char *)"READY";
		break;
		case robotnik_msgs::State::EMERGENCY_STATE:
			return (char *)"EMERGENCY";
		break;
		case robotnik_msgs::State::FAILURE_STATE:
			return (char *)"FAILURE";
		break;
		case robotnik_msgs::State::SHUTDOWN_STATE:
			return (char *)"SHUTDOWN";
		break;
		default:
			return (char *)"UNKNOWN";
		break;
	}
}


/*!	\fn void CurtisController::switchToState(int new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void CurtisController::switchToState(int new_state){
	
	if(new_state == state)
		return;

	switch(new_state){
		case robotnik_msgs::State::FAILURE_STATE:
			failure_recover_time = ros::Time::now() + ros::Duration(RECOVER_TIMER);
		break;
	}
	// saves the previous state
	previous_state = state;
	ROS_INFO("%s::SwitchToState: %s -> %s", component_name.c_str(), getStateString(state), getStateString(new_state));	
	state = new_state;
	


}

/*!	\fn void CurtisController::rosSetup()
 * 	\brief Setups all ROS' stuff
*/
int CurtisController::rosSetup(){
	
	// Checks if has been initialized
	if(ros_initialized){
		ROS_INFO("%s::rosSetup: Already initialized",component_name.c_str());
		return INITIALIZED;
	}
	
	// Publishers
	state_publisher = pnh_.advertise<robotnik_msgs::State>("state", 1);
	master_drive_data_publisher = pnh_.advertise<curtis_msgs::DriveData>("master_drive", 1);
	slave_drive_data_publisher = pnh_.advertise<curtis_msgs::DriveData>("slave_drive", 1);
	
	// Subscribers
	if(mode_ == curtis_msgs::DriveData::MODE_THROTTLE){
		throttle_command_subscriber = pnh_.subscribe("command_throttle", 1, &CurtisController::cmdThrottleCb, this);
	}else if(mode_ == curtis_msgs::DriveData::MODE_TWIST){
		twist_command_subscriber = pnh_.subscribe("command_twist", 1, &CurtisController::cmdTwistCb, this);
	}
	//state_publisher = pnh_.advertise<std_msgs::Empty>("state", 1);
	
	// Services
	reset_service_caller = pnh_.advertiseService("reset", &CurtisController::resetSrv, this);

	ros_initialized = true;

	return OK;
	
	/*
	
	status_pub_ = pnh_.advertise<agvs_controller::DspicStatus>("status", 1);
	odom_pub_ = pnh_.advertise<nav_msgs::Odometry>(odom_frame_id_, 1);
	calibrate_srv_ = pnh_.advertiseService("calibrate",  &dspic_controller_node::CalibrateSrv, this);
	set_odom_service_ = pnh_.advertiseService("set_odometry", &dspic_controller_node::SetOdometry, this);*/
}


/*!	\fn void CurtisController::rosReadParams
 * 	\brief Reads the params set in ros param server
*/
void CurtisController::rosReadParams(){
	
	pnh_.param("desired_freq", desired_freq_, DEFAULT_THREAD_DESIRED_HZ);
	pnh_.param<string>("can_port", can_port_, DEFAULT_CAN_PORT);
	pnh_.param("watchdog_command", watchdog_command_, DEFAULT_WATCHDOG_COMMAND);
	pnh_.param("mode", mode_, DEFAULT_MODE);
	
	pnh_.param("wheel_diameter", motor_params_.wheel_diameter, WHEEL_DIAMETER_M);
	pnh_.param("max_rpm", motor_params_.max_rpm, MAX_RPM);
	pnh_.param("max_v_auto", motor_params_.max_v_auto, MAX_VEL_AUTO);
	pnh_.param("gear_ratio", motor_params_.gear_ratio, GEAR_RATIO);
	pnh_.param("motor_direction", motor_params_.motor_direction, MOTOR_DIRECTION);
	
	motor_params_.max_rps = motor_params_.max_rpm/60.0;
	motor_params_.max_v_mps = (M_PI * motor_params_.wheel_diameter) * (motor_params_.max_rps /motor_params_.gear_ratio);
	
	
	/* Example 
	pnh_.param<std::string>("port", port_, DEFAULT_DSPIC_PORT);
	pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "/odom_diff");
	pnh_.param<std::string>("base_frame_id", base_frame_id_, "/base_link");
	pnh_.param("publish_tf", publish_tf_, false);
	pnh_.param("desired_freq", desired_freq_, desired_freq_);*/
}

/*!	\fn int CurtisController::rosShutdown()
 * 	\brief Closes all ros stuff
*/
int CurtisController::rosShutdown(){
	if(running){
		ROS_INFO("%s::rosShutdown: Impossible while thread running, first must be stopped",component_name.c_str());
		return THREAD_RUNNING;
	}
	if(!ros_initialized){
		ROS_INFO("%s::rosShutdown: Impossible because of it's not initialized", component_name.c_str());
		return NOT_INITIALIZED;
	}
	
	ros_initialized = false;

	return OK;
}

/*!	\fn void CurtisController::rosPublish()
 * 	\brief Reads data a publish several info into different topics
*/
void CurtisController::rosPublish(){
	robotnik_msgs::State msg;
	
	// STATE
	msg.state = this->state;
	msg.desired_freq = this->desired_freq_;
	msg.real_freq = this->real_freq;
	msg.state_description = getStateString();	
	
	state_publisher.publish(msg);
	
	// MASTER DRIVE STATE
	curtis_msgs::DriveData msg_master;
	msg_master = master_drive->getDataMaster();
	msg_master.fault_description = master_drive->getFaultString(msg_master.fault_code);
	msg_master.desired_throttle = desired_throttle;
	last_interlock_ = interlock_;
	interlock_ = msg_master.interlock;
	master_drive_data_publisher.publish(msg_master);
	
	//DEBUG
	//ROS_INFO("Des. Speed: %f Real Speed: %f", desired_throttle, msg_master.speed);
	
	// SLAVE DRIVE STATE
	curtis_msgs::DriveData msg_slave;
	msg_slave = master_drive->getDataSlave();
	msg_slave.fault_description = master_drive->getFaultString(msg_slave.fault_code);
	slave_drive_data_publisher.publish(msg_slave);
	
}

/*! \fn int CurtisController::readCANMessages()
 *	\brief Read messages from CAN bus
 *	\return ERROR
 *	\return OK
*/
int CurtisController::readCANMessages(){
	TPCANRdMsg tpcmsg_read;
	int len=50;
	int i=0;
	byte aux_id = 0;


	// Read and process up to 100 messages
	for (i=0;i<len;i++) {
		int iRet = can_dev->read(&tpcmsg_read);
		
		if(iRet == ERROR){
		  ROS_ERROR("%s::ReadCANMessages: Error Reading messages from bus.", component_name.c_str());
		  return ERROR;
		}else {
			if (iRet == OK) {  // Something has been read			              
				/*aux_id = tpcmsg_read.Msg.ID & 0xF; 
				ROS_INFO( "CurtisController:ReadCANMessages: Read msg(%d): id=%x, len=%d, data[ %x %x %x %x %x %x %x %x ]", 
				i, tpcmsg_read.Msg.ID, tpcmsg_read.Msg.LEN, tpcmsg_read.Msg.DATA[0], tpcmsg_read.Msg.DATA[1], 
				tpcmsg_read.Msg.DATA[2], tpcmsg_read.Msg.DATA[3], tpcmsg_read.Msg.DATA[4], 
				tpcmsg_read.Msg.DATA[5], tpcmsg_read.Msg.DATA[6], tpcmsg_read.Msg.DATA[7]);		*/  
				//ROS_INFO("CurtisController::ReadCANMessages: id %x", tpcmsg_read.Msg.ID);
				master_drive->processCANMessage(tpcmsg_read.Msg); 
				//slave_drive->processCANMessage(tpcmsg_read.Msg); 

			}
		}
	}
	 return OK;
}


/*! \fn void CurtisController::cmdThrottleCb(const curtis_msgs::Throttle::ConstPtr& cmd)
	* Callback executed to process throttle msgs
*/
void CurtisController::cmdThrottleCb(const curtis_msgs::Throttle::ConstPtr& cmd){

    // Safety check
	last_command_time = ros::Time::now();
	
	if(fabs(cmd->value) <=100.0){
		if(state == robotnik_msgs::State::READY_STATE){
			//ROS_INFO("CurtisController::cmdThrottleCb");
			// SHRT_MAX: Throttle is stored in a 2 bytes variable in the driver.
			desired_throttle = motor_params_.motor_direction * (cmd->value * SHRT_MAX)/100;
		}
	}else{
		ROS_WARN("%s::cmdThrottleCb: Unable to set velocity, valid values between [-100.0 - 100.0]", component_name.c_str());
	}
}

/*! \fn void CurtisController::cmdTwistCb(const geometry_msgs::Twist::ConstPtr& cmd)
	* Callback executed to process twist msgs
*/
void CurtisController::cmdTwistCb(const geometry_msgs::Twist::ConstPtr& cmd){

    // Safety check
	last_command_time = ros::Time::now();
	
	if(fabs(cmd->linear.x) <= motor_params_.max_v_auto){ // Default: 2.5 m/s
		if(state == robotnik_msgs::State::READY_STATE){
			//ROS_INFO("CurtisController::cmdThrottleCb");
			// SHRT_MAX: Throttle is stored in a 2 bytes variable in the driver.
			// cmd->linear.x * 100 / MAX_VEL_MPS convert to % ref 
			desired_throttle = (((cmd->linear.x * 100 / motor_params_.max_v_mps) * SHRT_MAX)/100);
			//ROS_ERROR("desired_throttle: %f", desired_throttle);
		}
	}else{
		ROS_WARN("%s::cmdTwistCb: Unable to set velocity, valid values between [-2.5 - 2.5]", component_name.c_str());
	}
}

/*! \fn void CurtisController::resetSrv(const std_srv::Empty& empty)
	* Callback executed to process the reset service
*/
bool CurtisController::resetSrv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){

	if (!reset_){
		ROS_INFO("%s::resetSrv: Resetting driver.", component_name.c_str());
		reset_ = true;
		return true;
	}else{
		ROS_INFO("%s::resetSrv: Unable to reset driver.", component_name.c_str());
		reset_ = false;
		return false;
	}
	
}


// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "curtis_controller");
	
	ros::NodeHandle n;		
  	CurtisController controller(n);
	
	controller.start();

	return (0);
}
