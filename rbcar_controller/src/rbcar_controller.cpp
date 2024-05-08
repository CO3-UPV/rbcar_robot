/*! \class RbCarController
 *  \file rbcar_controller.cpp
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date 2015
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


#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <robotnik_msgs/State.h>
#include <robotnik_msgs/set_odometry.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>
#include <curtis_msgs/DriveData.h>
#include <rbcar_steering_controller/SetMotorStatus.h>
#include <rbcar_steering_controller/SteeringControllerStatus.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"
#include <tf/transform_broadcaster.h>

//! Size of string for logging
#define DEFAULT_THREAD_DESIRED_HZ	40.0

#define RBCAR_DEFAULT_CHILD_FRAME			"base_footprint"
#define RBCAR_DEFAULT_ODOM_FRAME			"odom"
#define RBCAR_DEFAULT_ODOM_TF				false					// Publish odom tf?	
#define RBCAR_DEFAULT_LEFT_REAR_JOINT		"left_rear_axle"		
#define RBCAR_DEFAULT_RIGHT_REAR_JOINT		"right_rear_axle"
#define RBCAR_DEFAULT_LEFT_FRONT_JOINT		"left_front_axle"
#define RBCAR_DEFAULT_RIGHT_FRONT_JOINT		"right_front_axle"
#define RBCAR_DEFAULT_LEFT_STEERING_JOINT	"left_steering_joint"
#define RBCAR_DEFAULT_RIGHT_STEERING_JOINT	"right_steering_joint"

#define RBCAR_DEFAULT_RIGHT_FRONT_SHOCK_JOINT	"right_front_shock"
#define RBCAR_DEFAULT_LEFT_FRONT_SHOCK_JOINT	"left_front_shock"
#define RBCAR_DEFAULT_RIGHT_REAR_SHOCK_JOINT	"right_rear_shock"
#define RBCAR_DEFAULT_LEFT_REAR_SHOCK_JOINT		"left_rear_shock"


#define RBCAR_MANUAL_MODE					1		// Manual control 
#define RBCAR_AUTO_MODE						2		// Automatic control
#define RBCAR_UNKNOWN_MODE					3		// Unknown

#define RBCAR_TIMEOUT_MSGS					1.0 	// Secs without receiving data from subcomponents
#define RBCAR_COMMAND_WATCHDOG				0.5 	// Secs without receiving commands

#define RBCAR_DEFAULT_DIST_AXIS_WHEELS		1.650	// Distance between front and rear wheels (m)

#define STEERING_MOTOR_DISABLED				1
#define STEERING_MOTOR_ENABLED				2

#define MAX_STEERING_ANGLE					0.60	//rads	(aprox ~26 degrees)
#define MAX_SPEED							2.0 //m/s

template <typename T> int sgn(T val){
	return (T(0) < val) - (val < T(0));
}
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

//! Struct to save the current joint states locally
struct SingleJointState{
	string joint_name;
	double position;
	double velocity;
};

//! Struct to save the parameters used for the odometry calculation
struct OdometryParams{
	//! Distance between frontal and rear axis
	double d_axis_wheels;	
	//! 
};

//! Class Rcomponent
class RbCarController{
	protected:
		//! Controls if has been initialized succesfully
		bool initialized, ros_initialized;
		//! Controls the execution of the RbCarController's thread
		bool running;
		//! State of the RbCarController
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
		
		//! Publish the component state
		ros::Publisher state_pub_;
		//! Publish the odometry
		ros::Publisher odom_pub_;
		//! Publish the Joint states
		ros::Publisher joint_state_pub_;
		//! Publish the command velocity
		ros::Publisher cmd_vel_pub_;
		//! Publish the command steering position
		ros::Publisher cmd_pos_pub_;

		//! Commands subscriber
		ros::Subscriber ackermann_command_sub_; 
		ros::Subscriber differential_command_sub_; 
		//! Curtis controller data
		ros::Subscriber curtis_controller_sub_; 
		//! Steering controller position
		//ros::Subscriber steering_controller_pos_sub_; 
		//! Steering controller status
		ros::Subscriber steering_controller_status_sub_; 
		
		ros::ServiceServer set_odometry_service_server_;
		//! sets the state of the motordrive
		ros::ServiceClient set_motor_steering_state_service_client_;
		
		//! General status diagnostic updater
		diagnostic_updater::Updater *diagnostic_;
		//! Diagnosis for the curtis data recepction
		diagnostic_updater::HeaderlessTopicDiagnostic *curtis_data_freq_, *steering_position_freq_;	  
		
		//! Robot odometry
		nav_msgs::Odometry odometry;
		//! Robot joints state
		//sensor_msgs::JointState joints;
		//! Frame for odometry
		string odom_frame_;
		//! Frame for the child frame (typically base_footprint)
		string child_frame_;
		//! Saves the joint values to publish the state
		map<string, SingleJointState> joints;
		//! Joint names of the vehicle
		string left_rear_joint_, right_rear_joint_, left_front_joint_, right_front_joint_, left_steering_joint_, right_steering_joint_;
		string left_front_shock_joint_, right_front_shock_joint_, left_rear_shock_joint_, right_rear_shock_joint_;
		//! Topic name of the curtis_controller DriveData
		string curtis_data_topic_;
		//! Topic name of the steering controller position
		string steering_controller_topic_;
		//! Frequency limits for topic's freq diagnostics 
		double min_data_topic_freq, max_data_topic_freq;
		//! Topic to publish position commands
		string cmd_pos_topic_;
		//! Topic to publish velocity commands
		string cmd_vel_topic_;
		//! Service client name to set the steering motor status
		string set_motor_status_service_;
		
		//! Saves the reception of commands and messages
		ros::Time last_command_time;
		ros::Time last_curtis_data_time;
		ros::Time last_steering_data_time;
		
		//! Data from curtis controller
		curtis_msgs::DriveData curtis_data;
		//! Status of the steering controller 
		rbcar_steering_controller::SteeringControllerStatus steering_controller_status;
		//! Ackermann Command 
		ackermann_msgs::AckermannDriveStamped ackermann_command;
		
		//! Robot control mode
		int control_mode;
		//! Params for odometry calculation
		OdometryParams odom_params;
		//! Flag to enable/disable the tf brodcast
		bool publish_tf_;
		//! Object to publish the odom transformation
		tf::TransformBroadcaster tf_broadcaster;
		//! max speed
		double max_speed_;
		
		
	public:
		//! Public constructor
		RbCarController(ros::NodeHandle h);
		//! Public destructor
		~RbCarController();
		
		//! Starts the control loop of the component and its subcomponents
		//! @return OK
		//! @return ERROR starting the thread
		//! @return RUNNING if it's already running
		//! @return NOT_INITIALIZED if it's not initialized
		int start();
		//! Stops the main control loop of the component and its subcomponents
		//! @return OK
		//! @return ERROR if any error has been produced
		//! @return NOT_RUNNING if the main thread isn't running
		int stop();
		//! Returns the general state of the RbCarController
		int getState();
		//! Returns the general state of the RbCarController as string
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
		int setup();
		//! Closes and frees the reserved resources
		//! @return OK
		//! @return ERROR if fails when closes the devices
		//! @return RUNNING if the component is running
		//! @return NOT_INITIALIZED if the component is not initialized
		int shutdown();
		//! All core component functionality is contained in this thread.
		//!	All of the RbCarController component state machine code can be found here.
		void controlLoop();
		//! Actions performed on initial state
		void initState();
		//! Actions performed on standby state
		void standbyState();
		//! Actions performed on ready state
		void readyState();
		//! Actions performed on the emergency state
		void emergencyState();
		//! Actions performed on Failure state
		void failureState();
		//! Actions performed on Shudown state
		void shutdownState();
		//! Actions performed in all states
		void allState();
		//! Switches between states
		void switchToState(int new_state);
		//! Setups all the ROS' stuff
		int rosSetup();
		//! Shutdowns all the ROS' stuff
		int rosShutdown();
		//! Reads data a publish several info into different topics
		void rosPublish();
		//! Reads params from params server
		void rosReadParams();
		//! Callback to process ackermann command messages
		void ackermannCommandCallback(const ackermann_msgs::AckermannDriveStampedConstPtr& message); 
		//! Callback to process differential command messages
        void differentialCommandCallback(const geometry_msgs::TwistConstPtr& message);
		//! Callback to process curtis drive data information
		void curtisDataCallback(const curtis_msgs::DriveDataConstPtr& message);
		//! Callback to process the steering controller position
		void steeringPositionCallback(const rbcar_steering_controller::SteeringControllerStatusConstPtr& message);
		
		// bool serviceServerCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); // Callback for a service server
		
		//! Diagnostic updater callback
		void diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat);
		//! Inits the odometry
		void initOdometry();
		//! Inits the joints values and map them	
		void initJoints();
		//! Updates the robot's odometry
		void updateOdometry();
		//! Sets the control mode from the received data
		void checkControlMode();
		//! Sets the motor velocity
		void setMotorVelocity(double vel);
		//! Sets the motor position
		void setSteeringPosition(double pos);
		//! Normalizes the number between +-Pi
		void radnorm(double* radians);
		//! Sets robot odometry
		bool setOdometryServiceCb(robotnik_msgs::set_odometry::Request &req, robotnik_msgs::set_odometry::Response &res );
		//! Sets the steering motor status
		bool setSteeringMotorStatus(int status);
};


/*! \fn RbCarController::RbCarController()
 *  \brief Constructor by default
 *	\param hz as double, sets the desired frequency of the controlthread
 *	\param h as ros::NodeHandle, ROS node handle
*/
RbCarController::RbCarController(ros::NodeHandle h):nh_(h), pnh_("~"){
	// Set main flags to false
	ros_initialized = initialized = running = false;
	// reads params from server
	rosReadParams();
	
	if(desired_freq_ <= 0.0)
		desired_freq_ = DEFAULT_THREAD_DESIRED_HZ;
		
	min_data_topic_freq = 5.0;
	max_data_topic_freq = 100.0;
	
	state = robotnik_msgs::State::INIT_STATE;
	// Realizar para cada una de las clases derivadas
	component_name.assign("RbCarController");
	
	control_mode = RBCAR_UNKNOWN_MODE;
}

/*! \fn RbCarController::~RbCarController()
 * Destructor by default
*/
RbCarController::~RbCarController(){
	
}

/*! \fn int RbCarController::setup()
 * Configures and initializes the component
 * \return OK
 * \return INITIALIZED if the component is already intialized
 * \return ERROR
*/
int RbCarController::setup(){
	// Checks if has been initialized
	if(initialized){
		ROS_INFO("%s::Setup: Already initialized",component_name.c_str());
		
		return INITIALIZED;
	}
	
	//
	///////////////////////////////////////////////////
	// Setups the component or another subcomponents if it's necessary //
	///////////////////////////////////////////////////
	initOdometry();
	initJoints();

	initialized = true;

	return OK;
}

/*! \fn int RbCarController::shutDown()
 * Closes and frees the reserved resources
 * \return OK
 * \return ERROR if fails when closes the devices
 * \return RUNNING if the component is running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int RbCarController::shutdown(){
	
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
	
	
	initialized = false;

	return OK;
}


/*! \fn int RbCarController::start()
 * Starts the control thread of the component and its subcomponents
 * \return OK
 * \return RUNNING if it's already running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int RbCarController::start(){
	// Performs ROS setup
	rosSetup();
	
	if(running){
		ROS_INFO("%s::start: the component's thread is already running", component_name.c_str());
		return THREAD_RUNNING;
	}
	
	ROS_INFO("%s started", component_name.c_str());
	
	running = true;
	
	// Executes the control loop
	controlLoop();
	
	return OK;

}

/*! \fn int RbCarController::stop()
 * Stops the control thread of the Motors
 * \return OK
 * \return ERROR if it can't be stopped
 * \return THREAD_NOT_RUNNING if the thread is not running
*/
int RbCarController::stop(){
	
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

/*!	\fn void RbCarController::controlLoop()
 *	\brief All core component functionality is contained in this thread.
*/
void RbCarController::controlLoop(){
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

/*!	\fn void RbCarController::initState()
 *	\brief Actions performed on initial 
 * 	Setups the component
*/
void RbCarController::initState(){
	// If component setup is successful goes to STANDBY (or READY) state
	if(setup() != ERROR){
		switchToState(robotnik_msgs::State::READY_STATE);
	}
}

/*!	\fn void RbCarController::shutdownState()
 *	\brief Actions performed on Shutdown state
*/
void RbCarController::shutdownState(){
	
	if(shutdown() == OK){
		switchToState(robotnik_msgs::State::INIT_STATE);
	}
}

/*!	\fn void RbCarController::standbyState()
 *	\brief Actions performed on Standby state
*/
void RbCarController::standbyState(){

}

/*!	\fn void RbCarController::readyState()
 *	\brief Actions performed on ready state
*/
void RbCarController::readyState(){
	
	ros::Time t_now = ros::Time::now();
	
	// State transitions
	if( (t_now - last_curtis_data_time).toSec() >  RBCAR_TIMEOUT_MSGS || (t_now - last_steering_data_time).toSec() >  RBCAR_TIMEOUT_MSGS){ // ||-> or
		// No data from subcomponents
		switchToState(robotnik_msgs::State::EMERGENCY_STATE);
	}
	
	if(control_mode == RBCAR_AUTO_MODE){
			
		if( (t_now - last_command_time).toSec()  > RBCAR_COMMAND_WATCHDOG){
			setMotorVelocity(0.0);
		}else{
			setMotorVelocity(ackermann_command.drive.speed);
			setSteeringPosition(ackermann_command.drive.steering_angle);	
		}
	}
	
	
	updateOdometry();
}

/*!	\fn void RbCarController::EmergencyState()
 *	\brief Actions performed on emergency state
*/
void RbCarController::emergencyState(){
	ros::Time t_now = ros::Time::now();
	
	// State transitions
	if( (t_now - last_curtis_data_time).toSec() <  RBCAR_TIMEOUT_MSGS && (t_now - last_steering_data_time).toSec() <  RBCAR_TIMEOUT_MSGS){ // && -> and
		// Data received
		switchToState(robotnik_msgs::State::READY_STATE);
	}
	
}

/*!	\fn void RbCarController::FailureState()
 *	\brief Actions performed on failure state
*/
void RbCarController::failureState(){

}

/*!	\fn void RbCarController::AllState()
 *	\brief Actions performed on all states
*/
void RbCarController::allState(){
	
	diagnostic_->update();
	
	checkControlMode();
	
	rosPublish();
}

/*!	\fn double RbCarController::getUpdateRate()
 * 	\brief Gets current update rate of the thread
 * 	\return real frequency of the thread
*/
double RbCarController::getUpdateRate(){
	return desired_freq_;
}

/*!	\fn int RbCarController::getState()
 * 	\brief returns the state of the component
*/
int RbCarController::getState(){
	return state;
}

/*!	\fn char *RbCarController::getStateString()
 *	\brief Gets the state of the component as string
*/
char *RbCarController::getStateString(){
	return getStateString(state);
}

/*!	\fn char *RbCarController::getStateString(int state)
 *	\brief Gets the state as a string
*/
char *RbCarController::getStateString(int state){
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


/*!	\fn void RbCarController::switchToState(int new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void RbCarController::switchToState(int new_state){
	
	if(new_state == state)
		return;

	// saves the previous state
	previous_state = state;
	ROS_INFO("%s::SwitchToState: %s -> %s", component_name.c_str(), getStateString(state), getStateString(new_state));	
	state = new_state;

}

/*!	\fn void RbCarController::rosSetup()
 * 	\brief Setups all ROS' stuff
*/
int RbCarController::rosSetup(){
	
	// Checks if has been initialized
	if(ros_initialized){
		ROS_INFO("%s::rosSetup: Already initialized",component_name.c_str());
		
		return INITIALIZED;
	}
	
	// Publishers
	state_pub_ = pnh_.advertise<robotnik_msgs::State>("state", 1);
	odom_pub_ = pnh_.advertise<nav_msgs::Odometry>("odom", 1);
	joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
	cmd_pos_pub_ = nh_.advertise<std_msgs::Float64>(cmd_pos_topic_, 1);
	
	// Subscribers
	ackermann_command_sub_ = pnh_.subscribe("command", 10,  &RbCarController::ackermannCommandCallback, this);	
	differential_command_sub_ = pnh_.subscribe("command_differential", 10,  &RbCarController::differentialCommandCallback, this);	
	curtis_controller_sub_ = nh_.subscribe(curtis_data_topic_, 10,  &RbCarController::curtisDataCallback, this);	
	steering_controller_status_sub_ = nh_.subscribe(steering_controller_topic_, 10,  &RbCarController::steeringPositionCallback, this);	
	
	// Service servers
	set_odometry_service_server_ = pnh_.advertiseService("set_odometry", &RbCarController::setOdometryServiceCb, this);
	// Services client
	set_motor_steering_state_service_client_ = nh_.serviceClient<rbcar_steering_controller::SetMotorStatus>(set_motor_status_service_);
	
	ros_initialized = true;
	

	
	// Sets up the diagnostic updater
	diagnostic_ = new diagnostic_updater::Updater();
	curtis_data_freq_ = new diagnostic_updater::HeaderlessTopicDiagnostic(curtis_data_topic_, *diagnostic_,
	                    diagnostic_updater::FrequencyStatusParam(&min_data_topic_freq, &max_data_topic_freq, 0.1, 10));
	steering_position_freq_ = new diagnostic_updater::HeaderlessTopicDiagnostic(steering_controller_topic_, *diagnostic_,
	                    diagnostic_updater::FrequencyStatusParam(&min_data_topic_freq, &max_data_topic_freq, 0.1, 10));
	
	diagnostic_->setHardwareID("RbCarController");
	diagnostic_->add("State", this, &RbCarController::diagnosticUpdate);
	diagnostic_->broadcast(0, "Doing important initialization stuff.");
	return OK;

}


/*!	\fn void RbCarController::rosReadParams
 * 	\brief Reads the params set in ros param server
*/
void RbCarController::rosReadParams(){
	
	pnh_.param("desired_freq", desired_freq_, DEFAULT_THREAD_DESIRED_HZ); 
	pnh_.param<std::string>("odom_frame", odom_frame_, RBCAR_DEFAULT_ODOM_FRAME);
	pnh_.param<std::string>("child_frame", child_frame_, RBCAR_DEFAULT_CHILD_FRAME);
	
	pnh_.param<std::string>("left_rear_joint", left_rear_joint_, RBCAR_DEFAULT_LEFT_REAR_JOINT);
	pnh_.param<std::string>("right_rear_joint", right_rear_joint_, RBCAR_DEFAULT_RIGHT_REAR_JOINT);
	pnh_.param<std::string>("left_front_joint", left_front_joint_, RBCAR_DEFAULT_LEFT_FRONT_JOINT);
	pnh_.param<std::string>("right_front_joint", right_front_joint_, RBCAR_DEFAULT_RIGHT_FRONT_JOINT);
	pnh_.param<std::string>("right_steering_joint", right_steering_joint_, RBCAR_DEFAULT_RIGHT_STEERING_JOINT);
	pnh_.param<std::string>("left_steering_joint", left_steering_joint_, RBCAR_DEFAULT_LEFT_STEERING_JOINT);
	pnh_.param<std::string>("left_front_shock_joint", left_front_shock_joint_, RBCAR_DEFAULT_LEFT_FRONT_SHOCK_JOINT);
	pnh_.param<std::string>("right_front_shock_joint", right_front_shock_joint_, RBCAR_DEFAULT_RIGHT_FRONT_SHOCK_JOINT);
	pnh_.param<std::string>("left_rear_shock_joint", left_rear_shock_joint_, RBCAR_DEFAULT_LEFT_REAR_SHOCK_JOINT);
	pnh_.param<std::string>("right_rear_shock_joint", right_rear_shock_joint_, RBCAR_DEFAULT_RIGHT_REAR_SHOCK_JOINT);
	
	pnh_.param<std::string>("curtis_data_topic", curtis_data_topic_, "curtis_controller/drive_data");
	pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "curtis_controller/command");
	
	pnh_.param<std::string>("steering_position_topic", steering_controller_topic_, "rbcar_steering_controller/data");
	pnh_.param<std::string>("cmd_pos_topic", cmd_pos_topic_, "rbcar_steering_controller/command");
	
	pnh_.param<std::string>("set_motor_status_service", set_motor_status_service_, "rbcar_steering_controller/set_motor_status");
	
	pnh_.param("publish_tf", publish_tf_, false);
	
	pnh_.param("max_speed", max_speed_, MAX_SPEED);
	
	/*pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "/odom_diff");
	pnh_.param<std::string>("base_frame_id", base_frame_id_, "/base_link");
	
	pnh_.param("desired_freq", desired_freq_, desired_freq_);*/
}

/*!	\fn int RbCarController::rosShutdown()
 * 	\brief Closes all ros stuff
*/
int RbCarController::rosShutdown(){
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

/*!	\fn void RbCarController::rosPublish()
 * 	\brief Reads data a publish several info into different topics
*/
void RbCarController::rosPublish(){
	robotnik_msgs::State msg;
	sensor_msgs::JointState joints_state;
	ros::Time t_now = ros::Time::now();
	
	// STATE
	msg.state = this->state;
	msg.desired_freq = this->desired_freq_;
	msg.real_freq = this->real_freq;
	msg.state_description = getStateString();
	
	state_pub_.publish(msg);
	
	// ODOMETRY
	odometry.header.stamp = t_now;
	odom_pub_.publish(odometry);
	
	// JOINT STATES
	joints["right_steering_joint"].position = steering_controller_status.steering_position;
	joints["left_steering_joint"].position = steering_controller_status.steering_position;
	for (map<string,SingleJointState>::iterator it=joints.begin(); it!=joints.end(); ++it){
		//std::cout << it->first << " => " << it->second << '\n';
		joints_state.name.push_back(it->second.joint_name);
		joints_state.position.push_back(it->second.position);
		joints_state.velocity.push_back(it->second.velocity);
		joints_state.effort.push_back(0.0);
	}
	joints_state.header.stamp = t_now;
	joint_state_pub_.publish(joints_state);
	
	// Publish odom transformation
	if(publish_tf_){
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = t_now;
		odom_trans.header.frame_id = odom_frame_;
		odom_trans.child_frame_id = child_frame_;
		odom_trans.transform.translation.x = odometry.pose.pose.position.x;   
		odom_trans.transform.translation.y = odometry.pose.pose.position.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odometry.pose.pose.orientation;
		tf_broadcaster.sendTransform(odom_trans);
	}
}

/*!	\fn void RbCarController::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat)
 * 	\brief Callback to update the component diagnostic
*/
void RbCarController::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat){
	
	if(state == robotnik_msgs::State::READY_STATE || state == robotnik_msgs::State::INIT_STATE || state == robotnik_msgs::State::STANDBY_STATE)
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Everything OK!");
	else if (state == robotnik_msgs::State::EMERGENCY_STATE)
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Watch out!");
	else
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Error!");
	
	stat.add("State", getStateString());
}
	
	
/*!	\fn RbCarController::ackermannCommandCallback(const ackermann_msgs::AckermannDriveStampedConstPtr& message)
 * 	\brief Callback to process ackermann commands
*/	
void RbCarController::ackermannCommandCallback(const ackermann_msgs::AckermannDriveStampedConstPtr& message)
{
	//ROS_INFO("ackermannCommandCallback: Received msg");
	last_command_time = ros::Time::now();
	
	ackermann_command = *message;
}

/*!	\fn RbCarController::differentialCommandCallback(const geometry_msgs::TwistConstPtr& message)
 * 	\brief Callback to process ackermann commands
*/	
void RbCarController::differentialCommandCallback(const geometry_msgs::TwistConstPtr& message)
{
	//ROS_INFO("differentialCommandCallback: Received msg");
	last_command_time = ros::Time::now();

	double speed = sgn(message->linear.x)*sqrt(message->linear.x*message->linear.x + message->angular.z*message->angular.z*RBCAR_DEFAULT_DIST_AXIS_WHEELS*RBCAR_DEFAULT_DIST_AXIS_WHEELS);
	double steering = atan2(RBCAR_DEFAULT_DIST_AXIS_WHEELS * message->angular.z, message->linear.x);

	if (sgn(message->linear.x) < 0) {
		if (message->angular.z> 0) //sgn(message->linear.x)
			steering = steering - M_PI;
		if (message->angular.z < 0)
			steering = steering + M_PI;
		if(message->angular.z == 0 || message->angular.z == -0.0)
			steering = 0;
	}

	ROS_INFO_THROTTLE(1, "linear: %f, angular: %f\n, speed: %f, steering: %f", message->linear.x, message->angular.z, speed, steering);

    ackermann_command.header.stamp = last_command_time;
    ackermann_command.drive.speed = speed;
    ackermann_command.drive.acceleration = 0;
    ackermann_command.drive.jerk = 0;
    ackermann_command.drive.steering_angle = steering; 
    ackermann_command.drive.steering_angle_velocity = 0;

}

/*!	\fn void RbCarController::curtisDataCallback(const curtis_msgs::DriveDataConstPtr& message)
 * 	\brief Callback to process curtis drive data information
*/	
void RbCarController::curtisDataCallback(const curtis_msgs::DriveDataConstPtr& message)
{

	//ROS_INFO("RbCarController::curtisDataCallback: Received msg");
	curtis_data = *message;
	last_curtis_data_time = ros::Time::now();
	curtis_data_freq_->tick();
	
}


/*!	\fn void RbCarController::steeringPositionCallback(const rbcar_steering_controller::SteeringControllerStatusConstPtr& message)
 * 	\brief Callback to process steering controller position
*/	
void RbCarController::steeringPositionCallback(const rbcar_steering_controller::SteeringControllerStatusConstPtr& message)
{
	//ROS_INFO("RbCarController::steeringPositionCallback: Received msg");
	steering_controller_status = *message;
	last_steering_data_time = ros::Time::now();
	steering_position_freq_->tick();
}



// Callback handler for the service server
/*!	\fn bool RbCarController::setOdometryServiceCb(robotnik_msgs::set_odometry::Request &req, robotnik_msgs::set_odometry::Response &res )
 * 	\brief Sets the odometry of the robot
*/	
bool RbCarController::setOdometryServiceCb(robotnik_msgs::set_odometry::Request &req, robotnik_msgs::set_odometry::Response &res )
{
	ROS_INFO("RbCarController::setOdometryServiceCb: x = %lf, y= %lf, z= %lf, orientation = %lf", req.x, req.y, req.z, req.orientation);
	odometry.pose.pose.position.x = req.x;
	odometry.pose.pose.position.y = req.y;
	odometry.pose.pose.position.z = req.z;
	odometry.pose.pose.orientation.z = req.orientation;
	
	return true;
}

/*!	\fn void RbCarController::initOdometry()
 * 	\brief Inits the odometry
*/
void RbCarController::initOdometry(){
	odometry.pose.pose.position.x = 0.0;
	odometry.pose.pose.position.y = 0.0;
	odometry.pose.pose.position.z = 0.0;
	odometry.pose.pose.orientation.x = 0.0;
	odometry.pose.pose.orientation.y = 0.0;
	odometry.pose.pose.orientation.z = 0.0;
	odometry.pose.pose.orientation.w = 1.0;
	odometry.twist.twist.linear.x = 0.0;
	odometry.twist.twist.linear.y = 0.0;
	odometry.twist.twist.linear.z = 0.0;
	odometry.twist.twist.angular.x = 0.0;
	odometry.twist.twist.angular.y = 0.0;
	odometry.twist.twist.angular.z = 0.0;
	odometry.header.frame_id = odom_frame_;
	odometry.child_frame_id = child_frame_;
	
	// odom params
	odom_params.d_axis_wheels = RBCAR_DEFAULT_DIST_AXIS_WHEELS;
}

/*!	\fn void RbCarController::initJoints()
 * 	\brief Inits the joints values and map them
*/
void RbCarController::initJoints(){
	SingleJointState j;
	
	/*string joint_name;
	double position;
	double velocity;*/
	
	j.joint_name = left_rear_joint_;
	j.position = 0.0;
	j.velocity = 0.0;
	joints["left_rear_joint"] = j;
	
	j.joint_name = right_rear_joint_;
	joints["right_rear_joint"] = j;
	
	j.joint_name = left_front_joint_;
	joints["left_front_joint"] = j;
	
	j.joint_name = right_front_joint_;
	joints["right_front_joint"] = j;
	
	j.joint_name = left_steering_joint_;
	joints["left_steering_joint"] = j;
	
	j.joint_name = right_steering_joint_;
	joints["right_steering_joint"] = j;
	
	j.joint_name = left_front_shock_joint_;
	joints["left_front_shock_joint"] = j;
	
	j.joint_name = right_front_shock_joint_;
	joints["right_front_shock_joint"] = j;
	
	j.joint_name = left_rear_shock_joint_;
	joints["left_rear_shock_joint"] = j;
	
	j.joint_name = right_rear_shock_joint_;
	joints["right_rear_shock_joint"] = j;
	
	
	
	
}

/*!	\fn void RbCarController::updateOdometry()
 * 	\brief Updates the robot's odometry
*/
void RbCarController::updateOdometry(){
	
	int cursor = 0;
	double sample_period = 0.0;
	double beta_rads = steering_controller_status.steering_position;  	// current orientation angle (for ackerman odometry)
	double v_mps = curtis_data.speed; 								// Velocity from curtis controller
	double d = odom_params.d_axis_wheels; 				// distance between center of back axis to front axis
	double w = (v_mps / d) * sin(beta_rads);
	double yaw = tf::getYaw(odometry.pose.pose.orientation);
	
	if (real_freq > 0.0)
		sample_period = 1.0 / real_freq;

    	//ROS_INFO("RbCarController::updateOdometry: ");

    	// Odometry calculation (using motor speed and position of the motor direction)
	// odometry.pose.pose.orientation.z += w*sample_period;
	yaw += w*sample_period;

	// Normalize in +-Pi rads
   	radnorm(&yaw);
    	//radnorm(&odometry.pose.pose.orientation.z);
    	odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    
    	odometry.twist.twist.linear.x = v_mps * cos(beta_rads) * cos(yaw);
   	odometry.twist.twist.linear.y = v_mps * cos(beta_rads) * sin(yaw);
    	odometry.twist.twist.angular.z = w;	
    	odometry.pose.pose.position.x += odometry.twist.twist.linear.x * sample_period;
   	odometry.pose.pose.position.y += odometry.twist.twist.linear.y * sample_period;

	for(cursor = 0; cursor<3; cursor++){
		odometry.pose.covariance[cursor*6+cursor] = 10.017453295;
		odometry.twist.covariance[cursor*6+cursor] = 10.00872664625;
	}
	odometry.pose.covariance[2*6+2]=99999;
	odometry.twist.covariance[2*6+2] = 99999;
	for(cursor = 3; cursor<6; cursor++){
		odometry.pose.covariance[cursor*6+cursor] = 99999;
		odometry.twist.covariance[cursor*6+cursor] = 99999;
	}
	odometry.pose.covariance[5*6+5]=10.017453295;
	odometry.twist.covariance[5*6+5]=10.00872664625;
	
}

/*!	\fn void RbCarController::checkControlMode()
 * 	\brief 
*/
void RbCarController::checkControlMode(){
	ros::Time t_now = ros::Time::now();
	
	// Checks the data is being received
	if((t_now - last_curtis_data_time).toSec() >  RBCAR_TIMEOUT_MSGS){
		control_mode = RBCAR_UNKNOWN_MODE;
	}else{
	
		if(curtis_data.mode_auto and control_mode != RBCAR_AUTO_MODE){
			control_mode = RBCAR_AUTO_MODE;	
			// Enables the steering motor
			setSteeringMotorStatus(STEERING_MOTOR_ENABLED);
		
		}else if(not curtis_data.mode_auto and control_mode != RBCAR_MANUAL_MODE){
			control_mode = RBCAR_MANUAL_MODE;
			// Disables the steering motor
			setSteeringMotorStatus(STEERING_MOTOR_DISABLED);// STEERING_MOTOR_DISABLED
		}
	}
}

/*!	\fn void RbCarController::setMotorVelocity(double vel)
 * 	\brief Sends the velocity message to the motor controller
*/
void RbCarController::setMotorVelocity(double vel){
	geometry_msgs::Twist command;
	
	if(vel > 0.0 and vel > max_speed_)
		vel = max_speed_;
	else if(vel < 0.0 and vel < -max_speed_)
		vel = -max_speed_;
		
	command.linear.x = vel;
	cmd_vel_pub_.publish(command);
}

/*!	\fn void RbCarController::setSteeringPosition(double pos)
 * 	\brief Sends the steering position to the motor controller
*/
void RbCarController::setSteeringPosition(double pos){
	std_msgs::Float64 command;
	
	
	// TEST
	int counts;
	
	if(pos > 0.0 and pos > MAX_STEERING_ANGLE)
		pos = MAX_STEERING_ANGLE;
	else if (pos < 0.0 and pos < -MAX_STEERING_ANGLE)
		pos = -MAX_STEERING_ANGLE;

	command.data = pos;
	
	
	cmd_pos_pub_.publish(command);
}


/*!	\fn void RbCarController::radnorm(double* radians)
 * 	\brief Normalizes between -PI and PI
*/
void RbCarController::radnorm(double* radians)
{
	while (*radians >= (M_PI)) {
		*radians -= 2.0 * M_PI;
	}
	while (*radians <= (-M_PI)) {
		*radians += 2.0 * M_PI;
	}
}


/*!	\fn bool RbCarController::setSteeringMotorStatus(int status
 * 	\brief Sets the steering motor status
*/
bool RbCarController::setSteeringMotorStatus(int status){
	rbcar_steering_controller::SetMotorStatus srv;
	
	if(status == STEERING_MOTOR_DISABLED)
		srv.request.status = 2;
	else if(status == STEERING_MOTOR_ENABLED)
		srv.request.status = 0;
	
	//ROS_INFO("RbCarController::setSteeringMotorStatus to %d", status);
	
	set_motor_steering_state_service_client_.call(srv);
	
}
		
		


// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "rbcar_controller");
	
	ros::NodeHandle n;		
  	RbCarController controller(n);
	
	controller.start();

	return (0);
}

