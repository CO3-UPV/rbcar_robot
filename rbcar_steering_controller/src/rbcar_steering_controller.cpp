/** \file rbcar_steering_controller.cc
 * \author Robotnik Automation S.L.L.
 * \version 1.0
 * \date    2015
 *
 * \brief RbcarSteeringNode class driver
 * Component control a steering motor by using a ROBUR drive
 * (C) 2015 Robotnik Automation, SLL
*/

#include <string.h>
#include <vector>
#include <stdint.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>               // battery, gyro, and steering angle
#include <std_msgs/Float64.h>               // battery, gyro, and steering angle
#include <std_msgs/Int32.h>               
#include <std_msgs/Bool.h>					// 
#include <math.h>
#include <cstdlib>

#include <robotnik_msgs/MotorStatus.h>
#include <rbcar_steering_controller/EpcEncoderStatus.h>
#include <sensor_msgs/JointState.h>

//#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include <rbcar_steering_controller/MotorPosition.h>
#include <rbcar_steering_controller/EpcEncoder.h>
#include <rbcar_steering_controller/SetMotorStatus.h>
#include <rbcar_steering_controller/SteeringControllerStatus.h>


#define RBCAR_MIN_COMMAND_REC_FREQ			1.0
#define	RBCAR_MAX_COMMAND_REC_FREQ			20.0

#define RBCAR_DEFAULT_ID_MOTOR				1
#define RBCAR_DEFAULT_ID_ENCODER			0x7F
#define RBCAR_DEFAULT_HZ					50.0

#define RBCAR_DEFAULT_ABS_ENC_RESOLUTION	4096 // counts/rev
#define INCREMENTAL_ENCODER_RESOLUTION		1000 // counts/rev



using namespace std;


//! Struct to save the configuration to control the steering position 
struct SteeringConfig{
	// Read from ROS param server
	int abs_encoder_right;
	int abs_encoder_left;
	int motor_encoder_left;
	int motor_encoder_right;
	int abs_encoder_resolution;
	double steering_angle_resolution;	// Of the theoric central axis
	int abs_encoder_zero_position;		// Encoder counts for the 0 steering position
	// Calculated
	int abs_counts_range;				// Total counts from left to right (right - left)
	int motor_counts_range;				// Total counts from left to right (right - left)
	int abs_limit_left;					// Counts limit supposing 0 counts is the center
	int abs_limit_right;				// Counts limit supposing 0 counts is the center
	int abs_to_motor;					// Relation between absolute and motor counts
	double a;							// 'a' param for the linear function to convert from abs counts to angle (y = ax + b)
	double b;							// 'b' param for the linear function to convert from abs counts to angle (y = ax + b)
		
	double extrem_angle_left;
	double extrem_angle_right;
	
	int extrem_points_left; 
	int extrem_points_right; 
	
	int number_points_considered_zero;
		
	double value_calibrate_a; 
	double value_calibrate_b;
	
};

class RbcarSteeringNode
{

private:
	self_test::TestRunner self_test_;
	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	double desired_freq_;
	diagnostic_updater::Updater diagnostic_;					 	// General status diagnostic updater
	diagnostic_updater::FrequencyStatus freq_diag_;		         	// Component frequency diagnostics
	diagnostic_updater::HeaderlessTopicDiagnostic *subs_command_freq; // Topic reception frequency diagnostics
    diagnostic_updater::FunctionDiagnosticTask command_freq_;
	//! Topic set the motor position on counts
	ros::Subscriber cmd_position_counts_;
	ros::Subscriber cmd_angle_;
	//! Topic to publish the current motor counts
	ros::Publisher pub_status_;  
	
	// Services
	ros::ServiceServer set_motor_status_;
	
	//! CAN ID
	int can_motor_id_, can_encoder_id_;
	//! Can device port
	string can_dev_;
	//! CAN device
	PCan *canDev;
	//! motor for steering
	MotorPosition *motorSteering;
	//! absolute encoder to read the position of the joint direction
	EpcEncoder *encoderSteering;
	
	bool running;
	//! Flag to set the status of can comm
	bool can_communication;
	//! Saves the time of the last command received
	ros::Time last_command_time;
	
	
	int publish_status_counter;
	int publish_status_freq; // number of loop cycles to publish the status
	
	//! Saves the configuration for the steering position control
	SteeringConfig steering_config_;
	//! Motor counts, absolute encoder counts
	std_msgs::Int32 motor_counts, encoder_counts, deb_counts;
	//! Current computed angle
	std_msgs::Float64 current_angle;
	//! Component's status
	rbcar_steering_controller::SteeringControllerStatus status;
	
public:


/*!	\fn summit_controller::rbcar_steering_controller()
 * 	\brief Public constructor
*/
RbcarSteeringNode(ros::NodeHandle h) : self_test_(), diagnostic_(),node_handle_(h), private_node_handle_("~"), 
	desired_freq_(RBCAR_DEFAULT_HZ),
	freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05)   ),
	command_freq_("Command frequency check", boost::bind(&RbcarSteeringNode::checkCommandSubscriber, this, _1))
  {
    running = false;
    can_communication = false;

    private_node_handle_.param<std::string>("port", can_dev_, "/dev/pcan32");
    private_node_handle_.param<int>("can_motor_id", can_motor_id_, RBCAR_DEFAULT_ID_MOTOR);
    private_node_handle_.param<int>("can_encoder_id", can_encoder_id_, RBCAR_DEFAULT_ID_ENCODER);
    private_node_handle_.param<int>("abs_encoder_left", steering_config_.abs_encoder_left, 0);
    private_node_handle_.param<int>("abs_encoder_right", steering_config_.abs_encoder_right, 0);
    private_node_handle_.param<int>("motor_encoder_left", steering_config_.motor_encoder_left, 0);
    private_node_handle_.param<int>("motor_encoder_right", steering_config_.motor_encoder_right, 0);
    private_node_handle_.param<int>("abs_encoder_resolution", steering_config_.abs_encoder_resolution, RBCAR_DEFAULT_ABS_ENC_RESOLUTION);
    private_node_handle_.param<int>("abs_encoder_zero_position", steering_config_.abs_encoder_zero_position, 3109);
    private_node_handle_.param<double>("steering_angle_resolution", steering_config_.steering_angle_resolution, 0.45);
    
    
    private_node_handle_.param<double>("extrem_angle_left", steering_config_.extrem_angle_left, 0.56);
    private_node_handle_.param<double>("extrem_angle_right", steering_config_.extrem_angle_right, 0.625);
    private_node_handle_.param<int>("extrem_points_left", steering_config_.extrem_points_left, 2675);
    private_node_handle_.param<int>("extrem_points_right", steering_config_.extrem_points_right,3603);
    private_node_handle_.param<int>("number_points_considered_zero", steering_config_.number_points_considered_zero, 10);
    private_node_handle_.param<double>("value_calibrate_a", steering_config_.value_calibrate_a, 0.0012946926);
    private_node_handle_.param<double>("value_calibrate_b", steering_config_.value_calibrate_b, 4.022851115);
    
    //private_node_handle_.param<double>("gearbox_reduction", gearbox_, TRAC_GEAR);
    
	// Subscribers
	cmd_position_counts_ = private_node_handle_.subscribe<std_msgs::Int32>("command_position_counts", 1, &RbcarSteeringNode::cmdPositionCountsCallback, this);
	cmd_angle_ = private_node_handle_.subscribe<std_msgs::Float64>("command_angle", 1, &RbcarSteeringNode::cmdAngleCallback, this);
	
	// Publishers
	pub_status_ = private_node_handle_.advertise<rbcar_steering_controller::SteeringControllerStatus>("status", 10);
    // Services
    set_motor_status_ = private_node_handle_.advertiseService("set_motor_status", &RbcarSteeringNode::setMotorStatus, this);
    
  	// Creation of every object
    canDev = new PCan( can_dev_, CAN_BAUD_1M );  // default com speed 1M

	motorSteering = new MotorPosition(can_motor_id_, canDev, desired_freq_);
	encoderSteering = new EpcEncoder(can_encoder_id_, canDev, desired_freq_);
	
    self_test_.add("Connect Test", this, &RbcarSteeringNode::connectTest);

    // Component frequency diagnostics
    diagnostic_.setHardwareID("rbcar_steering_controller");
    diagnostic_.add("Motor Controller", this, &RbcarSteeringNode::controllerDiagnostic);
    diagnostic_.add( freq_diag_ );
    diagnostic_.add( command_freq_ );
    
    // Topics freq control 
    // For /rbcar_steering_controller/command
    double min_freq = RBCAR_MIN_COMMAND_REC_FREQ; // If you update these values, the
    double max_freq = RBCAR_MAX_COMMAND_REC_FREQ; // HeaderlessTopicDiagnostic will use the new values.
    subs_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/rbcar_steering_controller/command", diagnostic_,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq, 0.1, 10));
    subs_command_freq->addTask(&command_freq_); // Adding an additional task to the control

    publish_status_freq = (int)desired_freq_;
    publish_status_counter=0;
    
    // sets the params for the steering control
    setSteeringParams();
  }

/*!	\fn rbcar_steering_controller::~rbcar_steering_controller()
 * 	\brief Public destructor
*/
~RbcarSteeringNode(){

	stop();
   

    delete subs_command_freq;


}

/*!\fn void rbcar_steering_controller::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
  * Callback - velocity references 
*/
void cmdPositionCountsCallback(const std_msgs::Int32::ConstPtr& cmd)
{
	//ROS_INFO("rbcar_steering_controller::cmdPositionCountsCallback: value = %d",cmd->data);
	last_command_time = ros::Time::now();		
	motorSteering->SetMotorPosition(cmd->data);
}


/*!\fn void rbcar_steering_controller::cmdAngleCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
  * Callback - velocity references 
*/
void cmdAngleCallback(const std_msgs::Float64::ConstPtr& cmd)
{
	setSteeringPosition(cmd->data);
	//ROS_INFO("rbcar_steering_controller::cmdPositionCountsCallback: value = %d",cmd->data);		
}


/*!	\fn int rbcar_steering_controller::start()
 * 	\brief Start Controller
*/
int start(){

	stop();
    
	// Setup motor drive	
	if(motorSteering->Setup()!= OK){
		ROS_ERROR("rbcar_steering_controller::rbcar_steering_controller: Error in Driver setup.");
	}
	// Setup can encoder
	if(encoderSteering->Setup()!= OK){
		ROS_ERROR("rbcar_steering_controller::rbcar_steering_controller: Error in encoder setup.");
	}

	// Start motor controller
	if(motorSteering->Start()!= OK){
		ROS_ERROR("rbcar_steering_controller::rbcar_steering_controller: Error in Driver start.");
		return -1;
	}
	// Start can encoder
	if(encoderSteering->Start()!= OK){
		ROS_ERROR("rbcar_steering_controller::rbcar_steering_controller: Error in encoder start.");
		return -1;
	}

	usleep(100000);
	
	ROS_INFO("rbcar_steering_controller::rbcar_steering_controller: controller started");
	freq_diag_.clear();
	running = true;
	
	return 0;
}

/*!	\fn rbcar_steering_controller::stop()
 * 	\brief Stop Controller
*/
int stop(){

    ROS_INFO("Stopping driver");
    if(running)
    {
    	motorSteering->Stop();
    	encoderSteering->Stop();
		sleep(1);
		running = false;
    }
    return 0;
}

/*!	\fn rbcar_steering_controller::connectTest()
 * 	\brief Test to connect to Motors
*/
void connectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
{
   // connection test
   // TBC
   status.summary(0, "Connected successfully.");
}

/*
 *\brief Checks the status of the controller. Diagnostics
 *
 */
void controllerDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if(!can_communication){
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "CAN communication error");
	}else{
		States state = motorSteering->GetState();
		
		if(state == READY_STATE){
			stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Controller ready");
		}else if(state == INIT_STATE){
			stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Controller initialing");
		}else{
			stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Controller on Failure or unknown state");
		}
	}
	
	stat.add("Communication Status", motorSteering->GetCommunicationStatusString()); 
	// add and addf are used to append key-value pairs.
    stat.add("Motor status", motorSteering->GetStatusString()); // Internal controller state
    // add status word string
	stat.add("Status Word", motorSteering->GetStatusWordString()); 
	
	
}

/*! \fn void controllerPublishing()
 * \brief Publish Motors Status Topics
 *
 */
void controllerPublishing()
{
	
	rbcar_steering_controller::SteeringControllerStatus status;
	
	// MOTOR STATUS
	status.motor_status.state = motorSteering->GetStateString();
	status.motor_status.status = motorSteering->GetStatusString();
	status.motor_status.statusword = motorSteering->GetStatusWordString();
	status.motor_status.driveflags = motorSteering->GetStatusRegisterFlagsString();

	// ENCODER STATUS	
	status.encoder_status.state = encoderSteering->GetStateString();
	status.encoder_status.communication_status = encoderSteering->GetCommunicationStatusString();
	
	pub_status_.publish(status);
	
}


/*! \fn void checkCommandSubscriber(diagnostic_updater::DiagnosticStatusWrapper &stat)
 *	\brief Checks that the robot is receiving at a correct frequency the command messages. Diagnostics
 *
 */
void checkCommandSubscriber(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");
	
	ros::Time current_time = ros::Time::now();

	double diff = (current_time - last_command_time).toSec();

	if(diff > 1.0){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Topic is not receiving commands");
	}else{
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");
	}
}
	

////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

/*!	\fn void rbcar_steering_controller::toggleMotorPower(unsigned char val)
 *	\brief Switches on/off the motor
*/
void toggleMotorPower(unsigned char val)
{
       /* switch(val) {
        case '1':       //Enable Motors
            driver->Start();
            ROS_INFO("rbcar_steering_controller::ToggleMotorPower: Motors enabled");
        break;
        case '0':       //Disable Motors
            driver->Stop();
            ROS_INFO("rbcar_steering_controller::ToggleMotorPower: Motors disabled");
        break;
        }*/
}

/*! \fn int readCANMessages()
 *	\brief Read messages from CAN bus
 *	\return -1 if ERROR
 *	\return 0 if OK
*/
int readCANMessages(){
	TPCANRdMsg tpcmsg_read;
	int len=100;
	int i=0;
	byte aux_id = 0;
	
	// Read and process up to 100 messages
	for (i=0;i<len;i++) {
        int iRet = canDev->Read(&tpcmsg_read);
		if(iRet == ERROR){
			ROS_ERROR("rbcar_steering_controller::ReadCANMessages: Error Reading messages from bus");
			can_communication = false;
			return -1;
		}else {
			if (iRet == OK) {  // Something has been read			              
				aux_id = tpcmsg_read.Msg.ID & 0xFF; 
			
				motorSteering->ProcessCANMessage(tpcmsg_read.Msg);
							
				encoderSteering->ProcessCANMessage(tpcmsg_read.Msg);
				
				can_communication = true;
			}
		}
	}
	return 0;
}


// Method readAndPublish()
int readAndPublish(){
	static double prevtime = 0;
	
	// Syncronize CAN bus
	motorSteering->Syncronize();
	
	if(readCANMessages() != 0){
		ROS_ERROR("RbcarSteeringNode::readAndPublish:Error reading CAN messages"); 
		return -1;
	}
	
	// MOTOR STATUS
	status.motor_status.state = motorSteering->GetStateString();
	status.motor_status.status = motorSteering->GetStatusString();
	status.motor_status.statusword = motorSteering->GetStatusWordString();
	status.motor_status.driveflags = motorSteering->GetStatusRegisterFlagsString();

	// ENCODER STATUS	
	status.encoder_status.state = encoderSteering->GetStateString();
	status.encoder_status.communication_status = encoderSteering->GetCommunicationStatusString();
	
	status.abs_encoder_counts = encoderSteering->GetEncoderCounts();
	status.motor_counts = motorSteering->GetMotorCounts();
	status.steering_position = encoderCountsToAngle(status.abs_encoder_counts);
	
	pub_status_.publish(status);

	freq_diag_.tick();
	return(0);
}

bool spin()
{
    ROS_INFO("rbcar_steering_controller::spin() - 1");
    ros::Rate r(desired_freq_);  // 50.0 

    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
		if (start() == 0){
	
			while(ros::ok() && node_handle_.ok()) {
				//ROS_INFO("while() 1"); 
				if(readAndPublish() < 0)
					ROS_INFO("Error readAndPublish"); 
					//break;
					self_test_.checkTest();
					diagnostic_.update();
					
					
					/*if (publish_status_counter > publish_status_freq) {
						controllerPublishing();
						publish_status_counter=0;
					}
					publish_status_counter++;*/
					ros::spinOnce();
					r.sleep();
			}
			ROS_INFO("END OF ros::ok() !!!");
		} else {
			// No need for diagnostic here since a broadcast occurs in start
			// when there is an error.
			usleep(1000000);
			self_test_.checkTest();
			ros::spinOnce();
		}
   }

   ROS_INFO("rbcar_steering_controller::spin(): Bye bye!");
   return true;
}

/*! \fn  bool setMotorStatus(rbcar_steering_controller::SetMotorStatus::Request &req, rbcar_steering_controller::SetMotorStatus::Response &res )
  * Sets the odometry of the robot
*/
bool setMotorStatus(rbcar_steering_controller::SetMotorStatus::Request &req, rbcar_steering_controller::SetMotorStatus::Response &res )
{
	
	res.ret = true;
	
	if(req.status == 0){
		ROS_INFO("rbcar_steering_controller::setMotorStatus: OPERATION_ENABLED");
		motorSteering->SetDesiredStatus(Dzcante020l080::OPERATION_ENABLED);
	}else if(req.status == 1){
		ROS_INFO("rbcar_steering_controller::setMotorStatus: QUICK_STOP");
		motorSteering->SetDesiredStatus(Dzcante020l080::QUICK_STOP);
	}else if(req.status == 2){
		ROS_INFO("rbcar_steering_controller::setMotorStatus: POWER_DISABLED(READY_TO_SWITCH_ON)");
		motorSteering->SetDesiredStatus(Dzcante020l080::READY_TO_SWITCH_ON);
	}else{
		ROS_INFO("rbcar_steering_controller::setMotorStatus: UNKWNOW status = %d", req.status);
		res.ret = false;
	}
	
	
	return true;
}

/*! \fn  int setSteeringParams()
  *  Sets the params for the steering control loop
*/
int setSteeringParams(){
	
	steering_config_.abs_counts_range = steering_config_.abs_encoder_right - steering_config_.abs_encoder_left;
	steering_config_.motor_counts_range = steering_config_.motor_encoder_right - steering_config_.motor_encoder_left;
	steering_config_.abs_limit_left =  steering_config_.abs_encoder_zero_position - steering_config_.abs_encoder_left;
	steering_config_.abs_limit_right = steering_config_.abs_encoder_zero_position - steering_config_.abs_encoder_right;
	steering_config_.abs_to_motor = steering_config_.motor_counts_range/steering_config_.abs_counts_range;
	steering_config_.a = steering_config_.steering_angle_resolution/(steering_config_.abs_encoder_zero_position-steering_config_.abs_encoder_left); 
	steering_config_.b = steering_config_.abs_encoder_zero_position;
		
}

/*! \fn  double encoderCountsToAngle(int counts)
  *  Converts absolute encoder counts into angle
  *  \return converted angle in radians
*/
double encoderCountsToAngle(int counts){

	if((counts>=(steering_config_.abs_encoder_zero_position-steering_config_.number_points_considered_zero))&&(counts<(steering_config_.abs_encoder_zero_position+steering_config_.number_points_considered_zero))) return 0;
	
	return ((-steering_config_.value_calibrate_a*counts)+steering_config_.value_calibrate_b);

}

/*! \fn  int setSteeringPosition(double angle)
  *  sets the steering position
  * \param angle as double, desired position in radians
  * \return 0 if OK, -1 if ERROR
*/
int setSteeringPosition(double angle){
    int desired_counts;	
	// Desired counts - current counts
	desired_counts=-((angle-steering_config_.value_calibrate_b)/steering_config_.value_calibrate_a);	
	if(angle>=steering_config_.extrem_angle_left)desired_counts=steering_config_.extrem_points_left;
	if(angle<=-steering_config_.extrem_angle_right) desired_counts=steering_config_.extrem_points_right;
	
	int diff_counts = desired_counts - status.abs_encoder_counts;
	int inc_motor_counts = diff_counts*steering_config_.abs_to_motor;
	int desired_motor_counts = status.motor_counts + inc_motor_counts;
	
	motorSteering->SetMotorPosition(desired_motor_counts);
}


}; // class RbcarSteeringNode

// MAIN
int main(int argc, char** argv)
{
        //rbcar_steering_controller::Battery battery;
	ros::init(argc, argv, "rbcar_steering_controller");
	
	ros::NodeHandle n;		
  	RbcarSteeringNode rsn(n);

    rsn.spin();

	return (0);
}
// EOF


