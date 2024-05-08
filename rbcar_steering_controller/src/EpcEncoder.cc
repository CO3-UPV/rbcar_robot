/*! \class EpcEncoder
 *  \file EpcEncoder.cc
 *	\author Robotnik Automation S.L.L
 *	\version 1.0
 *	\date 2010
 *  \brief Class to manage the state of the inputs and outputs of a module I/O
 *
 * (C) 2010 Robotnik Automation, SLL
 *  All rights reserved.
 */

#include <rbcar_steering_controller/EpcEncoder.h>
#include <math.h>


// Time
const unsigned int EpcEncoder::uiDelayTrans = 1000000;  // nanoseconds

/*! \fn void *AuxControlCommunicationThread(void *threadParam)
 * \param threadParam as void *, parameters of thread
 * Function executing in the thread
*/
void *AuxEpcEncoderControlCommunicationThread(void *threadParam){
    EpcEncoder *ComponentThread = (EpcEncoder *)threadParam;
	ComponentThread->ControlCommunicationThread();

	pthread_detach(pthread_self());
	return NULL;
}

/*! \fn void *AuxReadSDOMessagesThread(void *threadParam)
 * \param threadParam as void *, parameters of thread
 * Function executing in the thread
*/
void *AuxEpcEncoderReadSDOMessagesThread(void *threadParam){
    EpcEncoder *ComponentThread = (EpcEncoder *)threadParam;
	ComponentThread->ReadSDOMessagesThread();

	pthread_detach(pthread_self());
	return NULL;
}

/*! \fn EpcEncoder::EpcEncoder(byte CanId, ESDCan *can_device, double desired_hz)
 * Constructor
*/
EpcEncoder::EpcEncoder(byte can_id, PCan *can_device, double desired_hz):EpcMaCanOpen(can_device){
    // Change the name to the name of the derived class
    sComponentName.assign("EpcEncoder");
    CanId = can_id;
    threadData.dDesiredHz = desired_hz;     // attribute of Component class
    commStatus = (CommunicationStatus)UNKNOWN;
    encoder_counts = 0;


}

/*! \fn EpcEncoder::~EpcEncoder()
 * Destructor
*/
EpcEncoder::~EpcEncoder(){

}

/*! \fn ReturnValue EpcEncoder::Setup()
 * Configures and initializes the component
 * \return OK
 * \return INITIALIZED if the component is already intialized
 * \return ERROR
*/
ReturnValue EpcEncoder::Setup(){

	// Checks if has been initialized
	if(bInitialized){
		ROS_ERROR("%s::Setup: Already initialized",sComponentName.c_str());
		return INITIALIZED;
	}

	if(EpcMaCanOpen::Setup() == ERROR){
		ROS_ERROR("%s::Setup: Error in setup",sComponentName.c_str());
		return ERROR;
	}

	// Configure the component
	if(Configure() != OK){
		ROS_ERROR("%s::Setup: Error in Configure",sComponentName.c_str());
		return ERROR;
	}

	bInitialized = true;
	

	return OK;
}

/*! \fn ReturnValue EpcEncoder::Configure()
 * Configures devices and performance of the component
 * \return OK
 * \return ERROR
*/
ReturnValue EpcEncoder::Configure(){

	if(ResetCANCommunication() == ERROR) {
		ROS_ERROR("%s::Configure: Node %d: Error reseting CAN communication", sComponentName.c_str(), CanId);
		return ERROR;
	}
	Delay();
	if(PreoperationalCANCommunication() == ERROR) {
		ROS_ERROR("%s::Configure: Node %d: Error preoperational CAN communication", sComponentName.c_str(), CanId);
		return ERROR;
	}
	Delay();
	if(ConfigureCANMsgs() != OK)	{
		ROS_ERROR("%s: Configure: Node %d: Error configuring CAN msgs", sComponentName.c_str(), CanId);
		return ERROR;
	}
	Delay();
	if(Initialize() != OK ){
		ROS_ERROR("%s::Configure: Node %d: Error Initializing the drive", sComponentName.c_str(), CanId);
		return ERROR;
	}

	if(StartCANCommunication() == ERROR)	{
		ROS_ERROR("%s::Configure: Node %d: Error starting communication control state machine", sComponentName.c_str(), CanId);
		return ERROR;
	}

	return OK;
}

/*! \fn ReturnValue EpcEncoder::ConfigureCANMsgs()
 * Configures CAN TPDOs and RPDOs messages
 * \return OK
 * \return ERROR
*/
ReturnValue EpcEncoder::ConfigureCANMsgs(){

	// Configures Node guards timer
	/*if(ConfigureNodeGuardTimer()!= OK){
		ROS_ERROR("%s::ConfigureCANMsgs: Node %d: Error configuring Node guard timer", sComponentName.c_str(), CanId);
		return ERROR;
	}*/
	/*
	if(ConfigureTPDO(1, 10, true) != OK){
		ROS_ERROR("%s::ConfigureCANMsgs: Node %d: Error configuring TPDO1", sComponentName.c_str(), CanId);
		return ERROR;
	}else{
		//ROS_INFO("%s::ConfigureCANMsgs: Node %d: configuring TPDO1", sComponentName.c_str(), CanId);
		Delay();
	}*/

	return OK;
}

/*! \fn CommunicationStatus EpcEncoder::GetCommunicationStatus(){
 * 	\brief function to get the communication status of the device
 * 	\return Comm status of the of the component
*/
CommunicationStatus EpcEncoder::GetCommunicationStatus(){
	return commStatus;
}

/*! \fn char *EpcEncoder::GetCommunicationStatusString()
 * Get communication status like a string
*/
char *EpcEncoder::GetCommunicationStatusString(){
	switch(commStatus){
		case PRE_OPERATIONAL:
			return (char *)"PRE_OPERATIONAL";
		break;
		case OPERATIONAL:
			return (char *)"OPERATIONAL    ";
		break;
		case STOPPED:
			return (char *)"STOPPED        ";
		break;
		case FAULT:
			return (char *)"FAULT          ";
		break;
		default:
			return (char *)"UNKNOWN        ";
		break;
	}
}

/*!	\fn void EpcEncoder::SwitchToCommunicationStatus(CommunicationStatus new_status)
 * 	\brief Change the value of the communication status
 * 	\param new_status as CommunicationStatus, new communication status value
*/
void EpcEncoder::SwitchToCommunicationStatus(CommunicationStatus new_status){

	if(new_status!= commStatus){ //Only if it's different
		commStatus = new_status;
		switch(new_status){
			case UNKNOWN:
				ROS_INFO("%s::ChangeCommunicationStatus: Node %d: comm status changed to UNKNOWN", sComponentName.c_str(), CanId);
				break;
			case STOPPED:
				ROS_INFO("%s::ChangeCommunicationStatus: Node %d: comm status changed to STOPPED", sComponentName.c_str(), CanId);
				break;
			case PRE_OPERATIONAL:
				ROS_INFO("%s::ChangeCommunicationStatus: Node %d: comm status changed to PRE_OPERATIONAL", sComponentName.c_str(), CanId);
				break;
			case OPERATIONAL:
				ROS_INFO("%s::ChangeCommunicationStatus: Node %d: comm status changed to OPERATIONAL", sComponentName.c_str(), CanId);
				break;
			case FAULT:
				ROS_INFO("%s::ChangeCommunicationStatus: Node %d: comm status changed to FAULT", sComponentName.c_str(), CanId);
				break;
			default:
				ROS_INFO("%s::ChangeCommunicationStatus: Node %d: comm status changed to ???????\n", sComponentName.c_str(), CanId);
			break;
		}
	}
}

/*!	\fn void EpcEncoder::InitState()
 *	\brief Actions performed on initial state
*/
void EpcEncoder::InitState(){
	// State transitions
	if(commStatus == (CommunicationStatus)FAULT) {
		//ROS_INFO("%s::InitState: Node %d: Fault To Emergency", sComponentName.c_str(), CanId);
		SwitchToState(EMERGENCY_STATE);		
	} else if(commStatus != OPERATIONAL) {
		//ROS_INFO("%s::InitState: Node %d: Start Communication", sComponentName.c_str(), CanId);
		StartCANCommunication();
	} else {
		//ROS_INFO("%s::InitState: Node %d: To Ready State", sComponentName.c_str(), CanId);
		SwitchToState(READY_STATE);
	}

	// Inits nodeguard reply timer
	clock_gettime(threadData.pthreadPar.clock, &tNodeGuardReply);
	tsnorm(&tNodeGuardReply);
}

/*!	\fn void EpcEncoder::ReadyState()
 *	\brief Actions performed on ready state
 *         EpcEncoder will be ready while the communication with the device be OK
*/
void EpcEncoder::ReadyState(){
	struct timespec tNow;
	long diff = 0;

        // State transitions
	if(commStatus == (CommunicationStatus)FAULT)
		SwitchToState(EMERGENCY_STATE);
	else if(commStatus != OPERATIONAL)
		SwitchToState(INIT_STATE);

}


/*!	\fn void EpcEncoder::RestartState()
 *	\brief Actions performed on restart state
 *         EpcEncoder will be completely restarted
*/
void EpcEncoder::RestartState(){
	 
}

/*!	\fn void EpcEncoder::EmergencyState()
 *	\brief Actions performed on emergency state
*/
void EpcEncoder::EmergencyState(){
    static unsigned int count_cycles = 0;
    static unsigned int recover_cycles = 4*(unsigned int)threadData.dDesiredHz;
    // State transitions
    if(commStatus != (CommunicationStatus)FAULT) 
        SwitchToState(INIT_STATE);       
    else if(count_cycles == recover_cycles){    // Tries to recover the communication
        count_cycles = 0;
        ResetCANCommunication();
    } else
        count_cycles++;

}

/*!	\fn void EpcEncoder::AllState()
 *	\brief Actions performed on all states
*/
void EpcEncoder::AllState(){

}

/*!	\fn void EpcEncoder::ProcessCANMessage(TPCANMsg msg)
 * \brief Process received CAN messages
 * \param msg as a TPCANMsg, the message to process
*/
void EpcEncoder::ProcessCANMessage(TPCANMsg msg){
	unsigned int NMT_id = 0x700 + CanId;	// COB-id for NMT message response from drive
	unsigned int SDO_Response_id = 0x580 + CanId;
	unsigned int TPDO1_id = TPDO1 + CanId, TPDO2_id = TPDO2 + CanId;
	unsigned int EMERGENCY_msg = 0x80 + CanId;
	
	//ROS_INFO("EpcEncoder::ProcessCANMessage: CanID:%d msg.DATA[0]=%d",CanId,msg.DATA[0]);

	if(msg.ID == NMT_id) { //node Guard message received
		// Saves the current time
		//ROS_INFO("EpcEncoder::ProcessCANMessage: CanID:%d msg.DATA[0]=%d",CanId,msg.DATA[0]);
		clock_gettime(threadData.pthreadPar.clock, &tNodeGuardReply);
		tsnorm(&tNodeGuardReply);
		// DEBUG NODE_GUARD
		// if (CanId==7)  ROS_INFO("ProcessCANMsg 7 ");

		if((msg.DATA[0]== 0x04) || (msg.DATA[0] == 0x84)){ // Communication in STOPPED state
			SwitchToCommunicationStatus(STOPPED);
			return;
		}
		if((msg.DATA[0]== 0x05) || (msg.DATA[0] == 0x85)){ // Communication in OPERATIONAL state
			SwitchToCommunicationStatus(OPERATIONAL);
			return;
		}
		if((msg.DATA[0]== 0x7F) || (msg.DATA[0] == 0xFF)){ // Communication in PRE_OPERATIONAL state
			SwitchToCommunicationStatus(PRE_OPERATIONAL);
			return;
		}
	}
	//
	// PDO (Process Data Object) Messages
	// 
	if(msg.ID == TPDO1_id){
		ProcessTPDO1Msg(msg);
		return;
	}
	//
	// 
	else if(msg.ID == TPDO2_id){
		ProcessTPDO2Msg(msg);
		//ROS_INFO("TPDO21");
		return;
	}
	
	//
	// SDO (Service Data Objects) Messages
	else if(msg.ID == SDO_Response_id){
		//ROS_INFO("SDO: %x%x ",msg.DATA[2],msg.DATA[1]);
	    
	}//else
	//	printf("Unknown message = %x vs %x\n", msg.id, TPDO22_id);
	
	else if(msg.ID == EMERGENCY_msg){
		
	}
	
}

/*!	\fn ReturnValue EpcEncoder::StartCANCommunication()
 * 	\brief Sends a message to put CAN communication in Operational state
 * 	\return OK
 *  \return ERROR
*/
ReturnValue EpcEncoder::StartCANCommunication(){
	//ROS_INFO("%s::StartCANCommunication: starting com for node %d", sComponentName.c_str(), CanId);
    	return Send(CanId, START_COMM_MSG, WRITE, 0);
}

/*!	\fn ReturnValue EpcEncoder::ResetCANCommunication()
 * 	\brief Sends a message to reset CAN communication
 * 	\return OK
 *  \return ERROR
*/
ReturnValue EpcEncoder::ResetCANCommunication(){
	ROS_INFO("%s::ResetCANCommunication: Reseting com for node %d", sComponentName.c_str(), CanId);
        return Send(CanId, RESET_COMM_MSG, WRITE, 0);
}

/*!	\fn ReturnValue EpcEncoder::StopCANCommunication()
 * 	\brief Sends a message to stop CAN communication
 * 	\return OK
 *  \return ERROR
*/
ReturnValue EpcEncoder::StopCANCommunication(){
    return Send(CanId, STOP_COMM_MSG, WRITE, 0);
}

/*!	\fn ReturnValue EpcEncoder::PreoperationalCANCommunication()
 * 	\brief Sends a message to put the CAN comm into preoperational state
 * 	\return OK
 *  \return ERROR
*/
ReturnValue EpcEncoder::PreoperationalCANCommunication(){
    return Send(CanId, PREOPERATIONAL_COMM_MSG, WRITE, 0);
}

/*!	\fn ReturnValue EpcEncoder::Syncronize()
 * 	\brief Function to send SYNC message by the CAN network
 * 	\return OK
 *  \return ERROR
*/
ReturnValue EpcEncoder::Syncronize(){
	return Send(CanId, SYNC_MSG, WRITE, 0);
}


/*! \fn void EpcEncoder::ProcessTPDO1Msg(TPCANMsg msg)
 *  \brief Process received CAN TPDO1 messages
 *  \param msg as a TPCANMsg, the message to process
*/
void EpcEncoder::ProcessTPDO1Msg(TPCANMsg msg){
	int32_t aux = 0;
    aux =  (int) msg.DATA[1] * 256 + (int) msg.DATA[0];

    encoder_counts = aux;
   // ROS_INFO("%s::ProcessTPDO1Msg: encoder_counts = %d", sComponentName.c_str(), encoder_counts);
}

/*!	\fn	 void EpcEncoder::ProcessTPDO2Msg(TPCANMsg msg)
 * 	\brief Process received CAN TPDO2 messages 
 * 	\param msg as a TPCANMsg, the message to process
*/
void EpcEncoder::ProcessTPDO2Msg(TPCANMsg msg){
	int32_t aux = 0;
    aux =  (int) msg.DATA[1] * 256 + (int) msg.DATA[0];

    encoder_counts = aux;
  //  ROS_INFO("%s::ProcessTPDO2Msg: encoder_counts = %d", sComponentName.c_str(), encoder_counts);
}


/*!	\fn	 void EpcEncoder::ProcessEmergencyMsg(TPCANMsg msg)
 * 	\brief Process received CAN EMCY messages (0x80 + CAN_ID)
 * 	\param msg as a TPCANMsg, the message to process
*/
void EpcEncoder::ProcessEmergencyMsg(TPCANMsg msg){
	int32_t error_code = 0, error_reg = 0, info1 = 0, info2 = 0;
	
    error_code = (int) msg.DATA[1] * 256 + (int) msg.DATA[0];
    error_reg = (int) msg.DATA[2];
    info1 = (int) msg.DATA[3];
    info2 = (int) msg.DATA[4];
    
	ROS_ERROR("%s::ProcessEmergencyMsg: Error code = %x, Error register = %x, Info 1 = %x, Info 2 = %x", sComponentName.c_str(), error_code, error_reg, info1, info2);
}




/*! \fn ReturnValue EpcEncoder::Start()
 * Starts the control thread of the component and its subcomponents
 * \return OK
 * \return RUNNING if it's already running
 * \return NOT_INITIALIZED if the component is not initialized
*/
ReturnValue EpcEncoder::Start(){
    ReturnValue ret;
    int priority_aux = 25;
    double frec_aux = 5.0;

    // Starts main thread
    ret = Component::Start();

    if(ret != OK)
        return ret;

    // Creates auxiliar thread to control the communication status with the drive
    ret = Component::CreateTask(priority_aux, frec_aux, AuxEpcEncoderControlCommunicationThread);

    if(ret == ERROR){
        bRunning = false;
        ROS_ERROR("%s::Start: Error launching auxiliary thread", sComponentName.c_str());
		return ERROR;
    }

	// Creates auxiliar thread to read SDO messages from the module
    /*ret = Component::CreateTask(priority_aux, frec_aux, AuxEpcEncoderReadSDOMessagesThread);

    if(ret == ERROR){
        bRunning = false;
        ROS_ERROR("%s::Start: Error launching auxiliary thread 2", sComponentName.c_str());
	return ERROR;
    }*/

    return OK;
}

/*! \fn void EpcEncoder::ControlCommunicationThread()
 	* Thread to control the communication with the driver
 	* All auxiliary threads should have this structure
*/
void EpcEncoder::ControlCommunicationThread(){
	struct sched_param schedp;
	int policy = threadData.pthreadPar.prio? SCHED_FIFO : SCHED_OTHER;
	struct timespec now, next, interval;
	//long diff;
	unsigned long sampling_period_us;
	thread_data current_data;
	struct timespec time;
	long time_diff = 0;
	bool bPrintError = false;

	// LLamada obligatoria para obtener los datos del thread
	if(GetThreadData(&current_data) == ERROR ){
	ROS_ERROR("%s::ControlCommunicationThread: Error getting thread data", sComponentName.c_str());
	return;
	}
	ROS_INFO("%s::ControlCommunicationThread: Executing thread (%d, %lf)", sComponentName.c_str(), 
				current_data.pthreadPar.prio, current_data.dDesiredHz );

	// Robotnik thread priority and timing management
	memset(&schedp, 0, sizeof(schedp));
	schedp.sched_priority = current_data.pthreadPar.prio;
	sched_setscheduler(0, policy, &schedp);
	clock_gettime(current_data.pthreadPar.clock, &now);
 	next = now;
	next.tv_sec++;  // start in next second?
	sampling_period_us =  (long unsigned int) (1.0 / current_data.dDesiredHz * 1000000.0);
	interval.tv_sec = sampling_period_us / USEC_PER_SEC;  // interval parameter in uS
	interval.tv_nsec = (sampling_period_us % USEC_PER_SEC)*1000;

	ROS_INFO("%s::ControlCommunicationThread: Starting the thread", sComponentName.c_str());

	while(bRunning) { // Executes state machine code while bRunning flag is active
		clock_nanosleep(threadData.pthreadPar.clock, TIMER_ABSTIME, &next, NULL);
		clock_gettime(threadData.pthreadPar.clock, &now);
		next.tv_sec += interval.tv_sec;
		next.tv_nsec += interval.tv_nsec;
		tsnorm(&next);
        //////////////// ACTIONS  //////////////////////////
        // Sending node guard message
        Send(CanId, NODEGUARD_MSG, WRITE, 0);
		
        // Gets the current time to compare
        clock_gettime(threadData.pthreadPar.clock, &time);
        tsnorm(&time);
        time_diff = time.tv_sec - tNodeGuardReply.tv_sec;

		// Bug - 2013/06/26 - Fails during homing 
        if(time_diff > NMT_TIMEOUT){
			if(!bPrintError){
				ROS_ERROR( "%s::ControlCommunicationThread: Node %d: error in communication with the device", sComponentName.c_str(), CanId);
				bPrintError = true;
				}
            SwitchToCommunicationStatus((CommunicationStatus)FAULT);
        }else
			bPrintError = false;

        ////////////////////////////////////////////////////
	}
	ROS_INFO( "%s::ControlCommunicationThread: End", sComponentName.c_str());
};

/*!	\fn void EpcEncoder::Delay()
 * 	\brief Sleeps for a moment
*/
void EpcEncoder::Delay(){
	struct timespec wait;

	clock_gettime(threadData.pthreadPar.clock, &wait);
	wait.tv_nsec+= uiDelayTrans;
	clock_nanosleep(threadData.pthreadPar.clock, TIMER_ABSTIME, &wait, NULL);
}

/*!	\fn void EpcEncoder::Delay(unsigned int value)
 * 	\brief Sleeps for a moment
*/
void EpcEncoder::Delay(unsigned int value){
	struct timespec wait;

	clock_gettime(threadData.pthreadPar.clock, &wait);
	wait.tv_nsec+= value * uiDelayTrans;
	clock_nanosleep(threadData.pthreadPar.clock, TIMER_ABSTIME, &wait, NULL);
}


/*!	\fn ReturnValue EpcEncoder::Initialize()
 * 	\brief Initializes the status of the drive
 * 	\return OK
 * 	\return ERROR
*/
ReturnValue EpcEncoder::Initialize(){

	return OK;
}


/*! \fn void EpcEncoder::ReadSDOMessagesThread()
 	* Reads SDO messages from the device
 	* All auxiliary threads should have this structure
*/
void EpcEncoder::ReadSDOMessagesThread(){
    struct sched_param schedp;
    int policy = threadData.pthreadPar.prio? SCHED_FIFO : SCHED_OTHER;
    struct timespec now, next, interval;
    unsigned long sampling_period_us;
    thread_data current_data;
    int drive_status_number = 1;	// number of Drive Status object to read (1-6)
    vector<double> vCurrent;        // Vector to store consumed current
    int index = 0;                  // Index of the latest value introduced in the previous vector
    double aux = 0.0;
    vector<double> vBattery;        // Vector to store the latest x values read
    int index_battery = 0;                  // Indice of the latest introduced values
    // Mandatory call to get the thread data
    if(GetThreadData(&current_data) == ERROR ){
        ROS_ERROR("IOModule::ReadSDOMessagesThread: Error getting thread data");
        return;
    }
    //ROS_INFO "%s::ReadSDOMessagesThread: Executing thread (%d, %lf)", sComponentName.c_str(), current_data.pthreadPar.prio, current_data.dDesiredHz );
	// Robotnik thread priority and timing management
	memset(&schedp, 0, sizeof(schedp));
	schedp.sched_priority = current_data.pthreadPar.prio;
	sched_setscheduler(0, policy, &schedp);
	clock_gettime(current_data.pthreadPar.clock, &now);
 	next = now;
	next.tv_sec++;  // start in next second?
	sampling_period_us =  (long unsigned int) (1.0 / current_data.dDesiredHz * 1000000.0);
	interval.tv_sec = sampling_period_us / USEC_PER_SEC;  // interval parameter in uS
	interval.tv_nsec = (sampling_period_us % USEC_PER_SEC)*1000;

	//ROS_INFO ( "%s::ReadSDOMessagesThread: Starting the thread", sComponentName.c_str());

	while(bRunning) { // Executes state machine code while bRunning flag is active
		clock_nanosleep(threadData.pthreadPar.clock, TIMER_ABSTIME, &next, NULL);
		clock_gettime(threadData.pthreadPar.clock, &now);
		next.tv_sec += interval.tv_sec;
		next.tv_nsec += interval.tv_nsec;
		tsnorm(&next);
        //////////////// ACTIONS  //////////////////////////
		//ROS_ERROR( "%s::ControlCommunicationThread: Node %d: Sending node guard", sComponentName.c_str(), CanId);
        if((commStatus != (CommunicationStatus)FAULT)){
                        // Reads Drive status		

		}
	}

	//ROS_INFO ( "%s::ReadSDOMessagesThread: End", sComponentName.c_str());
};





/*! \fn byte EpcEncoder::GetCanId()
*   \brief Returns the CAN id
*/
byte EpcEncoder::GetCanId(){
    return CanId;
};


/*! \fn int EpcEncoder::GetEncoderCounts()
*   \brief Gets the current counts the encoder
*/
int EpcEncoder::GetEncoderCounts(){
	return encoder_counts;
}




