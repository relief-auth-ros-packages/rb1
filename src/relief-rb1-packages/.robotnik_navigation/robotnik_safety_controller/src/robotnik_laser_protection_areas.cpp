/*! \class LaserProtectionArea
 *  \file LaserProtectionArea_template.cpp
 *	\author Robotnik Automation S.L.L
 *	\version 0.1.0
 *	\date 2015
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


#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <vector>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <robotnik_msgs/State.h>
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/set_digital_output.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

//! Size of string for logging
#define DEFAULT_THREAD_DESIRED_HZ	10.0
#define TOPIC_WATCHDOG				2.0	// wathdog reading topics in secs
#define DEFAULT_HYSTERESIS_0		0.05
#define DEFAULT_VELOCITY_LIMIT_0	0.2
#define DEFAULT_OUTPUT_LEVEL_0		1
#define DEFAULT_HYSTERESIS_1		0.3
#define DEFAULT_VELOCITY_LIMIT_1	0.4
#define DEFAULT_OUTPUT_LEVEL_1		2
#define DEFAULT_OUTPUT_LOGIC		true	// Inverts the value of the output activation

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


//! Class LaserProtectionArea
class LaserProtectionArea{
	protected:
		//! Controls if has been initialized succesfully
		bool initialized, ros_initialized;
		//! Controls the execution of the LaserProtectionArea's thread
		bool running;
		
		//! State of the LaserProtectionArea
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
		ros::Publisher state_publisher_;
		// Examples:
		ros::Subscriber odom_sub_; // topic subscriber
		ros::Subscriber cmd_sub_; // topic subscriber
		ros::Subscriber io_sub_; // topic subscriber
		// ros::ServiceServer service_server_; // service server
		ros::ServiceClient io_service_client_; // service client
		
		//! General status diagnostic updater
		diagnostic_updater::Updater *diagnostic_;
		
		ros::Time last_odom_time;
		ros::Time last_cmd_time;
		ros::Time last_io_time;
		string io_service_name_;
		nav_msgs::Odometry odom_msg_;
		geometry_msgs::Twist cmd_msg_;
		robotnik_msgs::inputs_outputs io_msg_;
		
		/*
		 * RIGHT NOW ONLY WORKS FOR ONE RANGE AND ONE OUTPUT!!!
		 *
		 * */
		//! value used to avoid changing the area when the velocity goes down the limit
		double hysteresis_level_0_;
		//! velocity level to change the safety area
		double velocity_level_0_; 
		//! digital output to change the area config for level 1
		int output_level_0_; 
		//! True or False. Logic to invert or not the output value
		bool output_logic_0_;

		//! value used to avoid changing the area when the velocity goes down the limit
		double hysteresis_level_1_;
		//! velocity level to change the safety area
		double velocity_level_1_; 
		//! digital output to change the area config for level 1
		int output_level_1_; 
		//! True or False. Logic to invert or not the output value
		bool output_logic_1_;
		
		int current_area;
		
	public:
		//! Public constructor
		LaserProtectionArea(ros::NodeHandle h);
		//! Public destructor
		~LaserProtectionArea();
		
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
		//! Returns the general state of the LaserProtectionArea
		int getState();
		//! Returns the general state of the LaserProtectionArea as string
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
		//!	All of the LaserProtectionArea component state machine code can be found here.
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
		
		// Examples
		// void topicCallback(const std_msgs::StringConstPtr& message); // Callback for a subscriptor
		// bool serviceServerCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); // Callback for a service server
		
		//! Diagnostic updater callback
		void diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat);
		void odomCallback(const nav_msgs::OdometryConstPtr& message);
		void cmdCallback(const geometry_msgs::TwistConstPtr& message);
		void ioCallback(const robotnik_msgs::inputs_outputsConstPtr& message);
		
};


/*! \fn LaserProtectionArea::LaserProtectionArea()
 *  \brief Constructor by default
 *	\param hz as double, sets the desired frequency of the controlthread
 *	\param h as ros::NodeHandle, ROS node handle
*/
LaserProtectionArea::LaserProtectionArea(ros::NodeHandle h):nh_(h), pnh_("~"){
	// Set main flags to false
	ros_initialized = initialized = running = false;
	// reads params from server
	rosReadParams();
	
	if(desired_freq_ <= 0.0)
		desired_freq_ = DEFAULT_THREAD_DESIRED_HZ;
		
	state = robotnik_msgs::State::INIT_STATE;
	// Realizar para cada una de las clases derivadas
	component_name.assign("LaserProtectionArea");
	last_odom_time = last_cmd_time = last_io_time = ros::Time(0);
	current_area = 0;
	
}

/*! \fn LaserProtectionArea::~LaserProtectionArea()
 * Destructor by default
*/
LaserProtectionArea::~LaserProtectionArea(){
	
}

/*! \fn int LaserProtectionArea::setup()
 * Configures and initializes the component
 * \return OK
 * \return INITIALIZED if the component is already intialized
 * \return ERROR
*/
int LaserProtectionArea::setup(){
	// Checks if has been initialized
	if(initialized){
		ROS_INFO("%s::Setup: Already initialized",component_name.c_str());
		
		return INITIALIZED;
	}
	
	//
	///////////////////////////////////////////////////
	// Setups the component or another subcomponents if it's necessary //
	///////////////////////////////////////////////////
	

	initialized = true;

	return OK;
}

/*! \fn int LaserProtectionArea::shutDown()
 * Closes and frees the reserved resources
 * \return OK
 * \return ERROR if fails when closes the devices
 * \return RUNNING if the component is running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int LaserProtectionArea::shutdown(){
	
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


/*! \fn int LaserProtectionArea::start()
 * Starts the control thread of the component and its subcomponents
 * \return OK
 * \return RUNNING if it's already running
 * \return NOT_INITIALIZED if the component is not initialized
*/
int LaserProtectionArea::start(){
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

/*! \fn int LaserProtectionArea::stop()
 * Stops the control thread of the Motors
 * \return OK
 * \return ERROR if it can't be stopped
 * \return THREAD_NOT_RUNNING if the thread is not running
*/
int LaserProtectionArea::stop(){
	
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

/*!	\fn void LaserProtectionArea::controlLoop()
 *	\brief All core component functionality is contained in this thread.
*/
void LaserProtectionArea::controlLoop(){
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

/*!	\fn void LaserProtectionArea::initState()
 *	\brief Actions performed on initial 
 * 	Setups the component
*/
void LaserProtectionArea::initState(){
	// If component setup is successful goes to STANDBY (or READY) state
	if(setup() != ERROR){
		switchToState(robotnik_msgs::State::STANDBY_STATE);
	}
}

/*!	\fn void LaserProtectionArea::shutdownState()
 *	\brief Actions performed on Shutdown state
*/
void LaserProtectionArea::shutdownState(){
	
	if(shutdown() == OK){
		switchToState(robotnik_msgs::State::INIT_STATE);
	}
}

/*!	\fn void LaserProtectionArea::standbyState()
 *	\brief Actions performed on Standby state
*/
void LaserProtectionArea::standbyState(){
	ros::Time t_now = ros::Time::now();
	bool topics_ok = true;
	if((t_now - last_odom_time).toSec() > TOPIC_WATCHDOG){
		topics_ok = false;
		ROS_WARN_STREAM_THROTTLE_NAMED(2, component_name, component_name << "::standbyState: odom not received");
	} 
	if((t_now - last_io_time).toSec() > TOPIC_WATCHDOG){
		topics_ok = false;
		ROS_WARN_STREAM_THROTTLE_NAMED(2, component_name, component_name << "::standbyState: io not received");
	} 
	
	if(topics_ok)
		switchToState(robotnik_msgs::State::READY_STATE);
}

/*!	\fn void LaserProtectionArea::readyState()
 *	\brief Actions performed on ready state
*/
void LaserProtectionArea::readyState(){
	ros::Time t_now = ros::Time::now();
	bool topics_ok = true;
	bool check_cmd = false;

	if((t_now - last_odom_time).toSec() > TOPIC_WATCHDOG){
		topics_ok = false;
		ROS_WARN_STREAM_THROTTLE_NAMED(2, component_name, component_name << "::readyState: odom not received");
	} 
	if((t_now - last_io_time).toSec() > TOPIC_WATCHDOG){
		topics_ok = false;
		ROS_WARN_STREAM_THROTTLE_NAMED(2, component_name, component_name << "::readyState: io not received");
	} 
	
	if((t_now - last_cmd_time).toSec() > TOPIC_WATCHDOG){
		check_cmd = false;
	} 
	
	if(not topics_ok){
		switchToState(robotnik_msgs::State::STANDBY_STATE);
		return;
	}
	
	double current_linear_vel = 0.0;
	current_linear_vel = sqrt(pow(odom_msg_.twist.twist.linear.x,2) + pow(odom_msg_.twist.twist.linear.y,2));
	double current_cmd_linear_vel = 0.0;
	current_cmd_linear_vel = sqrt(pow(cmd_msg_.linear.x,2) + pow(cmd_msg_.linear.y,2));
	
	switch(current_area){
		case 0:
			if(current_linear_vel > velocity_level_1_ or (check_cmd and current_cmd_linear_vel > velocity_level_1_)){
				current_area = 2;
				ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area " << current_area << ". Vel = " << current_linear_vel);
			}
			else if(current_linear_vel > velocity_level_0_ or (check_cmd and current_cmd_linear_vel > velocity_level_0_)){
				current_area = 1;
				ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area " << current_area << ". Vel = " << current_linear_vel);

			}else{
				bool must_write_level_0 = false;
				robotnik_msgs::set_digital_output write_do_level_0;
				if(io_msg_.digital_outputs[output_level_0_ - 1] != not output_logic_0_){
					must_write_level_0 = true;
					write_do_level_0.request.output = output_level_0_;
					write_do_level_0.request.value = not output_logic_0_;
				}

				bool must_write_level_1 = false;
				robotnik_msgs::set_digital_output write_do_level_1;
				if(io_msg_.digital_outputs[output_level_1_ - 1] != not output_logic_1_){
					must_write_level_1 = true;
					write_do_level_1.request.output = output_level_1_;
					write_do_level_1.request.value = not output_logic_1_;
				}

				if (must_write_level_0) {	
					io_service_client_.call( write_do_level_0 );
					ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: area " << current_area << " - setting output " << (int)write_do_level_0.request.output << " to " << (int)write_do_level_0.request.value);
				}
				if (must_write_level_1) {	
					io_service_client_.call( write_do_level_1 );
					ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: area " << current_area << " - setting output " << (int)write_do_level_1.request.output << " to " << (int)write_do_level_1.request.value);
				}

				if (must_write_level_0 or must_write_level_1)
					sleep(1);
			}		
			break;
		
		case 1:
			if(current_linear_vel > velocity_level_1_ or (check_cmd and current_cmd_linear_vel > velocity_level_1_)){
				current_area = 2;
				ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area " << current_area << ". Vel = " << current_linear_vel);

			}else if (current_linear_vel < (velocity_level_0_ - hysteresis_level_0_) or (check_cmd and current_cmd_linear_vel < (velocity_level_0_ - hysteresis_level_0_))) {
				current_area = 0;
				ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area " << current_area << ". Vel = " << current_linear_vel);
			}
			else {
				bool must_write_level_0 = false;
				robotnik_msgs::set_digital_output write_do_level_0;
				if(io_msg_.digital_outputs[output_level_0_ - 1] != output_logic_0_){
					must_write_level_0 = true;
					write_do_level_0.request.output = output_level_0_;
					write_do_level_0.request.value = output_logic_0_;
				}
				bool must_write_level_1 = false;
				robotnik_msgs::set_digital_output write_do_level_1;
				if(io_msg_.digital_outputs[output_level_1_ - 1] != not output_logic_1_){
					must_write_level_1 = true;
					write_do_level_1.request.output = output_level_1_;
					write_do_level_1.request.value = not output_logic_1_;
				}
				if (must_write_level_0) {
					io_service_client_.call( write_do_level_0 );
					ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: area " << current_area << " - setting output " << (int)write_do_level_0.request.output << " to " << (int)write_do_level_0.request.value);
				}
				if (must_write_level_1) {
					io_service_client_.call( write_do_level_1 );
					ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: area " << current_area << " - setting output " << (int)write_do_level_1.request.output << " to " << (int)write_do_level_1.request.value);
				}
				if (must_write_level_0 or must_write_level_1)
					sleep(1);
			}		
			break;

		case 2:
			if (current_linear_vel < (velocity_level_0_ - hysteresis_level_0_) or (check_cmd and current_cmd_linear_vel < (velocity_level_0_ - hysteresis_level_0_))) {
				current_area = 0;
				ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area " << current_area << ". Vel = " << current_linear_vel);
			}
			else if(current_linear_vel < (velocity_level_1_ - hysteresis_level_1_)  or (check_cmd and current_cmd_linear_vel < (velocity_level_1_ - hysteresis_level_1_))){
				current_area = 1;
				ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area " << current_area << ". Vel = " << current_linear_vel);
			}else{
				bool must_write_level_0 = false;
				robotnik_msgs::set_digital_output write_do_level_0;
				if(io_msg_.digital_outputs[output_level_0_ - 1] != output_logic_0_){
					must_write_level_0 = true;
					write_do_level_0.request.output = output_level_0_;
					write_do_level_0.request.value = output_logic_0_;
				}
				bool must_write_level_1 = false;
				robotnik_msgs::set_digital_output write_do_level_1;
				if(io_msg_.digital_outputs[output_level_1_ - 1] != output_logic_1_){
					must_write_level_1 = true;
					write_do_level_1.request.output = output_level_1_;
					write_do_level_1.request.value = output_logic_1_;
				}
				if (must_write_level_0) {
					io_service_client_.call( write_do_level_0 );
					ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: area " << current_area << " - setting output " << (int)write_do_level_0.request.output << " to " << (int)write_do_level_0.request.value);
				}
				if (must_write_level_1) {
					io_service_client_.call( write_do_level_1 );
					ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: area " << current_area << " - setting output " << (int)write_do_level_1.request.output << " to " << (int)write_do_level_1.request.value);
				}
				if (must_write_level_0 or must_write_level_1)
					sleep(1);
			}

			break;
	}
	
	
	/*pnh_.param("hysteresis", hysteresis_, DEFAULT_HYSTERESIS);
	pnh_.param("velocity_level_1", velocity_level_1_, DEFAULT_VELOCITY_LIMIT_1);
	pnh_.param("output_level_1", output_level_1_, DEFAULT_OUTPUT_LEVEL_1);
	pnh_.param("output_logic_1", output_logic_1_, DEFAULT_OUTPUT_LOGIC);*/
}

/*!	\fn void LaserProtectionArea::EmergencyState()
 *	\brief Actions performed on emergency state
*/
void LaserProtectionArea::emergencyState(){

}

/*!	\fn void LaserProtectionArea::FailureState()
 *	\brief Actions performed on failure state
*/
void LaserProtectionArea::failureState(){

}

/*!	\fn void LaserProtectionArea::AllState()
 *	\brief Actions performed on all states
*/
void LaserProtectionArea::allState(){
	
	diagnostic_->update();
	
	rosPublish();
}

/*!	\fn double LaserProtectionArea::getUpdateRate()
 * 	\brief Gets current update rate of the thread
 * 	\return real frequency of the thread
*/
double LaserProtectionArea::getUpdateRate(){
	return desired_freq_;
}

/*!	\fn int LaserProtectionArea::getState()
 * 	\brief returns the state of the component
*/
int LaserProtectionArea::getState(){
	return state;
}

/*!	\fn char *LaserProtectionArea::getStateString()
 *	\brief Gets the state of the component as string
*/
char *LaserProtectionArea::getStateString(){
	return getStateString(state);
}

/*!	\fn char *LaserProtectionArea::getStateString(int state)
 *	\brief Gets the state as a string
*/
char *LaserProtectionArea::getStateString(int state){
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


/*!	\fn void LaserProtectionArea::switchToState(int new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void LaserProtectionArea::switchToState(int new_state){
	
	if(new_state == state)
		return;

	// saves the previous state
	previous_state = state;
	ROS_INFO("%s::SwitchToState: %s -> %s", component_name.c_str(), getStateString(state), getStateString(new_state));	
	state = new_state;

}

/*!	\fn void LaserProtectionArea::rosSetup()
 * 	\brief Setups all ROS' stuff
*/
int LaserProtectionArea::rosSetup(){
	
	// Checks if has been initialized
	if(ros_initialized){
		ROS_INFO("%s::rosSetup: Already initialized",component_name.c_str());
		
		return INITIALIZED;
	}
	
	// Publishers
	state_publisher_ = pnh_.advertise<robotnik_msgs::State>("state", 1);
	
	ros_initialized = true;

	odom_sub_ = nh_.subscribe("odom", 10,  &LaserProtectionArea::odomCallback, this);	
	cmd_sub_ = nh_.subscribe("cmd_vel", 10,  &LaserProtectionArea::cmdCallback, this);	
	io_sub_ = nh_.subscribe("io", 10,  &LaserProtectionArea::ioCallback, this);	
	
	/*
	// EXAMPLES
	// Subscribers
	// topic, queue, callback
	sub_ = nh_.subscribe("topic name", 10,  &LaserProtectionArea::topicCallback, this);	
	
	// Services server
	service_server_ = pnh_.advertiseService("service name", &LaserProtectionArea::serviceServerCb, this);
	// Services client
	service_client_ = nh_.serviceClient<std_srvs::Empty>("service client name");
	
	*/
	// 
	io_service_client_ = nh_.serviceClient<robotnik_msgs::set_digital_output>(io_service_name_);
	
	// Sets up the diagnostic updater
	diagnostic_ = new diagnostic_updater::Updater();
	
	diagnostic_->setHardwareID("LaserProtectionArea");
	diagnostic_->add("State", this, &LaserProtectionArea::diagnosticUpdate);
	diagnostic_->broadcast(0, "Doing important initialization stuff.");
	return OK;

}


/*!	\fn void LaserProtectionArea::rosReadParams
 * 	\brief Reads the params set in ros param server
*/
void LaserProtectionArea::rosReadParams(){
	
	pnh_.param("desired_freq", desired_freq_, DEFAULT_THREAD_DESIRED_HZ);
	pnh_.param<std::string>("io_service_name", io_service_name_, "set_digital_outputs");
	pnh_.param("hysteresis_level_0", hysteresis_level_0_, DEFAULT_HYSTERESIS_0);
	pnh_.param("velocity_level_0", velocity_level_0_, DEFAULT_VELOCITY_LIMIT_0);
	pnh_.param("output_level_0", output_level_0_, DEFAULT_OUTPUT_LEVEL_0);
	pnh_.param("output_logic_0", output_logic_0_, DEFAULT_OUTPUT_LOGIC);
	pnh_.param("hysteresis_level_1", hysteresis_level_1_, DEFAULT_HYSTERESIS_1);
	pnh_.param("velocity_level_1", velocity_level_1_, DEFAULT_VELOCITY_LIMIT_1);
	pnh_.param("output_level_1", output_level_1_, DEFAULT_OUTPUT_LEVEL_1);
	pnh_.param("output_logic_1", output_logic_1_, DEFAULT_OUTPUT_LOGIC);
	
	/* Example 
	pnh_.param<std::string>("port", port_, DEFAULT_DSPIC_PORT);
	pnh_.param<std::string>("odom_frame_id", odom_frame_id_, "/odom_diff");
	pnh_.param<std::string>("base_frame_id", base_frame_id_, "/base_link");
	pnh_.param("publish_tf", publish_tf_, false);
	pnh_.param("desired_freq", desired_freq_, desired_freq_);*/
}

/*!	\fn int LaserProtectionArea::rosShutdown()
 * 	\brief Closes all ros stuff
*/
int LaserProtectionArea::rosShutdown(){
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

/*!	\fn void LaserProtectionArea::rosPublish()
 * 	\brief Reads data a publish several info into different topics
*/
void LaserProtectionArea::rosPublish(){
	robotnik_msgs::State msg;
	
	// STATE
	msg.state = this->state;
	msg.desired_freq = this->desired_freq_;
	msg.real_freq = this->real_freq;
	msg.state_description = getStateString();
	
	
	state_publisher_.publish(msg);
	
}

/*!	\fn void LaserProtectionArea::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat)
 * 	\brief Callback to update the component diagnostic
*/
void LaserProtectionArea::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper &stat){
	
	if(state == robotnik_msgs::State::READY_STATE || state == robotnik_msgs::State::INIT_STATE || state == robotnik_msgs::State::STANDBY_STATE)
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Everything OK!");
	else if (state == robotnik_msgs::State::EMERGENCY_STATE)
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Watch out!");
	else
		stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Error!");
	
	stat.add("State", getStateString());
}
	
	
	
// EXAMPLE: Callback handler associated with the subscriber
void LaserProtectionArea::odomCallback(const nav_msgs::OdometryConstPtr& message)
{
	//ROS_INFO("callback: Received msg: %s", message->data.c_str());
	last_odom_time = ros::Time::now();
	odom_msg_ = *message;
}
void LaserProtectionArea::cmdCallback(const geometry_msgs::TwistConstPtr& message)
{
	//ROS_INFO("callback: Received msg: %s", message->data.c_str());
	last_cmd_time = ros::Time::now();
	cmd_msg_ = *message;
}
void LaserProtectionArea::ioCallback(const robotnik_msgs::inputs_outputsConstPtr& message)
{
	//ROS_INFO("callback: Received msg: %s", message->data.c_str());
	io_msg_ = *message;
	last_io_time = ros::Time::now();
}

// Callback handler for the service server
/*bool LaserProtectionArea::serviceServerCb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	ROS_INFO("serviceServerCb: Received server");
	// Calls the client service
	std_srvs::Empty service;
	
	if(service_client_.call(service)){
		ROS_INFO("serviceServerCb: calling service");
	}else{
		ROS_ERROR("serviceServerCb: Error connecting service %s", service_client_name_.c_str());
	}
	
	return true;
}*/



// MAIN
int main(int argc, char** argv)
{
    ros::init(argc, argv, "LaserProtectionArea");
	
	ros::NodeHandle n;		
  	LaserProtectionArea controller(n);
	
	controller.start();

	return (0);
}

