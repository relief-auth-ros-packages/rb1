/** \file rb1_base_controller.cpp
 * \author Robotnik Automation S.L.L.
 * \version 3.0
 * \date    2017
 *
 * \brief rb1_base_controller class driver
 * Component to manage the RB1 servo controller set
 * (C) 2012 Robotnik Automation, SLL
*/

#include <algorithm>
#include <sstream>
#include <numeric>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <rb1_base_controller/rb1_base_controller.h>

#define RB1_COMMAND_WATCHDOG 0.1
#define DEFAULT_INMOTION_TIMER 2.0
#define RB1_ELEVATOR_DEFAULT_OUTPUT_UP 3
#define RB1_ELEVATOR_DEFAULT_OUTPUT_DOWN 2
#define RB1_ELEVATOR_DEFAULT_INPUT_UP 3
#define RB1_ELEVATOR_DEFAULT_INPUT_DOWN 2
#define ELEVATOR_ACTION_COMMAND_TIMEOUT_MSG 1.0            // default timeout processing an action received
#define ELEVATOR_MOVING_TIMEOUT 10.0                       // default timeout when raising/lowering the elevator
#define RECEIVED_IO_TIMEOUT 2.0                            // in secs. Timeout without receiving io values
#define RB1_ELEVATOR_DEFAULT_ELEVATOR_ACTION_TIMEOUT 10.0  // Max time in action (seconds)
#define RB1_ELEVATOR_DEFAULT_ELEVATOR_IN_TIMEOUT 10.0      // Time in timeout error (auto recover)
#define RB1_ELEVATOR_DEFAULT_HZ 10.0                       // controol loop frequency
#define RB1_ELEVATOR_DEFAULT_POS_UP 0.03
#define RB1_ELEVATOR_DEFAULT_POS_DOWN 0.0
#define RB1_DEFAULT_MAX_JOINT_SPEED 27.27

namespace rb1_base_controller
{
double radnorm(double value)
{
  while (value > M_PI)
    value -= M_PI;
  while (value < -M_PI)
    value += M_PI;
  return value;
}

double radnorm2(double value)
{
  while (value > 2.0 * M_PI)
    value -= 2.0 * M_PI;
  while (value < -2.0 * M_PI)
    value += 2.0 * M_PI;
  return value;
}

double radnormHalf(double value)
{  // norms the angle so it is between -M_PI/2 and M_PI/2
  double eps = 1e-5;
  while (value > 0.5 * M_PI + eps)
    value -= M_PI;
  while (value < -0.5 * M_PI - eps)
    value += M_PI;
  return value;
}

// norms a double value so if it is rounded to zero when it's below an epsylon
double normToZero(double value)
{
  double eps = 1e-4;
  if (std::abs(value) < eps)
    return 0;
  return value;
}

// return the sign, as -1 or 1, of the value. 0 is positive
inline double sign(double value)
{
  return (value < 0) ? -1 : 1;
}

// checks that v and w have the same sign. 0 is positive
inline bool haveSameSign(double v, double w)
{
  return (v < 0) == (w < 0);
}

RB1BaseController::RB1BaseController()
{
  // TODO: initialize all variables
  elevator_action_.action = robotnik_msgs::ElevatorAction::NO_ACTION;
  elevator_status_.state = robotnik_msgs::ElevatorStatus::IDLE;
  elevator_status_.position = robotnik_msgs::ElevatorStatus::UNKNOWN;
  elevator_current_position_ = elevator_position_down_;
  enabled_ = true;
}

// Service SetOdometry
bool RB1BaseController::setOdometrySrvCallback(robotnik_msgs::set_odometry::Request& request,
                                               robotnik_msgs::set_odometry::Response& response)
{
  robot_pose_.x = odom_.pose.pose.position.x = request.x;
  robot_pose_.y = odom_.pose.pose.position.y = request.y;

  tf::Quaternion q;
  q.setRPY(0, 0, request.orientation);

  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();

  robot_pose_.theta = request.orientation;

  // store new orientation
  init_yaw_ = last_yaw_;
  motion_yaw_ = request.orientation;

  ROS_INFO("%s::setOdometrySrvCallback: q.x=%5.2f q.y=%5.2f  q.z=%5.2f  q.w=%5.2f", controller_name_.c_str(), q.x(),
           q.y(), q.z(), q.w());

  response.ret = true;
  return true;
}

void RB1BaseController::readKinematicLimits(KinematicLimits& limit, ros::NodeHandle& controller_nh,
                                            const std::string& base)
{
  if (controller_nh.hasParam(base + "linear_speed_limit") == false)
  {  // limit does not exist!
    ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Cannot find parameter "
                                                                "linear_speed_limit parameter. I will gently set it to "
                                                             << limit.linear_speed);
  }
  else
  {
    controller_nh.param(base + "linear_speed_limit", limit.linear_speed, limit.linear_speed);
    if (limit.linear_speed < 0)
    {
      ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Watch out! You set "
                                                                  "linear_speed_limit, which is the limit of the "
                                                                  "modulo of the linear speed, to a negative value. I "
                                                                  "will gently set it as positive.");
      limit.linear_speed = -limit.linear_speed;
    }
  }

  if (controller_nh.hasParam(base + "linear_acceleration_limit") == false)
  {  // limit does not exist!
    ROS_WARN_STREAM_NAMED(controller_name_, controller_name_
                                                << "::initController: Cannot find parameter linear_acceleration_limit "
                                                   "parameter. I will gently set it to "
                                                << limit.linear_acceleration);
  }
  else
  {
    controller_nh.param(base + "linear_acceleration_limit", limit.linear_acceleration, limit.linear_acceleration);
    if (limit.linear_acceleration < 0)
    {
      ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Watch out! You set "
                                                                  "linear_acceleration_limit, which is the limit of "
                                                                  "the modulo of the linear acceleration, to a "
                                                                  "negative value. I will gently set it as positive.");
      limit.linear_acceleration = -limit.linear_acceleration;
    }
  }

  if (controller_nh.hasParam(base + "linear_deceleration_limit") == false)
  {  // limit does not exist!
    ROS_WARN_STREAM_NAMED(controller_name_, controller_name_
                                                << "::initController: Cannot find parameter linear_deceleration_limit "
                                                   "parameter. I will gently set it to "
                                                << limit.linear_deceleration);
  }
  else
  {
    controller_nh.param(base + "linear_deceleration_limit", limit.linear_deceleration, limit.linear_deceleration);
    if (limit.linear_deceleration < 0)
    {
      ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Watch out! You set "
                                                                  "linear_deceleration_limit, which is the limit of "
                                                                  "the modulo of the linear deceleration, to a "
                                                                  "negative value. I will gently set it as positive.");
      limit.linear_deceleration = -limit.linear_deceleration;
    }
  }

  if (controller_nh.hasParam(base + "angular_speed_limit") == false)
  {  // limit does not exist!
    ROS_WARN_STREAM_NAMED(controller_name_, controller_name_
                                                << "::initController: Cannot find parameter angular_speed_limit "
                                                   "parameter. I will gently set it to "
                                                << limit.angular_speed);
  }
  else
  {
    controller_nh.param(base + "angular_speed_limit", limit.angular_speed, limit.angular_speed);
    if (limit.angular_speed < 0)
    {
      ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Watch out! You set "
                                                                  "angular_speed_limit, which is the limit of the "
                                                                  "modulo of the angular speed, to a negative value. I "
                                                                  "will gently set it as positive.");
      limit.angular_speed = -limit.angular_speed;
    }
  }

  if (controller_nh.hasParam(base + "angular_acceleration_limit") == false)
  {  // limit does not exist!
    ROS_WARN_STREAM_NAMED(controller_name_, controller_name_
                                                << "::initController: Cannot find parameter angular_acceleration_limit "
                                                   "parameter. I will gently set it to "
                                                << limit.angular_acceleration);
  }
  else
  {
    controller_nh.param(base + "angular_acceleration_limit", limit.angular_acceleration, limit.angular_acceleration);
    if (limit.angular_acceleration < 0)
    {
      ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Watch out! You set "
                                                                  "angular_acceleration_limit, which is the limit of "
                                                                  "the modulo of the angular acceleration, to a "
                                                                  "negative value. I will gently set it as positive.");
      limit.angular_acceleration = -limit.angular_acceleration;
    }
  }

  if (controller_nh.hasParam(base + "angular_deceleration_limit") == false)
  {  // limit does not exist!
    ROS_WARN_STREAM_NAMED(controller_name_, controller_name_
                                                << "::initController: Cannot find parameter angular_deceleration_limit "
                                                   "parameter. I will gently set it to "
                                                << limit.angular_deceleration);
  }
  else
  {
    controller_nh.param(base + "angular_deceleration_limit", limit.angular_deceleration, limit.angular_deceleration);
    if (limit.angular_deceleration < 0)
    {
      ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: Watch out! You set "
                                                                  "angular_deceleration_limit, which is the limit of "
                                                                  "the modulo of the angular deceleration, to a "
                                                                  "negative value. I will gently set it as positive.");
      limit.angular_deceleration = -limit.angular_deceleration;
    }
  }
}

bool RB1BaseController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
  bool everything_ok = true;

  controller_name_ = "rb1_base_controller";

  // the topics are hardcoded, the way to change them is by using the remap option
  command_topic_ = "cmd_vel";
  odom_topic_ = "odom";

  elevator_digital_output_up_ = 6;
  elevator_digital_output_down_ = 5;

  // default values for some variables that can change using the param server
  // related to kinematics

  track_width_ = 0.543;
  wheel_diameter_ = 0.233;

  // Modified by li9i
  //track_width_ = 0.421;
  //wheel_diameter_ = 0.1524;

  limits_elevator_down_.linear_speed = 0.1;
  limits_elevator_down_.linear_acceleration = 0.1;
  limits_elevator_down_.linear_deceleration = 0.1;
  limits_elevator_down_.angular_speed = 0.1;
  limits_elevator_down_.angular_acceleration = 0.1;
  limits_elevator_down_.angular_deceleration = 0.1;

  limits_elevator_up_.linear_speed = 0.1;
  limits_elevator_up_.linear_acceleration = 0.1;
  limits_elevator_up_.linear_deceleration = 0.1;
  limits_elevator_up_.angular_speed = 0.1;
  limits_elevator_up_.angular_acceleration = 0.1;
  limits_elevator_up_.angular_deceleration = 0.1;

  set_digital_output_service_hw_ = "robotnik_base_hw/set_digital_output";

  //

  if (controller_nh.hasParam("limits") == false)
  {
    ROS_WARN_STREAM_NAMED(controller_name_, controller_name_
                                                << "::initController: Cannot find parameter "
                                                   "limits parameter. I will gently try to load in the old fashioned "
                                                   "way, but this configuration is DEPRECATED!");
    readKinematicLimits(limits_elevator_down_, controller_nh, "");
    readKinematicLimits(limits_elevator_up_, controller_nh, "");
  }
  else
  {
    readKinematicLimits(limits_elevator_down_, controller_nh, "limits/elevator_down/");
    readKinematicLimits(limits_elevator_up_, controller_nh, "limits/elevator_up/");
  }
  controller_nh.param("track_width", track_width_, track_width_);
  controller_nh.param("wheel_diameter", wheel_diameter_, wheel_diameter_);
  controller_nh.param("cmd_vel", command_topic_, command_topic_);
  controller_nh.param("odom_topic", odom_topic_, odom_topic_);
  controller_nh.param("reverse_logic", reverse_logic_, true);

  controller_nh.param("elevator_digital_output_up", elevator_digital_output_up_, RB1_ELEVATOR_DEFAULT_OUTPUT_UP);
  controller_nh.param("elevator_digital_output_down", elevator_digital_output_down_, RB1_ELEVATOR_DEFAULT_OUTPUT_DOWN);
  controller_nh.param("elevator_digital_input_up", elevator_digital_input_up_, RB1_ELEVATOR_DEFAULT_INPUT_UP);
  controller_nh.param("elevator_digital_input_down", elevator_digital_input_down_, RB1_ELEVATOR_DEFAULT_INPUT_DOWN);
  controller_nh.param<double>("elevator_position_up", elevator_position_up_, RB1_ELEVATOR_DEFAULT_POS_UP);
  controller_nh.param<double>("elevator_position_down_", elevator_position_down_, RB1_ELEVATOR_DEFAULT_POS_DOWN);
  controller_nh.param<bool>("has_elevator", has_elevator_, false);
  controller_nh.param("set_digital_output_service_hw", set_digital_output_service_hw_, set_digital_output_service_hw_);
  controller_nh.param<std::string>("joint/elevator/name", elevator_joint_name_, "rb1_base_elevator_platform_joint");

  odom_linear_velocity_covariance_ = 1e-9;
  odom_angular_velocity_covariance_ = 1e-9;
  odom_position_covariance_ = 1e-6;
  odom_orientation_covariance_ = 1e-6;

  controller_nh.param<double>("linear_velocity_covariance", odom_linear_velocity_covariance_,
                              odom_linear_velocity_covariance_);
  controller_nh.param<double>("angular_velocity_covariance", odom_angular_velocity_covariance_,
                              odom_angular_velocity_covariance_);
  controller_nh.param<double>("position_covariance", odom_position_covariance_, odom_position_covariance_);
  controller_nh.param<double>("orientation_covariance", odom_orientation_covariance_, odom_orientation_covariance_);

  if (elevator_digital_output_up_ < 1)
  {
    ROS_WARN_STREAM_NAMED(controller_name_,
                          controller_name_
                              << "::initController: param elevator_digital_output_up (=" << elevator_digital_output_up_
                              << ") has to be >= 1. Setting default value " << RB1_ELEVATOR_DEFAULT_OUTPUT_UP);
    elevator_digital_output_up_ = RB1_ELEVATOR_DEFAULT_OUTPUT_UP;
  }
  if (elevator_digital_output_down_ < 1)
  {
    ROS_WARN_STREAM_NAMED(
        controller_name_,
        controller_name_ << "::initController: param elevator_digital_output_down (=" << elevator_digital_output_down_
                         << ") has to be >= 1. Setting default value " << RB1_ELEVATOR_DEFAULT_OUTPUT_DOWN);
    elevator_digital_output_down_ = RB1_ELEVATOR_DEFAULT_OUTPUT_DOWN;
  }
  if (elevator_digital_input_up_ < 1)
  {
    ROS_WARN_STREAM_NAMED(controller_name_,
                          controller_name_
                              << "::initController: param elevator_digital_input_up (=" << elevator_digital_input_up_
                              << ") has to be >= 1. Setting default value " << RB1_ELEVATOR_DEFAULT_INPUT_UP);
    elevator_digital_input_up_ = RB1_ELEVATOR_DEFAULT_INPUT_UP;
  }
  if (elevator_digital_input_down_ < 1)
  {
    ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::initController: param elevator_digital_input_down (="
                                                             << elevator_digital_input_down_
                                                             << ") has to be >= 1. Setting default value "
                                                             << RB1_ELEVATOR_DEFAULT_INPUT_DOWN);
    elevator_digital_input_down_ = RB1_ELEVATOR_DEFAULT_INPUT_DOWN;
  }

  // related to coordinate frames
  odom_frame_ = "odom";
  robot_base_frame_ = "base_footprint";
  odom_broadcast_tf_ = true;
  controller_nh.param<std::string>("odom_frame", odom_frame_, odom_frame_);
  controller_nh.param<std::string>("robot_base_frame", robot_base_frame_, robot_base_frame_);
  controller_nh.param("odom_broadcast_tf", odom_broadcast_tf_, odom_broadcast_tf_);

  // realted to control mode and odometry orientation source

  motion_odometry_ = false;
  controller_nh.param("motion_odometry", motion_odometry_, motion_odometry_);

  controller_nh.param<std::string>("imu_topic", imu_topic_,
                                   "imu/data");  // Topic published by the imu_complementary_filter

  // related to timing
  double cmd_watchdog = 0.25;
  double imu_watchdog = 0.1;
  double odom_frequency = 100.0;
  double elevator_action_command_timeout_msg = 0.0;
  in_motion_timer_ = 2.0;
  hard_brake_ = true;
  controller_nh.param("cmd_watchdog_duration", cmd_watchdog, cmd_watchdog);
  controller_nh.param("imu_watchdog_duration", imu_watchdog, imu_watchdog);

  controller_nh.param("odom_publish_frequency", odom_frequency, odom_frequency);
  controller_nh.param("in_motion_timer", in_motion_timer_, in_motion_timer_);
  controller_nh.param("hard_brake", hard_brake_, hard_brake_);
  controller_nh.param("elevator_action_command_timeout_msg", elevator_action_command_timeout_msg,
                      ELEVATOR_ACTION_COMMAND_TIMEOUT_MSG);

  // begin to register ros stuff
  cmd_watchdog_duration_ = ros::Duration(cmd_watchdog);
  imu_watchdog_duration_ = ros::Duration(imu_watchdog);
  odom_publish_period_ = ros::Duration(1.0 / odom_frequency);
  elevator_status_control_loop_period_ = ros::Duration(1.0 / RB1_ELEVATOR_DEFAULT_HZ);
  elevator_action_command_timeout_ = ros::Duration(elevator_action_command_timeout_msg);

  cmd_vel_sub_ = root_nh.subscribe(command_topic_, 1, &RB1BaseController::cmdVelCallback, this);
  imu_sub_ = root_nh.subscribe(imu_topic_, 1, &RB1BaseController::imuCallback, this);
  io_sub_ = root_nh.subscribe("robotnik_base_hw/io", 1, &RB1BaseController::ioCallback, this);

  odom_pub_ = root_nh.advertise<nav_msgs::Odometry>(odom_topic_, 1);
  enabled_pub_ = controller_nh.advertise<std_msgs::Bool>("enabled", 1);

  transform_broadcaster_ = new tf::TransformBroadcaster();
  if (has_elevator_)
  {
    elevator_status_pub_ = controller_nh.advertise<robotnik_msgs::ElevatorStatus>("elevator_status", 1);
    joint_states_pub_ = root_nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    joint_states_msg.name.push_back(elevator_joint_name_);
    joint_states_msg.position.push_back(0.0);
    joint_states_msg.velocity.push_back(0.0);
    joint_states_msg.effort.push_back(0.0);
  }

  set_odometry_srv_ = root_nh.advertiseService("set_odometry", &RB1BaseController::setOdometrySrvCallback, this);
  set_elevator_srv_ = controller_nh.advertiseService("set_elevator", &RB1BaseController::setElevatorSrvCallback, this);
  enable_srv_ = controller_nh.advertiseService("enable", &RB1BaseController::enableSrvCallback, this);

  set_digital_output_client = root_nh.serviceClient<robotnik_msgs::set_digital_output>(set_digital_output_service_hw_);

  elevator_action_server_ = new actionlib::SimpleActionServer<robotnik_msgs::SetElevatorAction>(
      controller_nh, "set_elevator", boost::bind(&RB1BaseController::executeElevatorCallback, this, _1), false);

  joints_.resize(NUMBER_OF_JOINTS);

  joint_states_.resize(NUMBER_OF_JOINTS);
  joint_states_mean_.resize(NUMBER_OF_JOINTS);
  joint_references_.resize(NUMBER_OF_JOINTS);
  joint_commands_.resize(NUMBER_OF_JOINTS);

  // for now, the controller is for a robot with four wheel, and the joint names are these and only these
  joint_names_.resize(NUMBER_OF_JOINTS);
  controller_nh.param<std::string>("joint/right_wheel_joint/name", joint_names_[RIGHT_TRACTION_JOINT], "rb1_base_right_"
                                                                                                       "wheel_joint");
  controller_nh.param<std::string>("joint/left_wheel_joint/name", joint_names_[LEFT_TRACTION_JOINT], "rb1_base_left_"
                                                                                                     "wheel_joint");

  joint_limits_.resize(NUMBER_OF_JOINTS);

  joint_states_history_size_ = 1;
  joint_states_history_.resize(NUMBER_OF_JOINTS);
  for (size_t i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    joint_states_history_[i] = boost::circular_buffer<double>(joint_states_history_size_);
  }

  // set velocity limits
  double right_wheel_max_speed, left_wheel_max_speed;
  controller_nh.param<double>("joint/right_wheel_joint/max_speed", right_wheel_max_speed, RB1_DEFAULT_MAX_JOINT_SPEED);
  controller_nh.param<double>("joint/left_wheel_joint/max_speed", left_wheel_max_speed, RB1_DEFAULT_MAX_JOINT_SPEED);
  joint_limits_[RIGHT_TRACTION_JOINT] = std::make_pair(-right_wheel_max_speed, right_wheel_max_speed);
  joint_limits_[LEFT_TRACTION_JOINT] = std::make_pair(-left_wheel_max_speed, left_wheel_max_speed);

  if (everything_ok)
  {
    ROS_INFO_STREAM_NAMED(controller_name_, controller_name_ << "::initController: everything is OK!");
  }

  // initialize last_yaw_ and delta_yaw_ variable
  init_yaw_ = 0.0;
  last_yaw_ = 0.0;
  delta_yaw_ = 0.0;
  motion_yaw_ = 0.0;
  imu_yaw_ = 0.0;

  // first yaw read
  bFirstYawRead_ = false;
  bYawSensor_ = false;

  joints_[RIGHT_TRACTION_JOINT] = hw->getHandle(joint_names_[RIGHT_TRACTION_JOINT]);
  joints_[LEFT_TRACTION_JOINT] = hw->getHandle(joint_names_[LEFT_TRACTION_JOINT]);

  return everything_ok;
}

/**     \fn  RB1BaseController::setElevatorSrvCallback
 *
 */
bool RB1BaseController::setElevatorSrvCallback(robotnik_msgs::SetElevator::Request& req,
                                               robotnik_msgs::SetElevator::Response& res)
{
  ROS_INFO_THROTTLE(5,"%s::setElevatorSrvCallback: action %d. Configured with has_elevator=%d, RAISE:%d, LOWER:%d",
           controller_name_.c_str(), req.action.action, has_elevator_, robotnik_msgs::ElevatorAction::RAISE,
           robotnik_msgs::ElevatorAction::LOWER);
  res.ret = false;

  if (!has_elevator_)
  {
    ROS_ERROR_THROTTLE(5,"%s::setElevatorSrvCallback: controller with no elevator", controller_name_.c_str());
    return false;
  }

  if (inMotion())
  {
    ROS_WARN_THROTTLE(5,"%s::setElevatorSrvCallback: robot in motion. Action not allowed!", controller_name_.c_str());
    return false;
  }

  if (elevator_status_.state != robotnik_msgs::ElevatorStatus::IDLE)
  {
    ROS_WARN_THROTTLE(5,"%s::setElevatorSrvCallback: elevator not idle!", controller_name_.c_str());
    return false;
  }

  if (req.action.action == robotnik_msgs::ElevatorAction::RAISE or
      req.action.action == robotnik_msgs::ElevatorAction::LOWER)
  {
    last_elevator_action_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR_THROTTLE(5,"%s::setElevatorSrvCallback: Action %d not allowed", controller_name_.c_str(), req.action.action);
    return false;
  }

  elevator_action_ = req.action;

  res.ret = true;

  return true;
}

void RB1BaseController::executeElevatorCallback(const robotnik_msgs::SetElevatorGoalConstPtr& goal)
{
  ros::Rate rate(1);
  ros::Time initial_time;
  robotnik_msgs::SetElevator::Request elevator_request;
  robotnik_msgs::SetElevator::Response elevator_response;

  elevator_request.action = goal->action;
  bool callback_result = false;

  ros::Duration wait_time_for_robot_to_stop(5);
  initial_time = ros::Time::now();
  while (callback_result == false and ((ros::Time::now() - initial_time) < wait_time_for_robot_to_stop))
  {
    callback_result = setElevatorSrvCallback(elevator_request, elevator_response);
  }

  if (callback_result == false)
  {
    elevator_action_result_.result = false;
    elevator_action_result_.status = elevator_status_;
    elevator_action_server_->setAborted(elevator_action_result_, "internal callback failed");
    return;
  }
  if (goal->action.action == robotnik_msgs::ElevatorAction::STOP)
  {
    elevator_action_result_.result = true;
    elevator_action_result_.status = elevator_status_;
    elevator_action_server_->setSucceeded(elevator_action_result_, "ok");
    return;
  }
  bool timedout = false;
  ros::Duration timeout(15);
  initial_time = ros::Time::now();
  rate.sleep();
  while (((goal->action.action == robotnik_msgs::ElevatorAction::RAISE and
           elevator_status_.state == robotnik_msgs::ElevatorStatus::RAISING and
           elevator_status_.position != robotnik_msgs::ElevatorStatus::UP) or
          (goal->action.action == robotnik_msgs::ElevatorAction::LOWER and
           elevator_status_.state == robotnik_msgs::ElevatorStatus::LOWERING and
           elevator_status_.position != robotnik_msgs::ElevatorStatus::DOWN)) and
         (not timedout))
  {
    elevator_action_feedback_.status = elevator_status_;
    elevator_action_server_->publishFeedback(elevator_action_feedback_);
    rate.sleep();
    timedout = (ros::Time::now() - initial_time > timeout);
  }
  if (timedout == true)
  {
    elevator_action_result_.result = false;
    elevator_action_result_.status = elevator_status_;
    elevator_action_server_->setAborted(elevator_action_result_, "timedout");
    return;
  }
  else
  {
    elevator_action_result_.result = true;
    elevator_action_result_.status = elevator_status_;
    elevator_action_server_->setSucceeded(elevator_action_result_, "ok");
  }

  return;
}

std::string RB1BaseController::getHardwareInterfaceType() const
{
  // as result of being a Controller which uses different types of JointInterface, return the main interface type
  // in this case, it is a VelocityJointInterface
  return hardware_interface::internal::demangledTypeName<hardware_interface::VelocityJointInterface>();
}

/**
 * \brief Starts controller
 * \param time Current time
 */
void RB1BaseController::starting(const ros::Time& time)
{
  ROS_INFO_STREAM_NAMED(controller_name_, controller_name_ << ": Starting!");
  odom_last_sent_ = ros::Time::now();
  elevator_loop_last_execution_ = ros::Time::now();
  odom_last_update_ = ros::Time::now();  // check if there is some value that invalidates the result of a substraction
                                         // (equals 0). if there isn't any, seta flag for first_update_odometry
  cmd_last_stamp_ = ros::Time(0);        // maybe it is better to set it to 0, so if no cmd is received
  io_last_stamp_ = ros::Time(0);         // maybe it is better to set it to 0, so if no cmd is received
  last_elevator_action_time = ros::Time(0);
  elevator_action_init_time = ros::Time(0);
  // TODO: service to restart odometry?
  odom_ = nav_msgs::Odometry();
  received_cmd_ = geometry_msgs::Twist();
  robot_pose_ = geometry_msgs::Pose2D();

  cmd_watchdog_timedout_ = true;
  elevator_action_server_->start();
}

/**
 * \brief Stops controller
 * \param time Current time
 */
void RB1BaseController::stopping(const ros::Time& time)
{
  ROS_INFO_STREAM_NAMED(controller_name_, controller_name_ << "Stopping!");
  elevator_action_server_->shutdown();
}

void RB1BaseController::update(const ros::Time& time, const ros::Duration& period)
{
  limitCommand(period.toSec());
  // read joint states:
  //  - convert wheel angular velocity to linear velocity
  //  - normalize motor wheel angular position between [-pi, pi] (only needed for simulation)
  //  - round to 0 if values are below and epsylon
  //  - update joint_states_history_
  readJointStates();

  // update odometry info:
  //  - from joint values (closed loop)
  //  - from command (open loop)
  updateOdometry();

  // calculate joint velocity and position references, taking into account some constrains
  updateJointReferences();

  // TODO: rate of odom publishing
  if ((time - odom_last_sent_) > odom_publish_period_)
  {
    odom_last_sent_ = time;
    publishOdometry();
  }

  cmd_watchdog_timedout_ = ((time - cmd_last_stamp_) > cmd_watchdog_duration_);
  // if (cmd_watchdog_timedout_){
  //    ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << ":: watchdog timedout! " << cmd_watchdog_duration_
  //    << " " << (time - cmd_last_stamp_) );
  //} else ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << ":: watchdog not timedout! " <<
  // cmd_watchdog_duration_ << " " << (time - cmd_last_stamp_));
  imu_watchdog_timedout_ = ((time - imu_last_stamp_) > imu_watchdog_duration_);

  writeJointCommands();

  // limit references according to acceleration and speed limits and hard_brake parameter

  // elevator management
  if (has_elevator_ && (time - elevator_loop_last_execution_) > elevator_status_control_loop_period_)
  {
    publishElevatorStatus();
    elevatorControlLoop(time);
    elevator_loop_last_execution_ = time;
  }

  // enabled/disabled status
  std_msgs::Bool enabled_msg;
  enabled_msg.data = enabled_;
  enabled_pub_.publish(enabled_msg);
}

/**
 * \fn void RB1BaseController::elevatorControlLoop(const ros::Time& time)
 * \brief Performs the internal state machine to control de elevator system
 */
void RB1BaseController::elevatorControlLoop(const ros::Time& time)
{
  // Check the current status

  if (elevator_status_.state == robotnik_msgs::ElevatorStatus::IDLE)
  {
    // Check if a new action has been received
    if ((time - last_elevator_action_time) <= elevator_action_command_timeout_)
    {
      // processing the command
      elevator_ongoing_action_ = elevator_action_;

	  if(enabled_){
		  if (elevator_ongoing_action_.action == robotnik_msgs::ElevatorAction::RAISE)
		  {
			switchToElevatorState((string)robotnik_msgs::ElevatorStatus::RAISING);
		  }
		  else if (elevator_ongoing_action_.action == robotnik_msgs::ElevatorAction::LOWER)
		  {
			switchToElevatorState((string)robotnik_msgs::ElevatorStatus::LOWERING);
		  }
      }else
		ROS_WARN_STREAM_THROTTLE_NAMED(10, controller_name_, controller_name_ << "::elevatorControlLoop: controller is disabled");
    }
  }
  // RAISING
  else if (elevator_status_.state == robotnik_msgs::ElevatorStatus::RAISING)
  {
    if (received_io_.digital_outputs[elevator_digital_output_down_ - 1])
    {
      setDigitalOutput(elevator_digital_output_down_, false);
      // ROS_WARN("RAISING: disabling output down");
    }
    if (not received_io_.digital_outputs[elevator_digital_output_up_ - 1])
    {
      setDigitalOutput(elevator_digital_output_up_, true);
      // ROS_WARN("RAISING: enabling output up");
    }

    if (elevator_status_.position == robotnik_msgs::ElevatorStatus::UP)
    {
      ROS_INFO("%s::elevatorControlLoop: RAISING - >elevator Up", controller_name_.c_str());
      switchToElevatorState((string)robotnik_msgs::ElevatorStatus::IDLE);
      setDigitalOutput(elevator_digital_output_up_, false);
      // Setting the fake joint of the controller
      elevator_current_position_ = elevator_position_up_;
    }
    else
    {
      // Checking timeout
      if ((time - elevator_action_init_time).toSec() > RB1_ELEVATOR_DEFAULT_ELEVATOR_ACTION_TIMEOUT)
      {
        ROS_ERROR("%s::elevatorControlLoop: timeout in elevator RAISING action. %.1f seconds without finishing the "
                  "action",
                  controller_name_.c_str(), RB1_ELEVATOR_DEFAULT_ELEVATOR_ACTION_TIMEOUT);
        switchToElevatorState((string)robotnik_msgs::ElevatorStatus::ERROR_TIMEOUT);
      }
    }
  }
  // LOWERING
  else if (elevator_status_.state == robotnik_msgs::ElevatorStatus::LOWERING)
  {
    if (received_io_.digital_outputs[elevator_digital_output_up_ - 1])
    {
      setDigitalOutput(elevator_digital_output_up_, false);
      // ROS_WARN("LOWERING: disabling output up");
    }
    if (not received_io_.digital_outputs[elevator_digital_output_down_ - 1])
    {
      setDigitalOutput(elevator_digital_output_down_, true);
      // ROS_WARN("LOWERING: enabling output down");
    }

    if (elevator_status_.position == robotnik_msgs::ElevatorStatus::DOWN)
    {
      ROS_INFO("%s::elevatorControlLoop: LOWERING - >elevator Down", controller_name_.c_str());
      switchToElevatorState((string)robotnik_msgs::ElevatorStatus::IDLE);
      setDigitalOutput(elevator_digital_output_down_, false);
      // Setting the fake joint of the controller
      elevator_current_position_ = elevator_position_down_;
    }
    else
    {
      // Checking timeout
      if ((time - elevator_action_init_time).toSec() > RB1_ELEVATOR_DEFAULT_ELEVATOR_ACTION_TIMEOUT)
      {
        ROS_ERROR("%s::elevatorControlLoop: timeout in elevator LOWERING action. %.1f seconds without finishing the "
                  "action",
                  controller_name_.c_str(), RB1_ELEVATOR_DEFAULT_ELEVATOR_ACTION_TIMEOUT);
        switchToElevatorState((string)robotnik_msgs::ElevatorStatus::ERROR_TIMEOUT);
      }
    }
  }
  // ERROR GETTING I/O VALUES
  else if (elevator_status_.state == robotnik_msgs::ElevatorStatus::ERROR_G_IO)
  {
    if ((time - io_last_stamp_).toSec() <= RECEIVED_IO_TIMEOUT)
    {
      switchToElevatorState((string)robotnik_msgs::ElevatorStatus::IDLE);
    }
  }
  // ERROR TIMEOUT
  else if (elevator_status_.state == robotnik_msgs::ElevatorStatus::ERROR_TIMEOUT)
  {
    if ((time - elevator_action_init_time).toSec() > RB1_ELEVATOR_DEFAULT_ELEVATOR_IN_TIMEOUT)
    {
      switchToElevatorState((string)robotnik_msgs::ElevatorStatus::IDLE);
    }
  }

  // CHECKING ERRORS in all states
  if ((time - io_last_stamp_).toSec() > RECEIVED_IO_TIMEOUT)
  {
    ROS_ERROR("%s::elevatorControlLoop: I/O state is not being received anymore", controller_name_.c_str());
    switchToElevatorState((string)robotnik_msgs::ElevatorStatus::ERROR_G_IO);
  }
  else
  {
    // Processing elevator state
    if (reverse_logic_)
    {
      if (!received_io_.digital_inputs[elevator_digital_input_up_ - 1])
      {
        elevator_status_.position = robotnik_msgs::ElevatorStatus::UP;
      }
      else if (!received_io_.digital_inputs[elevator_digital_input_down_ - 1])
      {
        elevator_status_.position = robotnik_msgs::ElevatorStatus::DOWN;
      }
    }
    else
    {
      if (received_io_.digital_inputs[elevator_digital_input_up_ - 1])
      {
        elevator_status_.position = robotnik_msgs::ElevatorStatus::UP;
      }
      else if (received_io_.digital_inputs[elevator_digital_input_down_ - 1])
      {
        elevator_status_.position = robotnik_msgs::ElevatorStatus::DOWN;
      }
    }
  }
}

/**
   * \fn void RB1BaseController::switchToElevatorState(string new_state)
   * \brief Transition function between elevator states
   */
void RB1BaseController::switchToElevatorState(string new_state)
{
  if (new_state == elevator_status_.state)
    return;
  ROS_INFO("%s::switchToElevatorState: from %s to %s", controller_name_.c_str(), elevator_status_.state.c_str(),
           new_state.c_str());

  elevator_status_.state = new_state;

  if (elevator_status_.state == robotnik_msgs::ElevatorStatus::ERROR_G_IO)
  {
    elevator_status_.position = robotnik_msgs::ElevatorStatus::UNKNOWN;
    ROS_WARN("%s::switchToElevatorState: elevator on error, disabling all the outputs", controller_name_.c_str());
    setDigitalOutput(elevator_digital_output_down_, false);
    setDigitalOutput(elevator_digital_output_up_, false);
  }
  else if (elevator_status_.state == robotnik_msgs::ElevatorStatus::IDLE)
  {
    elevator_action_.action = robotnik_msgs::ElevatorAction::NO_ACTION;
  }
  else if ((elevator_status_.state == robotnik_msgs::ElevatorStatus::LOWERING) or
           (elevator_status_.state == robotnik_msgs::ElevatorStatus::RAISING))
  {
    elevator_action_init_time = ros::Time::now();  // Set the time of the action
  }
  else if (elevator_status_.state == robotnik_msgs::ElevatorStatus::ERROR_TIMEOUT)
  {
    elevator_action_init_time = ros::Time::now();  // Using the same timer to stay in this error state
    elevator_status_.position = robotnik_msgs::ElevatorStatus::UNKNOWN;
    ROS_WARN("%s::switchToElevatorState: elevator on error, disabling all the outputs", controller_name_.c_str());
    setDigitalOutput(elevator_digital_output_down_, false);
    setDigitalOutput(elevator_digital_output_up_, false);
  }
}

// inline int sign(double v)
//{
//  return v >= 0 : 1 ? 0;
//}

void RB1BaseController::limitCommand(double period)
{
  // XXX: we expect limits to be more restrictive when elevator up,
  // so apply limits for elevator down only if it is down. if we don't know, apply the
  // more restrictive
  const KinematicLimits& current_limit =
      (elevator_status_.position == robotnik_msgs::ElevatorStatus::DOWN) ? limits_elevator_down_ : limits_elevator_up_;

  bool linear_is_braking = received_cmd_.linear.x == 0 or
                           (sign(received_cmd_.linear.x) != sign(current_cmd_.linear.x)) or
                           (sign(received_cmd_.linear.x) != sign(received_cmd_.linear.x - current_cmd_.linear.x));

  double linear_acceleration =
      linear_is_braking ? current_limit.linear_deceleration : current_limit.linear_acceleration;
  double vx, w;

  double accel_x = (received_cmd_.linear.x - current_cmd_.linear.x) / period;

  //  if (linear_is_braking == true)
  //    ROS_INFO_STREAM_THROTTLE(1, "deceleration " << linear_acceleration << ", r: " << received_cmd_.linear.x << ",
  //    sr: "
  //                                                << sign(received_cmd_.linear.x) << " c: " << current_cmd_.linear.x
  //                                                << " d: " << received_cmd_.linear.x - current_cmd_.linear.x);
  //  else
  //    ROS_INFO_STREAM_THROTTLE(1, "acceleration " << linear_acceleration << ", r: " << received_cmd_.linear.x << ",
  //    sr: "
  //                                                << sign(received_cmd_.linear.x) << " c: " << current_cmd_.linear.x
  //                                                << " d: " << received_cmd_.linear.x - current_cmd_.linear.x);

  if (accel_x > linear_acceleration)
    accel_x = linear_acceleration;
  if (accel_x < -linear_acceleration)
    accel_x = -linear_acceleration;

  vx = current_cmd_.linear.x + accel_x * period;

  if (vx > current_limit.linear_speed)
    vx = current_limit.linear_speed;
  if (vx < -current_limit.linear_speed)
    vx = -current_limit.linear_speed;

  // If desired speed is zero, does not apply deceleration
  if (received_cmd_.linear.x == 0.0)
  {
    if (hard_brake_)
      current_cmd_.linear.x = 0.0;
    else
      current_cmd_.linear.x = vx;
  }
  else
  {
    current_cmd_.linear.x = vx;
  }

  bool angular_is_braking = received_cmd_.angular.z == 0 or
                            (sign(received_cmd_.angular.z) != sign(current_cmd_.angular.z)) or
                            (sign(received_cmd_.angular.z) != sign(received_cmd_.angular.z - current_cmd_.angular.z));

  double angular_acceleration =
      angular_is_braking ? current_limit.angular_deceleration : current_limit.angular_acceleration;
  ///  if (angular_is_braking == true)
  ///  ROS_INFO_STREAM_THROTTLE(1, "applying angular deceleration " << angular_acceleration);
  /// else
  /// ROS_INFO_STREAM_THROTTLE(1, "applying angular acceleration " << angular_acceleration);

  double accel_w = (received_cmd_.angular.z - current_cmd_.angular.z) / period;
  if (std::abs(accel_w) > angular_acceleration)
    accel_w = sign(accel_w) * angular_acceleration;

  w = current_cmd_.angular.z + accel_w * period;
  if (std::abs(w) > current_limit.angular_speed)
    w = sign(w) * current_limit.angular_speed;

  if (received_cmd_.angular.z == 0.0)
  {
    current_cmd_.angular.z = 0.0;
  }
  else
  {
    if (hard_brake_)
      current_cmd_.angular.z = 0;
    else
      current_cmd_.angular.z = w;
  }
}

void RB1BaseController::readJointStates()
{
  // read wheel velocity: convert from angular to linear
  for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++)
  {
    joint_states_[i] = normToZero(joints_[i].getVelocity());
  }

  for (size_t i = 0; i < NUMBER_OF_JOINTS; i++)
  {
    joint_states_history_[i].push_back(joint_states_[i]);
    double sum = std::accumulate(joint_states_history_[i].begin(), joint_states_history_[i].end(), 0.0);
    joint_states_mean_[i] = sum / joint_states_history_[i].size();
  }
}

void RB1BaseController::writeJointCommands()
{
  // set joint_commands_ to the values that must be sent to the actuators.
  // joint_commands_[i] can be, sorted by priority (from more to less):
  //   (1) if watchdog has timedout, vel = 0, pos is not sent
  //   (2) if motorowheels are not in position, vel = 0, pos is sent
  //   (3) default: vel and pos are sent

  static bool must_print_watchdog_message = false;
  bool disable_command = false;

  if (cmd_watchdog_timedout_)
  {
    if (must_print_watchdog_message)
    {
      ROS_WARN_STREAM_NAMED(controller_name_, controller_name_ << "::writeJointCommands: watchdog timedout!");
      must_print_watchdog_message = false;
    }
    disable_command = true;
  }

  if (elevator_status_.state != robotnik_msgs::ElevatorStatus::IDLE &&
      elevator_status_.state != robotnik_msgs::ElevatorStatus::ERROR_G_IO)
  {
    ROS_WARN_STREAM_THROTTLE_NAMED(2, controller_name_, controller_name_ << "::writeJointCommands: elevator active! "
                                                                            "disabling commands");
    disable_command = true;
  }

  if (disable_command)
  {
    //  TODO: send 0 velocity always, or just the first time
    // set all commands to 0
    for (size_t i = 0; i < NUMBER_OF_JOINTS; i++)
    {
      joint_commands_[i] = joint_references_[i] = 0;
    }
    // but only send speed commands
    for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++)
    {
      joints_[i].setCommand(joint_commands_[i]);
    }
    current_cmd_.linear.x = 0.0;
    current_cmd_.linear.y = 0.0;
    current_cmd_.angular.z = 0.0;
    return;
  }

  must_print_watchdog_message = true;

  for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++)
    joint_commands_[i] = joint_references_[i];

  // send commands to actuators
  for (size_t i = 0; i < NUMBER_OF_JOINTS; i++)
    joints_[i].setCommand(joint_commands_[i]);

  std::ostringstream oss;
  oss << "commands:";
  for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++)
  {
    oss << " wheel " << i << " : " << joint_commands_[i];
  }

  // ROS_INFO_STREAM_THROTTLE(1, oss.str());
}

void RB1BaseController::updateJointReferences()
{
  std::vector<double> q;
  q.resize(2);
  // Speed references for motor control
  if (enabled_){
	  double vx = current_cmd_.linear.x;
	  double w = current_cmd_.angular.z;

	  // divide by (wheel_diameter_/2.0) to convert from m/s to rad/s
	  q[0] = (vx + w * track_width_) / (wheel_diameter_ / 2.0);
	  q[1] = (vx - w * track_width_) / (wheel_diameter_ / 2.0);

	  // joint velocity references are scaled so each wheel does not exceed it's maximum velocity
	  setJointVelocityReferenceBetweenJointLimits(q);
	  // ROS_INFO_THROTTLE(1,"q1234=(%5.2f, %5.2f, %5.2f, %5.2f)", q[0],q[1],q[2],q[3]);
  }else{
	  q[0] = 0.0;
	  q[1] = 0.0;
	  ROS_WARN_STREAM_THROTTLE_NAMED(10, controller_name_, controller_name_ << "::updateJointReferences: controller is disabled");
  }
  // Motor control actions
  // Axis are not reversed in the omni (swerve) configuration
  joint_references_[RIGHT_TRACTION_JOINT] = q[0];
  joint_references_[LEFT_TRACTION_JOINT] = q[1];
}

void RB1BaseController::updateOdometry()
{
  // Linear speed of each wheel
  double vx = 0.0, vy = 0.0, w = 0.0;
  double frw1 = joint_states_[RIGHT_TRACTION_JOINT] * (wheel_diameter_ / 2.0);  // from rad/s to m/s
  double flw2 = joint_states_[LEFT_TRACTION_JOINT] * (wheel_diameter_ / 2.0);

  // Avoid to integrate motor noise
  if (fabs(frw1) < 0.001)
    frw1 = 0.0;
  if (fabs(flw2) < 0.001)
    flw2 = 0.0;

  // Get real freq.
  ros::Time current_time = ros::Time::now();
  double seconds_since_last_update = (current_time - odom_last_update_).toSec();
  odom_last_update_ = current_time;

  // Compute Position
  double fSamplePeriod = seconds_since_last_update;

  double v_left_mps, v_right_mps;
  v_left_mps = flw2;   //((flw2 + blw3) / 2.0);
  v_right_mps = frw1;  //((frw1 + brw4) / 2.0);

  vx = (v_right_mps + v_left_mps) / 2.0;  // m/s
  vy = 0.0;
  w = (v_right_mps - v_left_mps) / (2.0 * track_width_);  // rad/s

  // robot_pose_.x += cos(robot_pose_.theta) * vx * fSamplePeriod + cos(M_PI_2 + robot_pose_.theta) * vy *
  // fSamplePeriod;
  // robot_pose_.y += sin(robot_pose_.theta) * vx * fSamplePeriod + sin(M_PI_2 + robot_pose_.theta) * vy *
  // fSamplePeriod;
  robot_pose_.x += cos(robot_pose_.theta) * vx * fSamplePeriod - sin(robot_pose_.theta) * vy * fSamplePeriod;
  robot_pose_.y += sin(robot_pose_.theta) * vx * fSamplePeriod + cos(robot_pose_.theta) * vy * fSamplePeriod;

  // TODO - Check if bYawSensor is necessary.
  if (!imu_watchdog_timedout_)
    robot_pose_.theta = imu_yaw_;
  else
    robot_pose_.theta += w * fSamplePeriod;

  // ROS_INFO("Odom estimated x=%5.2f  y=%5.2f a=%5.2f", robot_pose_px_, robot_pose_py_, robot_pose_pa_);

  tf::Quaternion qt;
  tf::Vector3 vt;
  qt.setRPY(0, 0, robot_pose_.theta);
  vt = tf::Vector3(robot_pose_.x, robot_pose_.y, 0);

  odom_.header.stamp = ros::Time::now();
  odom_.header.frame_id = odom_frame_;
  odom_.child_frame_id = robot_base_frame_;

  odom_.pose.pose.position.x = vt.x();
  odom_.pose.pose.position.y = vt.y();
  odom_.pose.pose.position.z = vt.z();

  odom_.pose.pose.orientation.x = qt.x();
  odom_.pose.pose.orientation.y = qt.y();
  odom_.pose.pose.orientation.z = qt.z();
  odom_.pose.pose.orientation.w = qt.w();

  odom_.pose.covariance[0] = odom_.pose.covariance[7] = odom_.pose.covariance[14] = odom_position_covariance_;
  odom_.pose.covariance[21] = odom_.pose.covariance[28] = odom_.pose.covariance[35] = odom_orientation_covariance_;

  odom_.twist.twist.linear.x = vx;
  odom_.twist.twist.linear.y = vy;
  odom_.twist.twist.angular.z = w;

  odom_.twist.covariance[0] = odom_.twist.covariance[7] = odom_.twist.covariance[14] = odom_linear_velocity_covariance_;
  odom_.twist.covariance[21] = odom_.twist.covariance[28] = odom_.twist.covariance[35] =
      odom_angular_velocity_covariance_;
}

bool RB1BaseController::inMotion()
{
  static bool had_ref = false;
  static double starttime = 0.0;
  bool ref_timer = false;
  // bool e_stop = ((motor_velocity_[0]->GetMotorDigitalInputs() & DIGITAL_INPUT_ESTOP)!=0);
  bool e_stop = false;  // could be read from topic.
  bool zero_speed_ref =
      (received_cmd_.linear.x == 0) && (received_cmd_.linear.y == 0) && (received_cmd_.angular.z == 0);
  bool ref = (!e_stop && !zero_speed_ref);
  if (had_ref && !ref)
    starttime = ros::Time::now().toSec();
  double now = ros::Time::now().toSec();
  if ((starttime != 0.0) && (now - starttime < in_motion_timer_))
  {
    ref_timer = true;
  }
  else
  {
    ref_timer = false;
    starttime = 0.0;
  }
  had_ref = ref;

  return ref || ref_timer;
}

void RB1BaseController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  ROS_DEBUG_STREAM_NAMED(controller_name_, "Received command: (" << cmd_msg->linear.x << ")"
                                                                 << ", " << cmd_msg->linear.y << ", "
                                                                 << cmd_msg->angular.z << ")");
  received_cmd_ = *cmd_msg;

  cmd_last_stamp_ = ros::Time::now();
}

void RB1BaseController::ioCallback(const robotnik_msgs::inputs_outputsConstPtr& msg)
{
  // ROS_INFO_STREAM_NAMED(controller_name_, "::ioCallback: Received command");
  received_io_ = *msg;
  io_last_stamp_ = ros::Time::now();
}

void RB1BaseController::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  double roll_msg, pitch_msg, yaw_msg;
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf::Matrix3x3(q).getRPY(roll_msg, pitch_msg, yaw_msg);

  if (bFirstYawRead_)
    delta_yaw_ = yaw_msg - last_yaw_;

  imu_last_stamp_ = ros::Time::now();

  // Store the first value received
  if (!bFirstYawRead_)
  {
    init_yaw_ = yaw_msg;
    bFirstYawRead_ = true;
    motion_yaw_ = init_yaw_;  //
  }

  // Compute angle according to the init value
  last_yaw_ = yaw_msg;

  if (motion_odometry_)
  {
    // Integrate yaw only if robot is in motion (avoids integrating drift when robot is stopped for long periods)
    bool in_motion = this->inMotion();
    if (in_motion)
      motion_yaw_ += delta_yaw_;
    imu_yaw_ = radnorm2(motion_yaw_);
  }
  else
  {
    // Compute yaw just according to imu value (takes into account external actions on the robot)
    imu_yaw_ = radnorm2(last_yaw_ - init_yaw_);
  }

  // Use this value (note that if we use imu, this function is used only as a storage
  bYawSensor_ = true;
}

void RB1BaseController::publishOdometry()
{
  odom_pub_.publish(odom_);

  tf::Quaternion qt = tf::Quaternion(odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,
                                     odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
  tf::Vector3 vt = tf::Vector3(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);

  tf::Transform base_footprint_to_odom(qt, vt);
  if (this->odom_broadcast_tf_)
  {
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, ros::Time::now(), odom_frame_, robot_base_frame_));
  }
}

void RB1BaseController::publishElevatorStatus()
{
  joint_states_msg.header.stamp = ros::Time::now();
  joint_states_msg.position = { elevator_current_position_ };

  elevator_status_pub_.publish(elevator_status_);
  joint_states_pub_.publish(joint_states_msg);
}

void RB1BaseController::setJointVelocityReferenceBetweenJointLimits(std::vector<double>& wheel_speed)
{
  double max_scale_factor = 1.0;

  for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++)
  {
    double lower_limit = joint_limits_[i].first;
    double upper_limit = joint_limits_[i].second;

    double lower_scale_factor, upper_scale_factor;
    lower_scale_factor = upper_scale_factor = 1.0;

    if (wheel_speed[i] < lower_limit)
      lower_scale_factor = std::abs(wheel_speed[i] / lower_limit);
    if (upper_limit < wheel_speed[i])
      upper_scale_factor = std::abs(wheel_speed[i] / upper_limit);

    max_scale_factor = std::max(max_scale_factor, std::max(lower_scale_factor, upper_scale_factor));
  }

  for (size_t i = BEGIN_TRACTION_JOINT; i < END_TRACTION_JOINT; i++)
  {
    wheel_speed[i] /= max_scale_factor;
  }
}

int RB1BaseController::setDigitalOutput(int number, bool value)
{
  robotnik_msgs::set_digital_output io_srv_msg;

  io_srv_msg.request.value = value;
  io_srv_msg.request.output = number;
  set_digital_output_client.call(io_srv_msg);

  return 0;
}


/**     \fn  RB1BaseController::enableSrvCallback
 *		Enables/Disables the controller to accept or not velocity and action commands
 */
bool RB1BaseController::enableSrvCallback(robotnik_msgs::enable_disable::Request& req,
                                               robotnik_msgs::enable_disable::Response& res)
{
  enabled_ = req.value;

  res.ret = true;

  return true;
}
}
