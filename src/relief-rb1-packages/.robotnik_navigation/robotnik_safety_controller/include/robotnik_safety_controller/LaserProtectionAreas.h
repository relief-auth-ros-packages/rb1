#ifndef _LASER_PROTECTION_AREAS_
#define _LASER_PROTECTION_AREAS_

#include <rcomponent/rcomponent.h>

#include <ros/ros.h>

#include <robotnik_msgs/State.h>
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/set_digital_output.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/publisher.h>

//! Size of string for logging
#define TOPIC_WATCHDOG 2.0  // wathdog reading topics in secs
#define DEFAULT_HYSTERESIS_0 0.05
#define DEFAULT_VELOCITY_LIMIT_0 0.2
#define DEFAULT_OUTPUT_LEVEL_0 1
#define DEFAULT_HYSTERESIS_1 0.3
#define DEFAULT_VELOCITY_LIMIT_1 0.4
#define DEFAULT_OUTPUT_LEVEL_1 2
#define DEFAULT_OUTPUT_LOGIC true  // Inverts the value of the output activation

class LaserProtectionAreas : public rcomponent::RComponent
{
public:
  LaserProtectionAreas(ros::NodeHandle h);
  virtual ~LaserProtectionAreas();

protected:
  /* RComponent stuff */

  //! Setups all the ROS' stuff
  virtual int rosSetup();
  //! Reads params from params server
  virtual void rosReadParams();

  //! Actions performed on standby state
  virtual void standbyState();
  //! Actions performed on ready state
  virtual void readyState();

  /* RComponent stuff !*/

protected:
  /* ROS stuff */
  //! Public node handle, to receive data
  ros::NodeHandle nh_;
  //! Private node hanlde, to read params and publish data
  ros::NodeHandle pnh_;

  ros::Subscriber odom_sub_;              // topic subscriber
  ros::Subscriber cmd_sub_;               // topic subscriber
  ros::Subscriber io_sub_;                // topic subscriber
  ros::ServiceClient io_service_client_;  // service client

  //! General status diagnostic updater
  diagnostic_updater::Updater* diagnostic_;

  //! Diagnostic updater callback
  void diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void odomCallback(const nav_msgs::OdometryConstPtr& message);
  void cmdCallback(const geometry_msgs::TwistConstPtr& message);
  void ioCallback(const robotnik_msgs::inputs_outputsConstPtr& message);
  /* ROS stuff !*/

  /* Protection areas stuff */
  ros::Time last_odom_time;
  ros::Time last_cmd_time;
  ros::Time last_io_time;
  string io_service_name_;
  nav_msgs::Odometry odom_msg_;
  geometry_msgs::Twist cmd_msg_;
  robotnik_msgs::inputs_outputs io_msg_;
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
  /* Protection areas stuff !*/
};
#endif  // _LASER_PROTECTION_AREAS_
