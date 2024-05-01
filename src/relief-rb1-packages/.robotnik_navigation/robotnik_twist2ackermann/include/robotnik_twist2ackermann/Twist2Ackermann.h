#ifndef _TWIST2ACKERMANN_
#define _TWIST2ACKERMANN_

#include <ros/ros.h>

#include <rcomponent/rcomponent.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class Twist2Ackermann : public rcomponent::RComponent
{
public:
  Twist2Ackermann(ros::NodeHandle h);
  virtual ~Twist2Ackermann();

  bool setTwistCommandWatchdogTime(double time);
  bool setAckermannStatusWatchdogTime(double time);
  bool setWheelbase(double wheelbase);
  bool setMinimumTurningRadius(double turning_radius);
  bool transformTwistToAckermann(const geometry_msgs::Twist& twist, ackermann_msgs::AckermannDrive& ack);
  //! Returns true if the steering drive is close to the last command received.
  bool inPosition(const ackermann_msgs::AckermannDrive& ack);

protected:
  /* RComponent stuff */

  //! Setups all the ROS' stuff
  virtual int rosSetup();
  ////  //! Shutdowns all the ROS' stuff
  ////  virtual int rosShutdown();
  ////  //! Reads data a publish several info into different topics
  ////  virtual void rosPublish();
  //! Reads params from params server
  virtual void rosReadParams();
  //! Actions performed on standby state
  virtual void standbyState();
  //! Actions performed on ready state
  virtual void readyState();
  ////  //! Actions performed on the emergency state
  ////  virtual void emergencyState();
  ////  //! Actions performed on Failure state
  ////  virtual void failureState();

  /* RComponent stuff !*/

protected:
  /* ROS stuff */

  //! Public node handle, to receive data
  ros::NodeHandle nh_;
  //! Private node hanlde, to read params and publish data
  ros::NodeHandle pnh_;

  //! The command will be published by this publisher
  ros::Publisher ack_cmd_pub_;
  //! Command to be published
  //  ackermann_msgs::AckermannDriveStamped ack_cmd_;

  //! The command will be read by this subscriber
  ros::Subscriber twist_cmd_sub_;
  //! Command to be published
  geometry_msgs::TwistStamped twist_cmd_;

  //! Current status of ackermann drive will be read here
  ros::Subscriber ack_status_sub_;
  ackermann_msgs::AckermannDriveStamped ack_current_status_;

  //! Wheelbase, distance between axles, used to convert the twist command to an ackermann
  double wheelbase_;
  //! Minimum turning radius, twist commands with a lower turning radius will not be followed
  double min_turning_radius_;

  //! Maximum steering angle to validate if steering drive is in position
  double steering_angle_following_threshold_;
  double steering_angle_stopped_threshold_;
  double moving_speed_threshold_;

  // If true, we are in following mode, so the steering angle threshold is bigger
  bool following_mode_;

  void twistCmdCallback(const geometry_msgs::TwistConstPtr& twist);
  void ackStatusCallback(const ackermann_msgs::AckermannDriveStampedConstPtr& ack);

  ros::Duration ack_status_watchdog_;
  ros::Duration twist_cmd_watchdog_;

public:
  //! Topic names: are defined here as const (to not change them) and as public (to allow their access from the outside)
  //! If you want to change them, then remap!
  const std::string twist_cmd_topic_ = "cmd_vel";
  const std::string ack_status_topic_ = "status_ack";
  const std::string ack_cmd_topic_ = "cmd_ack";
};

#endif  //_TWIST2ACKERMANN_
