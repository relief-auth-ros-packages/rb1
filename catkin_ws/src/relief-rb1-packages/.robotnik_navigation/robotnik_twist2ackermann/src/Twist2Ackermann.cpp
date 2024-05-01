#include <robotnik_twist2ackermann/Twist2Ackermann.h>

Twist2Ackermann::Twist2Ackermann(ros::NodeHandle h) : rcomponent::RComponent(h), nh_(h), pnh_("~")
{
  component_name.assign(pnh_.getNamespace());

  rosReadParams();
}

Twist2Ackermann::~Twist2Ackermann()
{
}

int Twist2Ackermann::rosSetup()
{
  RComponent::rosSetup();

  twist_cmd_sub_ = nh_.subscribe(twist_cmd_topic_, 1, &Twist2Ackermann::twistCmdCallback, this);
  ack_status_sub_ = nh_.subscribe(ack_status_topic_, 1, &Twist2Ackermann::ackStatusCallback, this);

  ack_cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(ack_cmd_topic_, 1);
}

void Twist2Ackermann::rosReadParams()
{
  moving_speed_threshold_ = 0.2;
  steering_angle_following_threshold_ = 0.4;
  steering_angle_stopped_threshold_ = 0.05;

  double wheelbase_param = 0;
  pnh_.param<double>("wheelbase", wheelbase_param, wheelbase_param);
  if (setWheelbase(wheelbase_param) == false)
  {
    ROS_FATAL_STREAM("Twist2Ackermann:: invalid wheelbase: " << wheelbase_param << " (should be positive and greater "
                                                                                   "than 0)");
  }

  double min_turning_radius_param = 0;
  pnh_.param<double>("min_turning_radius", min_turning_radius_param, min_turning_radius_param);
  if (setMinimumTurningRadius(min_turning_radius_param) == false)
  {
    ROS_FATAL_STREAM("Twist2Ackermann:: invalid minimum turning radius: " << min_turning_radius_param
                                                                          << " (should be positive and greater "
                                                                             "than 0)");
  }

  double time_watchdog_twist = 0.2;
  pnh_.param<double>("watchdog_twist", time_watchdog_twist, time_watchdog_twist);
  if (setTwistCommandWatchdogTime(time_watchdog_twist) == false)
  {
    ROS_FATAL_STREAM("Twist2Ackermann:: invalid time for twist watchdog: " << time_watchdog_twist << " (should be "
                                                                                                     "positive)");
  }
  double time_watchdog_ackermann_status = 0.2;
  pnh_.param<double>("watchdog_ackermann_status", time_watchdog_ackermann_status, time_watchdog_ackermann_status);
  if (setAckermannStatusWatchdogTime(time_watchdog_ackermann_status) == false)
  {
    ROS_FATAL_STREAM("Twist2Ackermann:: invalid time for status watchdog: " << time_watchdog_ackermann_status
                                                                            << " (should be "
                                                                               "positive)");
  }
}

void Twist2Ackermann::standbyState()
{
  switchToState(robotnik_msgs::State::READY_STATE);
}

void Twist2Ackermann::readyState()
{
  if (ros::Time::now() - twist_cmd_.header.stamp > twist_cmd_watchdog_)
    return;

  ackermann_msgs::AckermannDrive ack_computed;
  transformTwistToAckermann(twist_cmd_.twist, ack_computed);

  ackermann_msgs::AckermannDriveStamped ack_to_send;
  ack_to_send.drive = ack_computed;
  ack_to_send.header.stamp = ros::Time::now();
  if (inPosition(ack_computed) == false)
  {
    ack_to_send.drive.speed = 0;
  }
  ack_cmd_pub_.publish(ack_to_send);
}

void Twist2Ackermann::twistCmdCallback(const geometry_msgs::TwistConstPtr& twist)
{
  twist_cmd_.twist = *twist;
  twist_cmd_.header.stamp = ros::Time::now();
}

void Twist2Ackermann::ackStatusCallback(const ackermann_msgs::AckermannDriveStampedConstPtr& ack)
{
  ack_current_status_ = *ack;
  ack_current_status_.header.stamp = ros::Time::now();
}

bool Twist2Ackermann::inPosition(const ackermann_msgs::AckermannDrive& ack)
{
  // no status received in a while
  if (ros::Time::now() - ack_current_status_.header.stamp > ack_status_watchdog_)
  {
    return false;
  }

  // we are moving
  if (std::abs(ack_current_status_.drive.speed) > moving_speed_threshold_)
  {
    if (std::abs(ack_current_status_.drive.steering_angle - ack.steering_angle) < steering_angle_following_threshold_)
    {
      return true;
    }
    return false;
  }
  else
  {
    if (std::abs(ack_current_status_.drive.steering_angle - ack.steering_angle) < steering_angle_stopped_threshold_)
    {
      return true;
    }

    return false;
  }
  return false;
}

bool Twist2Ackermann::transformTwistToAckermann(const geometry_msgs::Twist& twist, ackermann_msgs::AckermannDrive& ack)
{
  // turning radius is less than the minimum
  if (twist.angular.z != 0 and std::abs(twist.linear.x / twist.angular.z) < min_turning_radius_)
  {
    ROS_WARN_STREAM_THROTTLE(1, "cannot turn with a radius that is below my minimum"
                                    << twist.linear.x << " " << twist.angular.z << " "
                                    << twist.linear.x / twist.angular.z);
    return false;
  }

  double delta = 0;

  if (twist.angular.z == 0)
    delta = 0;
  else
    delta = std::atan(wheelbase_ / (twist.linear.x / twist.angular.z));

  ack.steering_angle = delta;
  ack.steering_angle_velocity = 0;
  ack.speed = twist.linear.x;
  ack.acceleration = 0;
  ack.jerk = 0;

  ROS_INFO_STREAM_THROTTLE(1, ack << " " << wheelbase_ << " " << twist.linear.x << " " << twist.angular.z);
  ROS_INFO_STREAM_THROTTLE(1, twist);

  return true;
}

bool Twist2Ackermann::setWheelbase(double wheelbase)
{
  if (wheelbase <= 0)
  {
    return false;
  }

  wheelbase_ = wheelbase;
  return true;
}

bool Twist2Ackermann::setMinimumTurningRadius(double turning_radius)
{
  if (turning_radius <= 0)
  {
    return false;
  }
  min_turning_radius_ = turning_radius;
  return true;
}

bool Twist2Ackermann::setTwistCommandWatchdogTime(double time)
{
  if (time < 0)
    return false;

  twist_cmd_watchdog_ = ros::Duration(time);
  return true;
}

bool Twist2Ackermann::setAckermannStatusWatchdogTime(double time)
{
  if (time < 0)
    return false;

  ack_status_watchdog_ = ros::Duration(time);
  return true;
}
