#include <robotnik_safety_controller/LaserProtectionAreas.h>

LaserProtectionAreas::LaserProtectionAreas(ros::NodeHandle h) : RComponent(h), nh_(h), pnh_("~")
{
  component_name.assign(pnh_.getNamespace());

  rosReadParams();
}

LaserProtectionAreas::~LaserProtectionAreas()
{
}

int LaserProtectionAreas::rosSetup()
{
  RComponent::rosSetup();

  odom_sub_ = nh_.subscribe("odom", 10, &LaserProtectionAreas::odomCallback, this);
  cmd_sub_ = nh_.subscribe("cmd_vel", 10, &LaserProtectionAreas::cmdCallback, this);
  io_sub_ = nh_.subscribe("io", 10, &LaserProtectionAreas::ioCallback, this);

  io_service_client_ = nh_.serviceClient<robotnik_msgs::set_digital_output>(io_service_name_);

  // Sets up the diagnostic updater
  diagnostic_ = new diagnostic_updater::Updater();

  diagnostic_->setHardwareID("LaserProtectionAreas");
  diagnostic_->add("State", this, &LaserProtectionAreas::diagnosticUpdate);
  diagnostic_->broadcast(0, "Doing important initialization stuff.");
  return rcomponent::OK;
}

void LaserProtectionAreas::rosReadParams()
{
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
}

void LaserProtectionAreas::diagnosticUpdate(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (state == robotnik_msgs::State::READY_STATE || state == robotnik_msgs::State::INIT_STATE ||
      state == robotnik_msgs::State::STANDBY_STATE)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Everything OK!");
  else if (state == robotnik_msgs::State::EMERGENCY_STATE)
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Watch out!");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Error!");

  stat.add("State", getStateString());
}
void LaserProtectionAreas::odomCallback(const nav_msgs::OdometryConstPtr& message)
{
  // ROS_INFO("callback: Received msg: %s", message->data.c_str());
  last_odom_time = ros::Time::now();
  odom_msg_ = *message;
}
void LaserProtectionAreas::cmdCallback(const geometry_msgs::TwistConstPtr& message)
{
  // ROS_INFO("callback: Received msg: %s", message->data.c_str());
  last_cmd_time = ros::Time::now();
  cmd_msg_ = *message;
}
void LaserProtectionAreas::ioCallback(const robotnik_msgs::inputs_outputsConstPtr& message)
{
  // ROS_INFO("callback: Received msg: %s", message->data.c_str());
  io_msg_ = *message;
  last_io_time = ros::Time::now();
}

void LaserProtectionAreas::standbyState()
{
  ros::Time t_now = ros::Time::now();
  bool topics_ok = true;
  if ((t_now - last_odom_time).toSec() > TOPIC_WATCHDOG)
  {
    topics_ok = false;
    ROS_WARN_STREAM_THROTTLE_NAMED(2, component_name, component_name << "::standbyState: odom not received");
  }
  if ((t_now - last_io_time).toSec() > TOPIC_WATCHDOG)
  {
    topics_ok = false;
    ROS_WARN_STREAM_THROTTLE_NAMED(2, component_name, component_name << "::standbyState: io not received");
  }

  if (topics_ok)
    switchToState(robotnik_msgs::State::READY_STATE);
}

/*!	\fn void LaserProtectionAreas::readyState()
 *	\brief Actions performed on ready state
*/
void LaserProtectionAreas::readyState()
{
  ros::Time t_now = ros::Time::now();
  bool topics_ok = true;
  bool check_cmd = false;

  if ((t_now - last_odom_time).toSec() > TOPIC_WATCHDOG)
  {
    topics_ok = false;
    ROS_WARN_STREAM_THROTTLE_NAMED(2, component_name, component_name << "::readyState: odom not received");
  }
  if ((t_now - last_io_time).toSec() > TOPIC_WATCHDOG)
  {
    topics_ok = false;
    ROS_WARN_STREAM_THROTTLE_NAMED(2, component_name, component_name << "::readyState: io not received");
  }

  if ((t_now - last_cmd_time).toSec() > TOPIC_WATCHDOG)
  {
    check_cmd = false;
  }

  if (not topics_ok)
  {
    switchToState(robotnik_msgs::State::STANDBY_STATE);
    return;
  }

  double current_linear_vel = 0.0;
  current_linear_vel = sqrt(pow(odom_msg_.twist.twist.linear.x, 2) + pow(odom_msg_.twist.twist.linear.y, 2));
  double current_cmd_linear_vel = 0.0;
  current_cmd_linear_vel = sqrt(pow(cmd_msg_.linear.x, 2) + pow(cmd_msg_.linear.y, 2));

  switch (current_area)
  {
    case 0:
      if (current_linear_vel > velocity_level_1_ or (check_cmd and current_cmd_linear_vel > velocity_level_1_))
      {
        current_area = 2;
        ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area "
                                                                         << current_area
                                                                         << ". Vel = " << current_linear_vel);
      }
      else if (current_linear_vel > velocity_level_0_ or (check_cmd and current_cmd_linear_vel > velocity_level_0_))
      {
        current_area = 1;
        ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area "
                                                                         << current_area
                                                                         << ". Vel = " << current_linear_vel);
      }
      else
      {
        bool must_write_level_0 = false;
        robotnik_msgs::set_digital_output write_do_level_0;
        if (io_msg_.digital_outputs[output_level_0_ - 1] != not output_logic_0_)
        {
          must_write_level_0 = true;
          write_do_level_0.request.output = output_level_0_;
          write_do_level_0.request.value = not output_logic_0_;
        }

        bool must_write_level_1 = false;
        robotnik_msgs::set_digital_output write_do_level_1;
        if (io_msg_.digital_outputs[output_level_1_ - 1] != not output_logic_1_)
        {
          must_write_level_1 = true;
          write_do_level_1.request.output = output_level_1_;
          write_do_level_1.request.value = not output_logic_1_;
        }

        if (must_write_level_0)
        {
          io_service_client_.call(write_do_level_0);
          ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name,
                                         component_name << "::readyState: area " << current_area << " - setting output "
                                                        << (int)write_do_level_0.request.output << " to "
                                                        << (int)write_do_level_0.request.value);
        }
        if (must_write_level_1)
        {
          io_service_client_.call(write_do_level_1);
          ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name,
                                         component_name << "::readyState: area " << current_area << " - setting output "
                                                        << (int)write_do_level_1.request.output << " to "
                                                        << (int)write_do_level_1.request.value);
        }

        if (must_write_level_0 or must_write_level_1)
          sleep(1);
      }
      break;

    case 1:
      if (current_linear_vel > velocity_level_1_ or (check_cmd and current_cmd_linear_vel > velocity_level_1_))
      {
        current_area = 2;
        ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area "
                                                                         << current_area
                                                                         << ". Vel = " << current_linear_vel);
      }
      else if (current_linear_vel < (velocity_level_0_ - hysteresis_level_0_) or
               (check_cmd and current_cmd_linear_vel < (velocity_level_0_ - hysteresis_level_0_)))
      {
        current_area = 0;
        ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area "
                                                                         << current_area
                                                                         << ". Vel = " << current_linear_vel);
      }
      else
      {
        bool must_write_level_0 = false;
        robotnik_msgs::set_digital_output write_do_level_0;
        if (io_msg_.digital_outputs[output_level_0_ - 1] != output_logic_0_)
        {
          must_write_level_0 = true;
          write_do_level_0.request.output = output_level_0_;
          write_do_level_0.request.value = output_logic_0_;
        }
        bool must_write_level_1 = false;
        robotnik_msgs::set_digital_output write_do_level_1;
        if (io_msg_.digital_outputs[output_level_1_ - 1] != not output_logic_1_)
        {
          must_write_level_1 = true;
          write_do_level_1.request.output = output_level_1_;
          write_do_level_1.request.value = not output_logic_1_;
        }
        if (must_write_level_0)
        {
          io_service_client_.call(write_do_level_0);
          ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name,
                                         component_name << "::readyState: area " << current_area << " - setting output "
                                                        << (int)write_do_level_0.request.output << " to "
                                                        << (int)write_do_level_0.request.value);
        }
        if (must_write_level_1)
        {
          io_service_client_.call(write_do_level_1);
          ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name,
                                         component_name << "::readyState: area " << current_area << " - setting output "
                                                        << (int)write_do_level_1.request.output << " to "
                                                        << (int)write_do_level_1.request.value);
        }
        if (must_write_level_0 or must_write_level_1)
          sleep(1);
      }
      break;

    case 2:
      if (current_linear_vel < (velocity_level_0_ - hysteresis_level_0_) or
          (check_cmd and current_cmd_linear_vel < (velocity_level_0_ - hysteresis_level_0_)))
      {
        current_area = 0;
        ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area "
                                                                         << current_area
                                                                         << ". Vel = " << current_linear_vel);
      }
      else if (current_linear_vel < (velocity_level_1_ - hysteresis_level_1_) or
               (check_cmd and current_cmd_linear_vel < (velocity_level_1_ - hysteresis_level_1_)))
      {
        current_area = 1;
        ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name, component_name << "::readyState: moving to area "
                                                                         << current_area
                                                                         << ". Vel = " << current_linear_vel);
      }
      else
      {
        bool must_write_level_0 = false;
        robotnik_msgs::set_digital_output write_do_level_0;
        if (io_msg_.digital_outputs[output_level_0_ - 1] != output_logic_0_)
        {
          must_write_level_0 = true;
          write_do_level_0.request.output = output_level_0_;
          write_do_level_0.request.value = output_logic_0_;
        }
        bool must_write_level_1 = false;
        robotnik_msgs::set_digital_output write_do_level_1;
        if (io_msg_.digital_outputs[output_level_1_ - 1] != output_logic_1_)
        {
          must_write_level_1 = true;
          write_do_level_1.request.output = output_level_1_;
          write_do_level_1.request.value = output_logic_1_;
        }
        if (must_write_level_0)
        {
          io_service_client_.call(write_do_level_0);
          ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name,
                                         component_name << "::readyState: area " << current_area << " - setting output "
                                                        << (int)write_do_level_0.request.output << " to "
                                                        << (int)write_do_level_0.request.value);
        }
        if (must_write_level_1)
        {
          io_service_client_.call(write_do_level_1);
          ROS_WARN_STREAM_THROTTLE_NAMED(1, component_name,
                                         component_name << "::readyState: area " << current_area << " - setting output "
                                                        << (int)write_do_level_1.request.output << " to "
                                                        << (int)write_do_level_1.request.value);
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
