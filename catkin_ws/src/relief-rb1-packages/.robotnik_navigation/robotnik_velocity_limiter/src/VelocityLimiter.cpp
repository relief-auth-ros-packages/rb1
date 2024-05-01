#include <robotnik_velocity_limiter/VelocityLimiter.h>

#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <laser_geometry/laser_geometry.h>
#include <std_msgs/String.h>

namespace robotnik_navigation
{
VelocityLimiter::VelocityLimiter(ros::NodeHandle h) : RComponent(h), nh_(h), pnh_("~")
{
  component_name.assign(pnh_.getNamespace());

  rosReadParams();
}

VelocityLimiter::~VelocityLimiter()
{
}

void VelocityLimiter::rosReadParams()
{
  // RComponent::rosReadParams(); //not need to call it because it is called by the constructor of RComponent

  robot_base_frame_ = "base_footprint";
  readParam(pnh_, "robot_base_frame", robot_base_frame_, robot_base_frame_);
  
  laser_inputs_.clear();
  readParam(pnh_, "laser", laser_inputs_, laser_inputs_);
  
}

int VelocityLimiter::rosSetup()
{
  RComponent::rosSetup();

  // we need a buffer larger than the goal_averaging_time , otherwise the lookupTwist will fail
  double cache_time = tf::Transformer::DEFAULT_CACHE_TIME;

  tf_listener_ = new tf::TransformListener(ros::Duration(cache_time));

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel_out", 1);
  cmd_vel_sub_ = nh_.subscribe("cmd_vel_in", 1, &VelocityLimiter::cmdVelCb, this);

  log_pub_ = pnh_.advertise<std_msgs::String>("log", 1);
  
  laser_sub_.clear();
  for (std::string& laser_input : laser_inputs_)
  {
    laser_sub_.push_back(nh_.subscribe(laser_input, 1, &VelocityLimiter::laserScanCb, this));
  }

  angular_velocity_interpolator_ = new LinearVelocityInterpolator();
  front_forward_velocity_interpolator_ = new LinearVelocityInterpolator();
  front_backward_velocity_interpolator_ = new LinearVelocityInterpolator();
  rear_forward_velocity_interpolator_ = new LinearVelocityInterpolator();
  rear_backward_velocity_interpolator_ = new LinearVelocityInterpolator();

  /*** TODO: THIS PIECE OF CODE SHOULD BE REFACTORED ASAP (MOVED TO LinearVelocityInterpolator ***/
  double min_distance_, max_distance_;
  double vel_at_min_distance_, vel_at_max_distance_; 


// angular velocity
  min_distance_ = 0.1;
  max_distance_ = 1.0;
  readParam(pnh_, "interpolator/angular/min_distance", min_distance_, min_distance_);
  readParam(pnh_, "interpolator/angular/max_distance", max_distance_, max_distance_);
  vel_at_min_distance_ = 1.0;
  vel_at_max_distance_ = 1.0;
  readParam(pnh_, "interpolator/angular/vel_at_min_distance", vel_at_min_distance_, vel_at_min_distance_);
  readParam(pnh_, "interpolator/angular/vel_at_max_distance", vel_at_max_distance_, vel_at_max_distance_);
  
  angular_velocity_interpolator_->setMinimumDistance(min_distance_);
  angular_velocity_interpolator_->setMaximumDistance(max_distance_);

  geometry_msgs::Twist min_vel;
  min_vel.linear.x = vel_at_min_distance_;
  min_vel.angular.z = vel_at_min_distance_;
  angular_velocity_interpolator_->setVelocityAtMinimumDistance(min_vel);

  geometry_msgs::Twist max_vel;
  max_vel.linear.x = vel_at_max_distance_;
  max_vel.angular.z = vel_at_max_distance_;
  angular_velocity_interpolator_->setVelocityAtMaximumDistance(max_vel);
  

// front/forward velocity
  min_distance_ = 0.1;
  max_distance_ = 1.0;
  readParam(pnh_, "interpolator/front/forward/min_distance", min_distance_, min_distance_);
  readParam(pnh_, "interpolator/front/forward/max_distance", max_distance_, max_distance_);
  vel_at_min_distance_ = 1.0;
  vel_at_max_distance_ = 1.0;
  readParam(pnh_, "interpolator/front/forward/vel_at_min_distance", vel_at_min_distance_, vel_at_min_distance_);
  readParam(pnh_, "interpolator/front/forward/vel_at_max_distance", vel_at_max_distance_, vel_at_max_distance_);
  
  front_forward_velocity_interpolator_->setMinimumDistance(min_distance_);
  front_forward_velocity_interpolator_->setMaximumDistance(max_distance_);

  min_vel.linear.x = vel_at_min_distance_;
  min_vel.angular.z = vel_at_min_distance_;
  front_forward_velocity_interpolator_->setVelocityAtMinimumDistance(min_vel);

  max_vel.linear.x = vel_at_max_distance_;
  max_vel.angular.z = vel_at_max_distance_;
  front_forward_velocity_interpolator_->setVelocityAtMaximumDistance(max_vel);
 

// front/backward velocity
  min_distance_ = 0.1;
  max_distance_ = 1.0;
  readParam(pnh_, "interpolator/front/backward/min_distance", min_distance_, min_distance_);
  readParam(pnh_, "interpolator/front/backward/max_distance", max_distance_, max_distance_);
  vel_at_min_distance_ = 1.0;
  vel_at_max_distance_ = 1.0;
  readParam(pnh_, "interpolator/front/backward/vel_at_min_distance", vel_at_min_distance_, vel_at_min_distance_);
  readParam(pnh_, "interpolator/front/backward/vel_at_max_distance", vel_at_max_distance_, vel_at_max_distance_);
  
  front_backward_velocity_interpolator_->setMinimumDistance(min_distance_);
  front_backward_velocity_interpolator_->setMaximumDistance(max_distance_);

  min_vel.linear.x = vel_at_min_distance_;
  min_vel.angular.z = vel_at_min_distance_;
  front_backward_velocity_interpolator_->setVelocityAtMinimumDistance(min_vel);

  max_vel.linear.x = vel_at_max_distance_;
  max_vel.angular.z = vel_at_max_distance_;
  front_backward_velocity_interpolator_->setVelocityAtMaximumDistance(max_vel);
 
// rear/forward

  min_distance_ = 0.1;
  max_distance_ = 1.0;
  readParam(pnh_, "interpolator/rear/forward/min_distance", min_distance_, min_distance_);
  readParam(pnh_, "interpolator/rear/forward/max_distance", max_distance_, max_distance_);
  vel_at_min_distance_ = 1.0;
  vel_at_max_distance_ = 1.0;
  readParam(pnh_, "interpolator/rear/forward/vel_at_min_distance", vel_at_min_distance_, vel_at_min_distance_);
  readParam(pnh_, "interpolator/rear/forward/vel_at_max_distance", vel_at_max_distance_, vel_at_max_distance_);
  
  rear_forward_velocity_interpolator_->setMinimumDistance(min_distance_);
  rear_forward_velocity_interpolator_->setMaximumDistance(max_distance_);

  min_vel.linear.x = vel_at_min_distance_;
  min_vel.angular.z = vel_at_min_distance_;
  rear_forward_velocity_interpolator_->setVelocityAtMinimumDistance(min_vel);

  max_vel.linear.x = vel_at_max_distance_;
  max_vel.angular.z = vel_at_max_distance_;
  rear_forward_velocity_interpolator_->setVelocityAtMaximumDistance(max_vel);


  // rear/backward

  min_distance_ = 0.1;
  max_distance_ = 1.0;
  readParam(pnh_, "interpolator/rear/backward/min_distance", min_distance_, min_distance_);
  readParam(pnh_, "interpolator/rear/backward/max_distance", max_distance_, max_distance_);
  vel_at_min_distance_ = 1.0;
  vel_at_max_distance_ = 1.0;
  readParam(pnh_, "interpolator/rear/backward/vel_at_min_distance", vel_at_min_distance_, vel_at_min_distance_);
  readParam(pnh_, "interpolator/rear/backward/vel_at_max_distance", vel_at_max_distance_, vel_at_max_distance_);
  
  rear_backward_velocity_interpolator_->setMinimumDistance(min_distance_);
  rear_backward_velocity_interpolator_->setMaximumDistance(max_distance_);

  min_vel.linear.x = vel_at_min_distance_;
  min_vel.angular.z = vel_at_min_distance_;
  rear_backward_velocity_interpolator_->setVelocityAtMinimumDistance(min_vel);

  max_vel.linear.x = vel_at_max_distance_;
  max_vel.angular.z = vel_at_max_distance_;
  rear_backward_velocity_interpolator_->setVelocityAtMaximumDistance(max_vel);

  /*** END OF WHAT HAS TO BE REFACTORED ***/

  // this is the skeleton to make the velocity interpolator more configurable
  // it should use a dynamic cast, but is not working, so velocity_interpolator_
  // is of type LinearVelocityInterpolator, instead of VelocityInterpolator
  // velocity_interpolator_ = linear_velocity_interpolator;

  angular_robot_footprint_ = (RobotFootprint*)new CircularRobotFootprint();
  angular_robot_footprint_->setFrame(robot_base_frame_);

  CircularRobotFootprint * front_footprint = new CircularRobotFootprint();
  front_footprint->setFrame(robot_base_frame_);
  front_footprint->setCenterX(0.5);
  front_robot_footprint_ = (RobotFootprint*) front_footprint;
  
  CircularRobotFootprint * rear_footprint = new CircularRobotFootprint();
  rear_footprint->setFrame(robot_base_frame_);
  rear_footprint->setCenterX(-0.5);
  rear_robot_footprint_ = (RobotFootprint*) rear_footprint;

  closest_point_timeout_ = ros::Duration(0.5);
}

int VelocityLimiter::rosShutdown()
{
  RComponent::rosShutdown();
}

void VelocityLimiter::rosPublish()
{
  RComponent::rosPublish();
}

void VelocityLimiter::standbyState()
{
  switchToState(robotnik_msgs::State::READY_STATE);
}

void VelocityLimiter::readyState()
{
}

void VelocityLimiter::emergencyState()
{
}

void VelocityLimiter::failureState()
{
}

void VelocityLimiter::laserScanCb(const sensor_msgs::LaserScan& scan)
{
  if (!tf_listener_->waitForTransform(
          scan.header.frame_id, robot_base_frame_,
          scan.header.stamp + ros::Duration().fromSec(scan.ranges.size() * scan.time_increment), ros::Duration(1.0)))
  {
    RCOMPONENT_WARN_STREAM_THROTTLE(5, "Cannot transform from frame \"" << scan.header.frame_id << "\" to \""
                                                                        << robot_base_frame_ << "\"");
    return;
  }

  sensor_msgs::PointCloud cloud;
  laser_projector_.transformLaserScanToPointCloud(robot_base_frame_, scan, cloud, *tf_listener_);

  geometry_msgs::Point current_point;
  bool result;

  result = angular_robot_footprint_->closestPoint(cloud, current_point);

  if (true == result)
  {
    double current_squared_distance =
        current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z;
    double closest_squared_distance = angular_closest_point_.point.x * angular_closest_point_.point.x +
                                      angular_closest_point_.point.y * angular_closest_point_.point.y +
                                      angular_closest_point_.point.z * angular_closest_point_.point.z;

    if (current_squared_distance < closest_squared_distance or
        (ros::Time::now() - angular_closest_point_.header.stamp) > closest_point_timeout_)
    {
      /// if (current_squared_distance < closest_squared_distance)
      /// RCOMPONENT_INFO_STREAM("Replacing by distance");
      /// else
      /// RCOMPONENT_WARN_STREAM("Replacing by TIME");
      angular_closest_point_.point = current_point;
      angular_closest_point_.header = cloud.header;
    }
  }
  else
  {
    RCOMPONENT_WARN_STREAM_THROTTLE(5, "Closest point calculation went wrong.");
  }
  
  result = front_robot_footprint_->closestPoint(cloud, current_point);

  if (true == result)
  {
    double current_squared_distance =
        current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z;
    double closest_squared_distance = front_closest_point_.point.x * front_closest_point_.point.x +
                                      front_closest_point_.point.y * front_closest_point_.point.y +
                                      front_closest_point_.point.z * front_closest_point_.point.z;

    if (current_squared_distance < closest_squared_distance or
        (ros::Time::now() - front_closest_point_.header.stamp) > closest_point_timeout_)
    {
      /// if (current_squared_distance < closest_squared_distance)
      /// RCOMPONENT_INFO_STREAM("Replacing by distance");
      /// else
      /// RCOMPONENT_WARN_STREAM("Replacing by TIME");
      front_closest_point_.point = current_point;
      front_closest_point_.header = cloud.header;
    }
  }
  else
  {
    RCOMPONENT_WARN_STREAM_THROTTLE(5, "Closest point calculation went wrong.");
  }
  
result = rear_robot_footprint_->closestPoint(cloud, current_point);

  if (true == result)
  {
    double current_squared_distance =
        current_point.x * current_point.x + current_point.y * current_point.y + current_point.z * current_point.z;
    double closest_squared_distance = rear_closest_point_.point.x * rear_closest_point_.point.x +
                                      rear_closest_point_.point.y * rear_closest_point_.point.y +
                                      rear_closest_point_.point.z * rear_closest_point_.point.z;

    if (current_squared_distance < closest_squared_distance or
        (ros::Time::now() - rear_closest_point_.header.stamp) > closest_point_timeout_)
    {
      /// if (current_squared_distance < closest_squared_distance)
      /// RCOMPONENT_INFO_STREAM("Replacing by distance");
      /// else
      /// RCOMPONENT_WARN_STREAM("Replacing by TIME");
      rear_closest_point_.point = current_point;
      rear_closest_point_.header = cloud.header;
    }
  }
  else
  {
    RCOMPONENT_WARN_STREAM_THROTTLE(5, "Closest point calculation went wrong.");
  }
  // RCOMPONENT_WARN_STREAM("Closest: " << closest_point_);
}

bool compareTwist(const geometry_msgs::Twist& one, const geometry_msgs::Twist & other)
{
  return one.linear.x == other.linear.x and one.linear.y == other.linear.y and one.angular.z == other.angular.z;
}

void VelocityLimiter::cmdVelCb(const geometry_msgs::Twist& cmd)
{
  input_cmd_vel_ = cmd;

  if ((ros::Time::now() - angular_closest_point_.header.stamp).toSec() > 2 * closest_point_timeout_.toSec() or
      (ros::Time::now() - front_closest_point_.header.stamp).toSec() > 2 * closest_point_timeout_.toSec() or
      (ros::Time::now() - rear_closest_point_.header.stamp).toSec() > 2 * closest_point_timeout_.toSec())
  {
    bool publish_even_with_no_laser_ = true;

    if (true == publish_even_with_no_laser_)
    {
      RCOMPONENT_WARN_STREAM_THROTTLE(5, "Closest point has a timestamp older than timeout. I do not filter velocity");
      cmd_vel_pub_.publish(input_cmd_vel_);
    }
    else
    {
      RCOMPONENT_WARN_STREAM_THROTTLE(5, "Closest point has a timestamp older than timeout. I do not publish velocity");
    }
    return;
  }

  // RCOMPONENT_INFO_STREAM("Closest point: " << closest_point_.point);

  angular_velocity_interpolator_->interpolateVelocity(input_cmd_vel_, angular_closest_point_.point, output_cmd_vel_);
  if (compareTwist(input_cmd_vel_, output_cmd_vel_) == true)
  {
    std_msgs::String msg;
    msg.data = "a  . from (" + std::to_string(input_cmd_vel_.linear.x) + ", " + std::to_string(input_cmd_vel_.angular.z) + ") to (" + std::to_string(output_cmd_vel_.linear.x) + ", " + std::to_string(output_cmd_vel_.angular.z) + "). point (" + std::to_string(angular_closest_point_.point.x) + ", " + std::to_string(angular_closest_point_.point.y) + ")";
    log_pub_.publish(msg);
  }  
  //ROS_INFO("angular: %f output: %f", input_cmd_vel_.linear.x,output_cmd_vel_.linear.x);
  if (output_cmd_vel_.linear.x > 0)
  {
     geometry_msgs::Twist input_linear = output_cmd_vel_;
     geometry_msgs::Twist temp_cmd_vel;
     front_forward_velocity_interpolator_->interpolateVelocity(input_linear, front_closest_point_.point, temp_cmd_vel);

    std_msgs::String msg;
    msg.data = "f f. from (" + std::to_string(input_linear.linear.x) + ", " + std::to_string(input_linear.angular.z) + ") to (" + std::to_string(temp_cmd_vel.linear.x) + ", " + std::to_string(temp_cmd_vel.angular.z) + "). point (" + std::to_string(front_closest_point_.point.x) + ", " + std::to_string(front_closest_point_.point.y) + ")";
    log_pub_.publish(msg);

//     ROS_INFO("front forward: %f output: %f", input_linear.linear.x, temp_cmd_vel.linear.x);
     rear_forward_velocity_interpolator_->interpolateVelocity(temp_cmd_vel, rear_closest_point_.point, output_cmd_vel_);
    msg.data = "f r. from (" + std::to_string(temp_cmd_vel.linear.x) + ", " + std::to_string(temp_cmd_vel.angular.z) + ") to (" + std::to_string(output_cmd_vel_.linear.x) + ", " + std::to_string(output_cmd_vel_.angular.z) + "). point (" + std::to_string(rear_closest_point_.point.x) + ", " + std::to_string(rear_closest_point_.point.y) + ")";
    log_pub_.publish(msg);
//     ROS_INFO("rear forward: %f output: %f", temp_cmd_vel.linear.x, output_cmd_vel_.linear.x);
  }
  else
  {
     geometry_msgs::Twist input_linear = output_cmd_vel_;
     geometry_msgs::Twist temp_cmd_vel;
     input_linear.linear.x =  -input_linear.linear.x;
     input_linear.linear.y =  -input_linear.linear.y;
     input_linear.angular.z = -input_linear.angular.z;
     
     rear_backward_velocity_interpolator_->interpolateVelocity(input_linear, rear_closest_point_.point, temp_cmd_vel);
     std_msgs::String msg;
    msg.data = "b r. from (" + std::to_string(input_linear.linear.x) + ", " + std::to_string(input_linear.angular.z) + ") to (" + std::to_string(temp_cmd_vel.linear.x) + ", " + std::to_string(temp_cmd_vel.angular.z) + "). point (" + std::to_string(rear_closest_point_.point.x) + ", " + std::to_string(rear_closest_point_.point.y) + ")";
    log_pub_.publish(msg);




//     ROS_INFO("rear backward: %f output: %f", input_linear.linear.x, temp_cmd_vel.linear.x);
     front_backward_velocity_interpolator_->interpolateVelocity(temp_cmd_vel, front_closest_point_.point, output_cmd_vel_);
    msg.data = "b f. from (" + std::to_string(temp_cmd_vel.linear.x) + ", " + std::to_string(temp_cmd_vel.angular.z) + ") to (" + std::to_string(output_cmd_vel_.linear.x) + ", " + std::to_string(output_cmd_vel_.angular.z) + "). point (" + std::to_string(front_closest_point_.point.x) + ", " + std::to_string(front_closest_point_.point.y) + ")";
    log_pub_.publish(msg);
//     ROS_INFO("front backward: %f output: %f", temp_cmd_vel.linear.x,output_cmd_vel_.linear.x);
     output_cmd_vel_.linear.x = -output_cmd_vel_.linear.x;
     output_cmd_vel_.linear.y = -output_cmd_vel_.linear.y;
     output_cmd_vel_.angular.z = -output_cmd_vel_.angular.z;
  }

  if (std::isnan(output_cmd_vel_.linear.x) or std::isnan(output_cmd_vel_.linear.y) or
      std::isnan(output_cmd_vel_.angular.z))
  {
    RCOMPONENT_WARN_STREAM_THROTTLE(5, "Computed a command with nan. Do not publishing it");
    return;
  }
  cmd_vel_pub_.publish(output_cmd_vel_);
}

}  // namespace
