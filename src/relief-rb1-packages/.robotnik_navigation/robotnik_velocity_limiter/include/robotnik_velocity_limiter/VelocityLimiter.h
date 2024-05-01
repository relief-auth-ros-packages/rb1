#ifndef _ROBOTNIK_VELOCITY_LIMITER_VELOCITY_LIMITER_
#define _ROBOTNIK_VELOCITY_LIMITER_VELOCITY_LIMITER_

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <sensor_msgs/LaserScan.h>

#include <rcomponent/rcomponent.h>

#include <robotnik_velocity_limiter/VelocityInterpolator.h>
#include <robotnik_velocity_limiter/RobotFootprint.h>

#include <laser_geometry/laser_geometry.h>

namespace robotnik_navigation
{
class VelocityLimiter : public rcomponent::RComponent
{
public:
  VelocityLimiter(ros::NodeHandle h);
  virtual ~VelocityLimiter();

protected:
  /* RComponent stuff */

  //! Setups all the ROS' stuff
  virtual int rosSetup();
  //! Shutdowns all the ROS' stuff
  virtual int rosShutdown();
  //! Reads data a publish several info into different topics
  virtual void rosPublish();
  //! Reads params from params server
  virtual void rosReadParams();

  virtual void standbyState();
  //! Actions performed on ready state
  virtual void readyState();
  //! Actions performed on the emergency state
  virtual void emergencyState();
  //! Actions performed on Failure state
  virtual void failureState();

  /* RComponent stuff !*/

protected:
  /* ROS stuff */

  //! Public node handle, to receive data
  ros::NodeHandle nh_;
  //! Private node hanlde, to read params and publish data
  ros::NodeHandle pnh_;

  //! The output command will be published by this publisher
  ros::Publisher cmd_vel_pub_;
  //! Command to be published
  geometry_msgs::Twist output_cmd_vel_;

  //! The input command will be read by this subscriber
  ros::Subscriber cmd_vel_sub_;
  //! Command read
  geometry_msgs::Twist input_cmd_vel_;

  //! To listen transformations
  tf::TransformListener* tf_listener_;

  //! Robot base frame, the origin of the robot. Typically base_fooprint
  std::string robot_base_frame_;

  //! Laser subscribers
  std::vector<ros::Subscriber> laser_sub_;
  //! Laser input topics
  std::vector<std::string> laser_inputs_;

  //! Closest point detected by inputs
  geometry_msgs::PointStamped angular_closest_point_;
  geometry_msgs::PointStamped front_closest_point_;
  geometry_msgs::PointStamped rear_closest_point_;
  //! Timeout
  ros::Duration closest_point_timeout_;

  RobotFootprint* angular_robot_footprint_;
  RobotFootprint* front_robot_footprint_;
  RobotFootprint* rear_robot_footprint_;

  //! To scale velocity between the limits
  // VelocityInterpolator * velocity_interpolator_;
  // It should be VelocityInterpolator, but dynamic cast is not working right now
  // so it would be changed in the future
  // TODO: change name to VelocityLinearInterpolator (as it is a linear interpolator, not an interpolator of only LinearVelocity)
  // LinearVelocityInterpolator* velocity_interpolator_;
  
  LinearVelocityInterpolator* angular_velocity_interpolator_;
  // front/rear means the position of obstacles that limit the velocity
  // forward/backward means which velocity is limited
  LinearVelocityInterpolator* front_forward_velocity_interpolator_;
  LinearVelocityInterpolator* front_backward_velocity_interpolator_;
  LinearVelocityInterpolator* rear_forward_velocity_interpolator_;
  LinearVelocityInterpolator* rear_backward_velocity_interpolator_;

  void laserScanCb(const sensor_msgs::LaserScan& scan);
  void cmdVelCb(const geometry_msgs::Twist& cmd);

  laser_geometry::LaserProjection laser_projector_;
  ros::Publisher log_pub_;
  /* VelocityLimiter stuff !*/
};
}  // namespace
#endif  // _ROBOTNIK_VELOCITY_LIMITER_VELOCITY_LIMITER_
