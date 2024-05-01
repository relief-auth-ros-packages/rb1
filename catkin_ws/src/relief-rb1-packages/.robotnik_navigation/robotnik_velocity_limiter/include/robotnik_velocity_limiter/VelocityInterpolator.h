#ifndef _ROBOTNIK_VELOCITY_LIMITER_VELOCITY_INTERPOLATOR_
#define _ROBOTNIK_VELOCITY_LIMITER_VELOCITY_INTERPOLATOR_

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

namespace robotnik_navigation
{
class VelocityInterpolator
{
public:
  VelocityInterpolator()
  {
  }
  virtual ~VelocityInterpolator()
  {
  }
  // closest point is closest point to footprint, not to base_link
  virtual void interpolateVelocity(const geometry_msgs::Twist& input, const geometry_msgs::Point& closest_point,
                                   geometry_msgs::Twist& output) = 0;
};

class LinearVelocityInterpolator : public VelocityInterpolator
{
public:
  LinearVelocityInterpolator()
  {
  }

  // closest point is closest point to footprint, not to base_link
  void interpolateVelocity(const geometry_msgs::Twist& input, const geometry_msgs::Point& closest_point,
                           geometry_msgs::Twist& output)
  {
    double distance = std::sqrt(closest_point.x * closest_point.x + closest_point.y * closest_point.y);
    if (distance > max_distance_)
    {
      ROS_INFO_STREAM_THROTTLE(5, "closest distance is greater than max, do not interpolate: " << distance << max_distance_);
      // then do not interpolate
      output = input;
      return;
    }
    double distance_ratio = (distance - min_distance_) / (max_distance_ - min_distance_);
    //ROS_INFO_STREAM("distance: " << distance << " min " << min_distance_ << " max " << max_distance_ << " distance_ratio " << distance_ratio);
    //ROS_INFO_STREAM("closest: " << closest_point.x << " " << closest_point.y);
    if (distance_ratio < 0)
      distance_ratio = 0;

    geometry_msgs::Twist vel_step;
    vel_step.linear.x = std::abs(vel_at_max_distance_.linear.x - vel_at_min_distance_.linear.x);
    vel_step.linear.y = std::abs(vel_at_max_distance_.linear.y - vel_at_min_distance_.linear.y);
    vel_step.angular.z = std::abs(vel_at_max_distance_.angular.z - vel_at_min_distance_.angular.z);

    //ROS_INFO_STREAM("vel_step: " << vel_step);

    geometry_msgs::Twist max_vel_allowed;
    max_vel_allowed.linear.x = vel_at_min_distance_.linear.x + distance_ratio * vel_step.linear.x;
    max_vel_allowed.linear.y = vel_at_min_distance_.linear.y + distance_ratio * vel_step.linear.y;
    max_vel_allowed.angular.z = vel_at_min_distance_.angular.z + distance_ratio * vel_step.angular.z;

    //ROS_INFO_STREAM("max_vel_allowed: " << max_vel_allowed);

    geometry_msgs::Twist scaled_vel;
    scaled_vel.linear.x = (max_vel_allowed.linear.x == 0) ? 0 : std::abs(input.linear.x / max_vel_allowed.linear.x);
    scaled_vel.linear.y = (max_vel_allowed.linear.y == 0) ? 0 : std::abs(input.linear.x / max_vel_allowed.linear.y);
    scaled_vel.angular.z = (max_vel_allowed.angular.z == 0) ? 0 : std::abs(input.angular.z / max_vel_allowed.angular.z);

    //ROS_INFO_STREAM("scaled_vel: " << scaled_vel);

    double scale = std::max(std::max(scaled_vel.linear.x, scaled_vel.linear.y), scaled_vel.angular.z);

    //ROS_INFO_STREAM("selected scale " << scale);
    output = input;

    if (scale == 0)
    {
      // if scale is 0, that means we are in the inner area and the maximum velocity there is 0
      output.linear.x = output.linear.y = output.angular.z = 0.0;
    }
    else if (scale > 1)
    {
      // if one of the velocities has an scale greater than 1, that means input was greater than allowed
      output.linear.x = output.linear.x / scale;
      output.linear.y = output.linear.y / scale;
      output.angular.z = output.angular.z / scale;
    }
    //ROS_INFO_STREAM("input" << input);
    //ROS_INFO_STREAM("output" << output);

    return;
  }

  void setMinimumDistance(const double d)
  {
    min_distance_ = d;
  }
  void setMaximumDistance(const double d)
  {
    max_distance_ = d;
  }
  void setVelocityAtMinimumDistance(const geometry_msgs::Twist& vel)
  {
    vel_at_min_distance_ = vel;
  }
  void setVelocityAtMaximumDistance(const geometry_msgs::Twist& vel)
  {
    vel_at_max_distance_ = vel;
  }

protected:
  geometry_msgs::Twist vel_at_min_distance_;
  geometry_msgs::Twist vel_at_max_distance_;
  double min_distance_;
  double max_distance_;
};
}  // namespace

#endif  // _ROBOTNIK_VELOCITY_LIMITER_VELOCITY_INTERPOLATOR_
