#ifndef _ROBOTNIK_VELOCITY_LIMITER_ROBOT_FOOTPRINT_
#define _ROBOTNIK_VELOCITY_LIMITER_ROBOT_FOOTPRINT_

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <costmap_2d/footprint.h>

#include <robotnik_velocity_limiter/utils.h>

namespace robotnik_navigation
{
class RobotFootprint
{
public:
  RobotFootprint()
  {
  }

  virtual bool init(const ros::NodeHandle &nh) = 0;

  void setFrame(const std::string& frame)
  {
    frame_ = frame;
  }

  bool closestPoint(const sensor_msgs::LaserScan& scan, geometry_msgs::Point& point)
  {
    if (scan.header.frame_id != frame_)
    {
      ROS_WARN("RobotFootprint: frames do not match. Skipping");
      point.x = point.y = point.z = std::numeric_limits<double>::quiet_NaN();
      return false;
    }
    return closestPointImpl(scan, point);
  }

  bool closestPoint(const sensor_msgs::PointCloud& cloud, geometry_msgs::Point& point)
  {
    if (cloud.header.frame_id != frame_)
    {
      ROS_WARN_THROTTLE(5, "RobotFootprint: frames do not match. Skipping");
      return false;
    }
    bool r = closestPointImpl(cloud, point);

    //** ROS_INFO_STREAM("RF" << point);
    return r;
  }

  virtual bool closestPointImpl(const sensor_msgs::LaserScan& scan, geometry_msgs::Point& point) = 0;
  virtual bool closestPointImpl(const sensor_msgs::PointCloud& cloud, geometry_msgs::Point& point) = 0;

protected:
  std::string frame_;
};

// class PolygonRobotFootprint : public RobotFootprint
// {
// public:
//   PolygonRobotFootprint()
//   {
//   }
//   
//   virtual bool init(const ros::NodeHandle &nh)
//   {
//      if (nh.hasParam("footprint") == false)
//      {
//         ROS_WARN_STREAM("No parameter \"" << nh.resolveName("footprint") << "\", I cannot build a polygon.");
//         return false;
//      }
//      
//      polygon_ = costmap_2d::makeFootprintFromParams(nh);
//      return true;
//   }
// private:
//  
//   std::vector<geometry_msgs::Point> polygon_;
// };

class CircularRobotFootprint : public RobotFootprint
{
public:
  CircularRobotFootprint()
  {
    radius_ = 0;
    center_x_ = 0;
    center_y_ = 0;
  }

  virtual bool init(const ros::NodeHandle &nh)
  {
    radius_ = 0;
    readParam(nh, "radius", radius_, radius_);
    center_x_ = 0;
    readParam(nh, "center_x", center_x_, center_x_);
    center_y_ = 0;
    readParam(nh, "center_y", center_y_, center_y_);
  }
  
  void setRadius(const double radius)
  {
    radius_ = radius;
  }
  
  void setCenterX(const double x)
  {
    center_x_ = x;
  }
 
  void setCenterY(const double y)
  {
    center_y_ = y;
  }

  virtual bool closestPointImpl(const sensor_msgs::LaserScan& scan, geometry_msgs::Point& point)
  {
    ROS_WARN_THROTTLE(5, "RobotFootprint: not implemented for LaserScan");
    point.x = point.y = point.z = std::numeric_limits<double>::quiet_NaN();
    return false;
  }

  virtual bool closestPointImpl(const sensor_msgs::PointCloud& cloud, geometry_msgs::Point& closest_point)
  {
    closest_point.x = closest_point.y = closest_point.z = std::numeric_limits<double>::quiet_NaN();
    double closest_squared_distance = std::numeric_limits<double>::infinity();
    int idx = 0;
    int max = 0;
    for (geometry_msgs::Point32 point : cloud.points)
    {
      max++;
      double squared_distance = (point.x - center_x_) * (point.x - center_x_) + (point.y - center_y_) * (point.y - center_y_) + point.z * point.z;
      if (squared_distance < closest_squared_distance)
      {
        closest_squared_distance = squared_distance;
        closest_point.x = point.x - center_x_;
        closest_point.y = point.y - center_y_;
        closest_point.z = point.z;
        idx = max;
      }
    }
    //** ROS_INFO("id %d max %d", idx, max);
    //** ROS_INFO_STREAM("CRF " << closest_point);
    return true;
  }

protected:
  double radius_;
  double center_x_;
  double center_y_;
};
}

#endif  // _ROBOTNIK_VELOCITY_LIMITER_ROBOT_FOOTPRINT_
