#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
//#include <robotnik_safety_controller/PointStampedArray.h>
#include <tf/transform_listener.h>
#include <string>
//#include <robot_local_control_msgs/Status.h>

#define TOPIC_TIMEOUT 1.0

typedef struct roi
{
  double min_x;
  double max_x;
  double min_y;
  double max_y;
} collsion_area;

sensor_msgs::LaserScan laser_msg;
sensor_msgs::LaserScan laser2_msg;
//robot_local_control_msgs::Status state_msg;
//std::string mode_msg;
ros::Subscriber laser_subscriber;
ros::Subscriber laser2_subscriber;
ros::Subscriber state_subscriber;
ros::Subscriber cmd_vel_sub_;
ros::Publisher pub_collision_roi;
ros::Publisher pub_warning_collision_roi;
ros::Publisher pub_collision_point;
ros::Publisher pub_collision_points_vector;
ros::Publisher pub_collision_points_bool;
ros::Publisher pub_warning_collision_point;
ros::Publisher pub_warning_points_vector;
ros::Publisher pub_warning_points_bool;
ros::Publisher cmd_vel_pub_;
geometry_msgs::Twist cmd_vel_in_msg_;
ros::Time cmd_vel_in_last_time_;
bool cmd_vel_published_;
tf::TransformListener* pListener = NULL;
std::string base_frame;
// robotnik_safety_controller::PointStampedArray global_collision_points;
// robotnik_safety_controller::PointStampedArray global_warning_points;

std::vector<geometry_msgs::PointStamped> global_collision_points;
std::vector<geometry_msgs::PointStamped> global_warning_points;

double desired_freq = 1.0;
double max_warning_speed = 0.5;
std::string cmd_vel_in_topic;
std::string cmd_vel_out_topic;
std::string safety_controller_mode;

collsion_area collision_roi_;
int minimum_points_for_detection_;
collsion_area warning_roi_;
// TODO: Get robot footprint from description
double robot_x_size = 10.0;
double robot_y_size = 10.0;
double robot_radius = 10.0;
geometry_msgs::PointStamped global_collision_point;
std_msgs::Bool collision_point_bool;
std_msgs::Bool warning_point_bool;
bool allow_only_rotation;
ros::Duration period_to_log(10);

void frontLaserCb(const sensor_msgs::LaserScanConstPtr& message)
{
  // Save laser msg
  laser_msg = *message;
}
void backLaserCb(const sensor_msgs::LaserScanConstPtr& message)
{
  // Save laser msg
  laser2_msg = *message;
}

/*
void stateCb(const robot_local_control_msgs::StatusPtr& message)
{
  // Save laser msg
  state_msg = *message;
  mode_msg = state_msg.control_state;
}
*/

void cmdVelCb(const geometry_msgs::TwistConstPtr& message)
{
  cmd_vel_in_last_time_ = ros::Time::now();

  // Save Twist msg
  cmd_vel_in_msg_ = *message;
  cmd_vel_published_ = false;
  // std::cout << "Entro al callback" << std::endl;
  ROS_INFO_THROTTLE(period_to_log.toSec(), "safety: command received");
}

bool collisionDetected()
{
  geometry_msgs::PointStamped collision_point, source_point;
  global_collision_points.clear();

  for (int i = 0; i < laser_msg.ranges.size(); i++)
  {
    if (laser_msg.ranges[i] < laser_msg.range_min || laser_msg.ranges[i] > laser_msg.range_max)
      continue;

    double angle = laser_msg.angle_min + i * laser_msg.angle_increment;
    double x = laser_msg.ranges[i] * cos(angle);
    double y = laser_msg.ranges[i] * sin(angle);

    source_point.header = laser_msg.header;
    source_point.point.x = x;
    source_point.point.y = y;
    source_point.point.z = 0.0;

    try
    {
      pListener->transformPoint(base_frame, source_point, collision_point);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_THROTTLE(period_to_log.toSec(), "Transform error: %s", ex.what());
    }

    // Check if scan is inside the collision ROI
    if (collision_roi_.min_x < collision_point.point.x && collision_point.point.x < collision_roi_.max_x &&
        collision_roi_.min_y < collision_point.point.y && collision_point.point.y < collision_roi_.max_y)
    {
      global_collision_points.push_back(collision_point);
      pub_collision_point.publish(collision_point);
    }
  }

  // second laser
  for (int i = 0; i < laser2_msg.ranges.size(); i++)
  {
    if (laser2_msg.ranges[i] < laser2_msg.range_min || laser2_msg.ranges[i] > laser2_msg.range_max)
      continue;

    double angle = laser2_msg.angle_min + i * laser2_msg.angle_increment;
    double x = laser2_msg.ranges[i] * cos(angle);
    double y = laser2_msg.ranges[i] * sin(angle);

    source_point.header = laser2_msg.header;
    source_point.point.x = x;
    source_point.point.y = y;
    source_point.point.z = 0.0;

    try
    {
      pListener->transformPoint(base_frame, source_point, collision_point);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_THROTTLE(period_to_log.toSec(),"Transform error: %s", ex.what());
    }

    // Check if scan is inside the collision ROI
    if (collision_roi_.min_x < collision_point.point.x && collision_point.point.x < collision_roi_.max_x &&
        collision_roi_.min_y < collision_point.point.y && collision_point.point.y < collision_roi_.max_y)
    {
      global_collision_points.push_back(collision_point);
      pub_collision_point.publish(collision_point);
    }
  }

  // There are no collision points detected
  if (global_collision_points.size() < minimum_points_for_detection_)
  {
    if (global_collision_points.size() != 0)
    {
      ROS_WARN_STREAM_THROTTLE(period_to_log.toSec(), "safety: i am detecting " << global_collision_points.size()
                                                            << " inside the collision area, but I need at least "
                                                            << minimum_points_for_detection_ << " to stop");
    }
    if (collision_point_bool.data == true)
    {
      collision_point_bool.data = false;
      pub_collision_points_bool.publish(collision_point_bool);
    }
    return false;
  }
  // There is at least one collision point detected
  else
  {
    if (collision_point_bool.data == false)
    {
      collision_point_bool.data = true;
      pub_collision_points_bool.publish(collision_point_bool);
    }
    //    pub_collision_points_vector.publish(global_collision_points);
    return true;
  }
}

bool warningDetected()
{
  geometry_msgs::PointStamped source_point, warning_point;
  global_warning_points.clear();

  for (int i = 0; i < laser_msg.ranges.size(); i++)
  {
    if (laser_msg.ranges[i] < laser_msg.range_min || laser_msg.ranges[i] > laser_msg.range_max)
      continue;

    double angle = laser_msg.angle_min + i * laser_msg.angle_increment;
    double x = laser_msg.ranges[i] * cos(angle);
    double y = laser_msg.ranges[i] * sin(angle);

    source_point.header = laser_msg.header;
    source_point.point.x = x;
    source_point.point.y = y;
    source_point.point.z = 0.0;

    try
    {
      pListener->transformPoint(base_frame, source_point, warning_point);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_THROTTLE(period_to_log.toSec(), "Error al transformar: %s", ex.what());
    }

    // Check if scan is inside the collision ROI
    if (warning_roi_.min_x < warning_point.point.x && warning_point.point.x < warning_roi_.max_x &&
        warning_roi_.min_y < warning_point.point.y && warning_point.point.y < warning_roi_.max_y)
    {
      pub_warning_collision_point.publish(warning_point);
      global_warning_points.push_back(warning_point);
    }
  }

  // laser 2
  for (int i = 0; i < laser2_msg.ranges.size(); i++)
  {
    if (laser2_msg.ranges[i] < laser2_msg.range_min || laser2_msg.ranges[i] > laser2_msg.range_max)
      continue;

    double angle = laser2_msg.angle_min + i * laser2_msg.angle_increment;
    double x = laser2_msg.ranges[i] * cos(angle);
    double y = laser2_msg.ranges[i] * sin(angle);

    source_point.header = laser2_msg.header;
    source_point.point.x = x;
    source_point.point.y = y;
    source_point.point.z = 0.0;

    try
    {
      pListener->transformPoint(base_frame, source_point, warning_point);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_THROTTLE(period_to_log.toSec(), "Error al transformar: %s", ex.what());
    }

    // Check if scan is inside the collision ROI
    if (warning_roi_.min_x < warning_point.point.x && warning_point.point.x < warning_roi_.max_x &&
        warning_roi_.min_y < warning_point.point.y && warning_point.point.y < warning_roi_.max_y)
    {
      pub_warning_collision_point.publish(warning_point);
      global_warning_points.push_back(warning_point);
    }
  }

  // There are no warning points detected
  if (global_warning_points.size() < minimum_points_for_detection_)
  {
    if (global_warning_points.size() != 0)
    {
      ROS_WARN_STREAM_THROTTLE(period_to_log.toSec(), "safety: i am detecting " << global_warning_points.size()
                                                            << " inside the warning area, but I need at least "
                                                            << minimum_points_for_detection_ << " to stop");
    }
    if (warning_point_bool.data == true)
    {
      warning_point_bool.data = false;
      pub_warning_points_bool.publish(warning_point_bool);
    }
    return false;
  }
  // There is at least one collision point detected
  else
  {
    if (warning_point_bool.data == false)
    {
      warning_point_bool.data = true;
      pub_warning_points_bool.publish(warning_point_bool);
    }
    // pub_warning_points_vector.publish(global_warning_points);
    return true;
  }
}

void publishROI()
{
  geometry_msgs::PolygonStamped polygon_msg;
  polygon_msg.header.frame_id = base_frame;
  geometry_msgs::Point32 point1, point2, point3, point4;

  // Define collision and warning area around base_link
  point1.x = collision_roi_.max_x;
  point1.y = collision_roi_.max_y;
  point2.x = collision_roi_.max_x;
  point2.y = collision_roi_.min_y;
  point3.x = collision_roi_.min_x;
  point3.y = collision_roi_.min_y;
  point4.x = collision_roi_.min_x;
  point4.y = collision_roi_.max_y;
  polygon_msg.polygon.points.push_back(point1);
  polygon_msg.polygon.points.push_back(point2);
  polygon_msg.polygon.points.push_back(point3);
  polygon_msg.polygon.points.push_back(point4);
  pub_collision_roi.publish(polygon_msg);

  polygon_msg.polygon.points.clear();
  point1.x = warning_roi_.max_x;
  point1.y = warning_roi_.max_y;
  point2.x = warning_roi_.max_x;
  point2.y = warning_roi_.min_y;
  point3.x = warning_roi_.min_x;
  point3.y = warning_roi_.min_y;
  point4.x = warning_roi_.min_x;
  point4.y = warning_roi_.max_y;
  polygon_msg.polygon.points.push_back(point1);
  polygon_msg.polygon.points.push_back(point2);
  polygon_msg.polygon.points.push_back(point3);
  polygon_msg.polygon.points.push_back(point4);
  pub_warning_collision_roi.publish(polygon_msg);
}

bool isRotationAllowed()
{
  /* for (int i = 0; i < global_collision_points.size(); i++)
   {
     if (pow(global_collision_points[i].point.x, 2) + pow(global_collision_points[i].point.y, 2) < robot_radius)
     {
       //std::cout << "No rotation allowed" << std::endl;
       ROS_WARN_THROTTLE(period_to_log.toSec(), "safety: no rotation allowed");
       return false;
     }
   }*/
  return allow_only_rotation;
}

std::set<int> occupiedAreas()
{
  // std::cout << "The following areas are occupied: " << std::endl;
  std::set<int> occupied_areas;
  for (int i = 0; i < global_collision_points.size(); i++)
  {
    if (global_collision_points[i].point.x > robot_x_size/2.0)
    {
      if (global_collision_points[i].point.y < -robot_y_size/2.0)
      {
        occupied_areas.insert(1);
      }
      else if (global_collision_points[i].point.y > robot_y_size/2.0)
      {
        occupied_areas.insert(3);
      }
      else
      {
        occupied_areas.insert(2);
      }
    }
    else if (-robot_x_size/2.0 < global_collision_points[i].point.x && global_collision_points[i].point.x < robot_x_size/2.0)
    {
      if (global_collision_points[i].point.y < -robot_y_size/2.0)
      {
        occupied_areas.insert(4);
      }
      else if (global_collision_points[i].point.y > robot_y_size/2.0)
      {
        occupied_areas.insert(6);
      }
      else
      {
        occupied_areas.insert(5);
      }
    }
    else if (global_collision_points[i].point.x < -robot_x_size/2.0)
    {
      if (global_collision_points[i].point.y < -robot_y_size/2.0)
      {
        occupied_areas.insert(7);
      }
      else if (global_collision_points[i].point.y > robot_y_size/2.0)
      {
        occupied_areas.insert(9);
      }
      else
      {
        occupied_areas.insert(8);
      }
    }
  }
  std::set<int>::iterator it;
  std::string areas = "";
  for (it = occupied_areas.begin(); it != occupied_areas.end(); ++it)
  {
    areas += std::to_string(*it) + " ";
  }
  if (occupied_areas.size() > 0) {
    ROS_INFO_STREAM_THROTTLE(period_to_log.toSec(), "safety: occupied areas " << areas);
  }
  return occupied_areas;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robotnik_safety_controller");

  ros::NodeHandle node;
  ros::NodeHandle pnh_("~");

  // Read ROI params

  // ALL PARAMETERS SHOULD BE READ AND IF NOT FOUND; SHOW A WARNING 
  pnh_.param("min_x", collision_roi_.min_x, -0.5);
  pnh_.param("max_x", collision_roi_.max_x, 0.5);
  pnh_.param("min_y", collision_roi_.min_y, -0.4);
  pnh_.param("max_y", collision_roi_.max_y, 0.4);
  pnh_.param("warning_min_x", warning_roi_.min_x, -1.0);
  pnh_.param("warning_max_x", warning_roi_.max_x, 1.0);
  pnh_.param("warning_min_y", warning_roi_.min_y, -0.8);
  pnh_.param("warning_max_y", warning_roi_.max_y, 0.8);
  pnh_.param("max_warning_speed", max_warning_speed, 0.5);
  pnh_.param("desired_freq", desired_freq, 10.0);
  pnh_.param("minimum_points_for_detection", minimum_points_for_detection_, 5);
  pnh_.param<std::string>("safety_controller_mode", safety_controller_mode, "");
  pnh_.param<std::string>("cmd_vel_in", cmd_vel_in_topic, "cmd_vel_in");
  pnh_.param<std::string>("cmd_vel_out", cmd_vel_out_topic, "cmd_vel_out");
  pnh_.param<std::string>("base_frame", base_frame, "base_footprint");
  pnh_.param<bool>("allow_only_rotation", allow_only_rotation, false);

  if (pnh_.param<double>("robot_x_size", robot_x_size, robot_x_size) == false)
  {
     ROS_ERROR_STREAM("No parameter robot_x_size found, I need the size of the robot to operate, using default value of " << robot_x_size);
  }
  
  if (pnh_.param<double>("robot_y_size", robot_y_size, robot_y_size) == false)
  {
     ROS_ERROR_STREAM("No parameter robot_y_size found, I need the size of the robot to operate, using default value of " << robot_y_size);
  }

  cmd_vel_pub_ = node.advertise<geometry_msgs::Twist>(cmd_vel_out_topic, 10);
  
  laser_subscriber = node.subscribe("front_laser/scan", 1, frontLaserCb);
  laser2_subscriber = node.subscribe("rear_laser/scan", 1, backLaserCb);
  //state_subscriber = node.subscribe("robot_local_control/state", 1, stateCb);
  cmd_vel_published_ = false;
  cmd_vel_sub_ = node.subscribe(cmd_vel_in_topic, 1, cmdVelCb);
  pub_collision_roi = pnh_.advertise<geometry_msgs::PolygonStamped>("collision_roi", 1);
  pub_warning_collision_roi = pnh_.advertise<geometry_msgs::PolygonStamped>("warning_collision_roi", 1);
  pub_collision_point = pnh_.advertise<geometry_msgs::PointStamped>("collision_point", 1);
  //  pub_collision_points_vector =
  //      pnh_.advertise<robotnik_safety_controller::PointStampedArray>("collision_points_vector", 1);
  pub_collision_points_bool = pnh_.advertise<std_msgs::Bool>("collision_points_bool", 1);
  pub_warning_collision_point = pnh_.advertise<geometry_msgs::PointStamped>("warning_collision_point", 1);
  //  pub_warning_points_vector = pnh_.advertise<robotnik_safety_controller::PointStampedArray>("warning_points_vector",
  //  1);
  pub_warning_points_bool = pnh_.advertise<std_msgs::Bool>("warning_points_bool", 1);

  pListener = new (tf::TransformListener);

  bool use_laser_ = true;
  bool use_pointcloud_ = false;
  ros::Rate rate(desired_freq);
  collision_point_bool.data = false;
  warning_point_bool.data = false;

  while (node.ok())
  {
    bool publish_cmd_vel = true;

    publishROI();

    // Check if we have an available scan msg
    if (use_laser_ == true and (ros::Time::now() - laser_msg.header.stamp) > ros::Duration(TOPIC_TIMEOUT))
    {
      ROS_WARN_THROTTLE(period_to_log.toSec(), "robotnik_safety_controller: Can't read laser msg, or it is too old");
      publish_cmd_vel = false;
    }

    // Check if we have an available twist msg
    if (use_pointcloud_ == true and (ros::Time::now() - cmd_vel_in_last_time_) > ros::Duration(TOPIC_TIMEOUT))
    {
      ROS_WARN_THROTTLE(period_to_log.toSec(), "robotnik_safety_controller: Can't read Twist msg, or it is too old");
      publish_cmd_vel = false;
    }

    bool collision = collisionDetected();
    bool warning = warningDetected();
    geometry_msgs::Twist cmd_vel_out_msg;

    if (publish_cmd_vel)
    {
      // Limit angular speed

      cmd_vel_out_msg = cmd_vel_in_msg_;
      // std::cout << "Input velocity is: " << cmd_vel_in_msg_.linear.x << ", " << cmd_vel_in_msg_.linear.y
      //          << cmd_vel_in_msg_.angular.z << std::endl;

      if (collision)
      {
        std::set<int> occupied_areas = occupiedAreas();
        bool is_rotation_allowed = isRotationAllowed();
        if (!is_rotation_allowed)
        {
          cmd_vel_out_msg.angular.z = 0.0;
        }
        // Obstacle into robot footprint
        if (occupied_areas.find(5) != occupied_areas.end() and
            cmd_vel_out_msg.linear.x != 0)  // but we can move to the rear
        {
          cmd_vel_out_msg.linear.x = 0.0;
          cmd_vel_out_msg.linear.y = 0.0;
          cmd_vel_out_msg.angular.z = 0.0;
        }
        if (occupied_areas.find(2) != occupied_areas.end())
        {
          cmd_vel_out_msg.linear.x = std::min(cmd_vel_out_msg.linear.x, 0.0);
        }
        if (occupied_areas.find(8) != occupied_areas.end())
        {
          cmd_vel_out_msg.linear.x = std::max(cmd_vel_out_msg.linear.x, 0.0);
        }
        if (occupied_areas.find(4) != occupied_areas.end())
        {
          cmd_vel_out_msg.linear.y = std::max(cmd_vel_out_msg.linear.y, 0.0);
        }
        if (occupied_areas.find(6) != occupied_areas.end())
        {
          cmd_vel_out_msg.linear.y = std::min(cmd_vel_out_msg.linear.y, 0.0);
        }
        // Diagonal areas
        if (occupied_areas.find(1) != occupied_areas.end())
        {
          if (cmd_vel_out_msg.linear.x > 0.0 && cmd_vel_out_msg.linear.y < 0.0)
          {
            cmd_vel_out_msg.linear.x = 0.0;
            cmd_vel_out_msg.linear.y = 0.0;
          }
        }
        if (occupied_areas.find(3) != occupied_areas.end())
        {
          if (cmd_vel_out_msg.linear.x > 0.0 && cmd_vel_out_msg.linear.y > 0.0)
          {
            cmd_vel_out_msg.linear.x = 0.0;
            cmd_vel_out_msg.linear.y = 0.0;
          }
        }
        if (occupied_areas.find(7) != occupied_areas.end())
        {
          if (cmd_vel_out_msg.linear.x < 0.0 && cmd_vel_out_msg.linear.y < 0.0)
          {
            cmd_vel_out_msg.linear.x = 0.0;
            cmd_vel_out_msg.linear.y = 0.0;
          }
        }
        if (occupied_areas.find(9) != occupied_areas.end())
        {
          if (cmd_vel_out_msg.linear.x < 0.0 && cmd_vel_out_msg.linear.y > 0.0)
          {
            cmd_vel_out_msg.linear.x = 0.0;
            cmd_vel_out_msg.linear.y = 0.0;
          }
        }
      }

      else if (warning)
      {
        if (cmd_vel_out_msg.linear.x > max_warning_speed)
        {
          cmd_vel_out_msg.linear.x = max_warning_speed;
        }
        else if (cmd_vel_out_msg.linear.x < -max_warning_speed)
        {
          cmd_vel_out_msg.linear.x = -max_warning_speed;
        }
        if (cmd_vel_out_msg.linear.y > max_warning_speed)
        {
          cmd_vel_out_msg.linear.y = max_warning_speed;
        }
        else if (cmd_vel_out_msg.linear.y < -max_warning_speed)
        {
          cmd_vel_out_msg.linear.y = -max_warning_speed;
        }
      }

      // Publish cmd_veL
      //      if (safety_controller_mode != "")
      //      {
      //        if (safety_controller_mode == mode_msg)
      //        {
      //          cmd_vel_pub_.publish(cmd_vel_out_msg);
      //        }
      //      }
      if (cmd_vel_published_ == false) {
        cmd_vel_pub_.publish(cmd_vel_out_msg);
        cmd_vel_published_ = true;
      }
      
    }

    else
    {
      if (collision)
      {
        // std::cout << "Entro aquí" << std::endl;
        ROS_WARN_STREAM_THROTTLE(period_to_log.toSec(), "safety: in collision");
        cmd_vel_out_msg.linear.x = 0.0;
        cmd_vel_out_msg.linear.y = 0.0;
        cmd_vel_out_msg.angular.z = 0.0;
        // Publish cmd_veL
        cmd_vel_pub_.publish(cmd_vel_out_msg);
      }
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};
