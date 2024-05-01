#ifndef ODOM_TESTS_H
#define ODOM_TESTS_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"


class OdomTests
{
  private:
    ros::NodeHandle nh_;

    // List subscribers
    ros::Subscriber odom_sub_;

    // List publishers
    ros::Publisher goal_pub_;

    // List subscribed topics
    std::string odom_topic_;

    // List logfile filenames
    std::string odom_filename_;

    // Callbacks
    void odomCallback(const nav_msgs::Odometry& msg);

    // Init / helpers
    void loadParams();
    void initLogfiles();

  public:
    OdomTests(ros::NodeHandle nh);
    ~OdomTests(void);

};

#endif // ODOM_TESTS_H
