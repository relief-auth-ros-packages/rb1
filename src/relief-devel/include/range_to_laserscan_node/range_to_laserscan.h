#ifndef RANGE_TO_LASERSCAN_H
#define RANGE_TO_LASERSCAN_H

#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"

class RangeToLaserscan
{
  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber range_sub_;
    ros::Publisher laserscan_pub_;

    std::string range_topic_in_;
    std::string laserscan_topic_out_;

    void rangeCallback(const sensor_msgs::Range& msg);


  public:
    RangeToLaserscan(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~RangeToLaserscan(void);

};

#endif // RANGE_TO_LASERSCAN_H
