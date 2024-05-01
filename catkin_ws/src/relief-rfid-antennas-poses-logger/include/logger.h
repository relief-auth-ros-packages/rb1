#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <math.h>

#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"


class Logger
{
  private:
    ros::NodeHandle nodehandle_;

    // List subscribers
    ros::Subscriber pose_sub_;
    ros::Subscriber pose_pipeline_sub_;
    ros::Subscriber velocity_sub_;

    // List subscribed topics
    std::string pose_topic_;
    std::string pose_pipeline_topic_;
    std::string velocity_topic_;

    // Type of robot running this node
    std::string robot_type_;

    // List logfile filenames
    std::string pose_robot_filename_;
    std::string pose_pipeline_robot_filename_;
    std::string pose_filename_;
    std::string pose_pipeline_filename_;
    std::string velocity_filename_;
    std::string antennas_polarities_filename_;

    // Active readers
    unsigned int num_readers_;
    std::vector<bool> active_readers_;
    std::vector<std::string> readers_macs_;

    // Active antennas per reader
    std::vector< std::vector<bool> > readers_active_antennas_;

    // Pose per antenna and per reader
    std::vector< std::vector< std::vector<double> > > reader_antenna_poses_;


    // List callbacks
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void posePipelineCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void velocityCallback(const geometry_msgs::Twist& msg);

    // Init / helpers
    void loadParams();
    void loadReaderAndAntennaParams();
    void initLogfiles();
    int sgn(const double& x);
    void writeReadersAntennasPolarities();


  public:
    Logger(void);
    ~Logger(void);
};

#endif // LOGGER_H
