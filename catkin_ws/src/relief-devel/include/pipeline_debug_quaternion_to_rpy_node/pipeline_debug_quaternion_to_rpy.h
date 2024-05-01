#ifndef PIPELINEQUATERNIONTORPY_H
#define PIPELINEQUATERNIONTORPY_H

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "relief_devel/PoseRPYWithCovarianceStamped.h"
#include "tf/transform_datatypes.h"

class PipelineQuaternionToRPY
{
  private:
    ros::NodeHandle nodehandle_;

    ros::Subscriber amcl_pose_sub_;
    ros::Subscriber icp_pose_sub_;
    ros::Subscriber dft_pose_sub_;
    ros::Subscriber gt_pose_sub_;

    ros::Publisher amcl_pose_pub_;
    ros::Publisher icp_pose_pub_;
    ros::Publisher dft_pose_pub_;
    ros::Publisher gt_pose_pub_;

    void amclPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped& msg);
    void icpPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped& msg);
    void dftPoseCallback(
      const geometry_msgs::PoseWithCovarianceStamped& msg);
    void gtPoseCallback(const nav_msgs::Odometry& msg);

    relief_devel::PoseRPYWithCovarianceStamped convert(
      const geometry_msgs::PoseWithCovarianceStamped& msg);


  public:
    PipelineQuaternionToRPY(void);
    ~PipelineQuaternionToRPY(void);

};

#endif // PIPELINEQUATERNIONTORPY_H
