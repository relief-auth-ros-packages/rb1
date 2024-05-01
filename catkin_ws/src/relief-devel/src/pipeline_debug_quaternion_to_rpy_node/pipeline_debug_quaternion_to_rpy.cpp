#include "pipeline_debug_quaternion_to_rpy_node/pipeline_debug_quaternion_to_rpy.h"


/*******************************************************************************
 * Constructor. Initializes publishers/subscribers.
 */
PipelineQuaternionToRPY::PipelineQuaternionToRPY(void)
{
  amcl_pose_sub_ = nodehandle_.subscribe("/amcl_pose", 1,
    &PipelineQuaternionToRPY::amclPoseCallback, this);
  icp_pose_sub_ = nodehandle_.subscribe(
    "/pipeline_laser_scan_matcher_node/pipeline_icp_corrected_pose", 1,
    &PipelineQuaternionToRPY::icpPoseCallback, this);
  dft_pose_sub_ = nodehandle_.subscribe(
    "/pipeline_laser_scan_matcher_node/pipeline_corrected_pose", 1,
    &PipelineQuaternionToRPY::dftPoseCallback, this);
  gt_pose_sub_ = nodehandle_.subscribe( "/ground_truth/state", 1,
    &PipelineQuaternionToRPY::gtPoseCallback, this);

  amcl_pose_pub_ = nodehandle_.advertise<relief_devel::PoseRPYWithCovarianceStamped>(
    "/amcl_pose_rpy", 1);
  icp_pose_pub_ = nodehandle_.advertise<relief_devel::PoseRPYWithCovarianceStamped>(
    "/pipeline_laser_scan_matcher_node/pipeline_icp_corrected_pose_rpy", 1);
  dft_pose_pub_ = nodehandle_.advertise<relief_devel::PoseRPYWithCovarianceStamped>(
    "/pipeline_laser_scan_matcher_node/pipeline_corrected_pose_rpy", 1);
  gt_pose_pub_ = nodehandle_.advertise<relief_devel::PoseRPYWithCovarianceStamped>(
    "/pipeline_laser_scan_matcher_node/ground_truth_pose_rpy", 1);

  ROS_INFO("[PipelineQuaternionToRPY] Node initialised");
}


/*******************************************************************************
 * Destructor
 */
PipelineQuaternionToRPY::~PipelineQuaternionToRPY(void)
{
  ROS_INFO("[PipelineQuaternionToRPY] Node destroyed");
}


/*******************************************************************************
 */
void PipelineQuaternionToRPY::amclPoseCallback(
  const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  relief_devel::PoseRPYWithCovarianceStamped mes = convert(msg);
  amcl_pose_pub_.publish(mes);
}


/*******************************************************************************
 */
void PipelineQuaternionToRPY::icpPoseCallback(
  const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  relief_devel::PoseRPYWithCovarianceStamped mes = convert(msg);
  icp_pose_pub_.publish(mes);
}


/*******************************************************************************
 */
void PipelineQuaternionToRPY::dftPoseCallback(
  const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  relief_devel::PoseRPYWithCovarianceStamped mes = convert(msg);
  dft_pose_pub_.publish(mes);
}

/*******************************************************************************
 */
void PipelineQuaternionToRPY::gtPoseCallback(const nav_msgs::Odometry& msg)
{
  geometry_msgs::PoseWithCovarianceStamped pwcs;
  pwcs.pose = msg.pose;

  // Only applies to the corridor map
  pwcs.pose.pose.position.x += 12.2;
  pwcs.pose.pose.position.y += 8.2;

  pwcs.header.stamp = msg.header.stamp;
  relief_devel::PoseRPYWithCovarianceStamped mes = convert(pwcs);
  gt_pose_pub_.publish(mes);
}


/*******************************************************************************
 */
relief_devel::PoseRPYWithCovarianceStamped PipelineQuaternionToRPY::convert(
  const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  tf::Quaternion q(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w);

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  relief_devel::PoseRPYWithCovarianceStamped mes;
  mes.header.stamp = msg.header.stamp;
  mes.header.frame_id = msg.header.frame_id;
  mes.position = msg.pose.pose.position;
  mes.roll = roll;
  mes.pitch = pitch;
  mes.yaw = yaw;
  mes.covariance = msg.pose.covariance;

  return mes;
}
