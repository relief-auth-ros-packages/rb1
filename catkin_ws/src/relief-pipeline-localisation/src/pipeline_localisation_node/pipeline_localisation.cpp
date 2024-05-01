/*
 * Copyright (c) 2011, Ivan Dryanovski, William Morris
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the CCNY Robotics Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*  This package uses Canonical Scan Matcher [1], written by
 *  Andrea Censi
 *  Code adopted and adapted from
 *  https://github.com/ccny-ros-pkg/scan_tools/tree/indigo/laser_scan_matcher
 *
 *  [1] A. Censi, "An ICP variant using a point-to-line metric"
 *  Proceedings of the IEEE International Conference
 *  on Robotics and Automation (ICRA), 2008
 */

#include <boost/assign.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pipeline_localisation_node/pipeline_localisation.h>

#define DEBUG_EXECUTION_TIMES 0

/*******************************************************************************
 * @brief Constructor
 * @param[in] nh [ros::NodeHandle]
 * @param[in] nh_private [ros::NodeHandle]
 */
PipelineLocalisation::PipelineLocalisation(
  ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  received_scan_(false),
  received_map_(false),
  received_both_scan_and_map_(0),
  ground_truths_latest_received_time_ (ros::Time::now()),
  robot_is_omw_(false),
  running_(false),
  omap_(ranges::OMap(1,1)),
  rm_(ranges::RayMarching(omap_, 1)),
  cddt_(ranges::CDDTCast(omap_, 1, 1)),
  br_(ranges::BresenhamsLine(omap_, 1))
{
  ROS_INFO("[Pipeline Localiser] Starting PipelineLocalisation");

  // **** init parameters

  initParams();

  // **** state variables

  f2b_.setIdentity();
  input_.laser[0] = 0.0;
  input_.laser[1] = 0.0;
  input_.laser[2] = 0.0;

  // Initialize output_ vectors as Null for error-checking
  output_.cov_x_m = 0;
  output_.dx_dy1_m = 0;
  output_.dx_dy2_m = 0;

  // **** publishers

  // The pipeline's result is fed-back to amcl through this publisher
  // 1. in case of true: either as the mean pose of a newly-initialised particle
  // filter, whose particle swarm is centered around the pipeline result, or
  // 2. in case of false: as a new particle in the particle filter's particle
  // swarm
  if (pipeline_feedback_init_pf_)
  {
    pipeline_feedback_pose_publisher_ =
      nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "/initialpose", 1);
  }
  else
  {
    pipeline_feedback_pose_publisher_ =
      nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        pipeline_feedback_particle_topic_, 1);
  }

  // The pipeline's result is published through this publisher
  pipeline_dft_corrected_pose_publisher_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      ros::this_node::getName() + "/corrected_pose", 1);

  // The icp-corrected amcl is published through this publisher
  pipeline_icp_corrected_pose_publisher_ =
    nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      ros::this_node::getName() + "/icp_corrected_pose", 1);

  // Publisher of the pipeline's execution times
  pipeline_execution_times_publisher_ =
    nh_.advertise<std_msgs::Duration>(
      ros::this_node::getName() + "/execution_times", 1);

  // The world scan is published through this publisher
  pipeline_world_scan_publisher_ =
    nh_.advertise<sensor_msgs::LaserScan>(
      ros::this_node::getName() + "/world_scan", 1);

  // The map scan is published through this publisher
  pipeline_map_scan_publisher_ =
    nh_.advertise<sensor_msgs::LaserScan>(
      ros::this_node::getName() + "/map_scan", 1);

  // The corrected map scan (its frame is the laser frame) is published
  // through this publisher for debugging reasons
  pipeline_map_scan_corrected_publisher_ =
    nh_.advertise<geometry_msgs::PoseArray>(
      ros::this_node::getName() + "/map_scan_corrected", 1);

  // The ground truth publisher
  pipeline_ground_truth_publisher_ =
    nh_.advertise<nav_msgs::Odometry>(
      ros::this_node::getName() + "/ground_truth", 1);

  // *** subscribers

  // This is the map
  map_subscriber_ = nh_.subscribe(map_topic_, 1,
    &PipelineLocalisation::mapCallback, this);

  // This is the pose published by amcl
  pose_subscriber_ = nh_.subscribe(input_pose_topic_, 1,
    &PipelineLocalisation::poseCallback, this);

  // This is the scan subscriber
  scan_subscriber_ = nh_.subscribe(scan_topic_, 1,
    &PipelineLocalisation::scanCallback, this);

  // This is the odom subscriber,
  // used to detect whether the robot is stationary or not
  odom_subscriber_ = nh_.subscribe(odom_topic_, 1,
    &PipelineLocalisation::odomCallback, this);

  // For debugging purposes; subscribe to the ground truth topic
  ground_truth_subscriber_ = nh_.subscribe("/ground_truth/state", 1,
    &PipelineLocalisation::groundTruthCallback, this);

  // Re-set the map png file
  omap_ = ranges::OMap(map_png_file_);
}


/*******************************************************************************
 * @brief Destructor
 * @params void
 */
PipelineLocalisation::~PipelineLocalisation()
{
  ROS_INFO("[Pipeline Localiser] Destroying PipelineLocalisation");
}


/*******************************************************************************
 * @brief This function provides the amcl algorithm with a corrected pose so
 * as to either (a) initialise itself with it, or (b) introduce the pipeline
 * result as a new particle.
 * @param[in] pose [const geometry_msgs::PoseWithCovarianceStamped&] The pose
 * to feed to the amcl
 * @return void
 */
void
PipelineLocalisation::closeLoop(
  const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  pipeline_feedback_pose_publisher_.publish(pose);
}


/*******************************************************************************
 * @brief Given two scans (their order matters), this function computes the
 * difference between all valid rays and returns a vector holding the values of
 * these diffs.
 * @param[in] scan_1 [const LDP&] The first scan (should be a world scan)
 * @param[in] scan_2 [const LDP&] The second scan (should be map scan)
 * @param[out] diff_scan [std::vector<double>] A vector holding the
 * difference between valid rays of scan_1 and scan_2.
 * @return [int] The number of valid rays
 */
int
PipelineLocalisation::computeRaysDiff(
  const LDP& scan_1,
  const LDP& scan_2,
  std::vector<double>* diff_scan)
{
  if (scan_1->nrays != scan_2->nrays)
  {
    ROS_ERROR("[Pipeline Localiser] DFT diff rays computation impossible ...");
    ROS_ERROR("                     returning empty diff vector");
    return 0;
  }

  int num_valid_rays = 0;
  for (unsigned int i = 0; i < scan_1->nrays; i++)
  {
    if ( scan_1->valid[i] && scan_2->valid[i] )
    {
      diff_scan->push_back(scan_1->readings[i] - scan_2->readings[i]);
      num_valid_rays++;
    }
    else
      diff_scan->push_back(0.0);
  }

  return num_valid_rays;
}


/*******************************************************************************
 * @brief Copies a source LDP structure to a target one.
 * @param[in] source [const LDP&] The source structure
 * @param[out] target [LDP&] The destination structure
 * @return void
 */
void
PipelineLocalisation::copyLDP(
  const LDP& source,
  LDP& target)
{
  unsigned int n = source->nrays;
  target = ld_alloc_new(n);

  copyMetaDataLDP(source, target);

  for (unsigned int i = 0; i < n; i++)
    copyRayDataLDP(source, target, i);
}

/*******************************************************************************
 * @brief Regarding a single ray with index id, this function copies data
 * from a ray of the source LDP structure to that of a target one.
 * @param[in] source [const LDP&] The source structure
 * @param[out] target [LDP&] The destination structure
 * @param[in] id [const int&] The ray index
 * @return void
 */
void
PipelineLocalisation::copyRayDataLDP(
  const LDP& source,
  LDP& target,
  const int& id)
{
  target->valid[id] = source->valid[id];
  target->theta[id] = source->theta[id];
  target->cluster[id]  = source->cluster[id];
  target->readings[id] = source->readings[id];
}

/*******************************************************************************
 * @brief Regarding the metadata from a source LDP, copy them to a
 * a target LDP.
 * @param[in] source [const LDP&] The source structure
 * @param[out] target [LDP&] The destination structure
 * @return void
 */
void
PipelineLocalisation::copyMetaDataLDP(
  const LDP& source,
  LDP& target)
{
  target->nrays = source->nrays;
  target->min_theta = source->min_theta;
  target->max_theta = source->max_theta;

  target->estimate[0] = source->estimate[0];
  target->estimate[1] = source->estimate[1];
  target->estimate[2] = source->estimate[2];

  target->odometry[0] = source->odometry[0];
  target->odometry[1] = source->odometry[1];
  target->odometry[2] = source->odometry[2];

  target->true_pose[0] = source->true_pose[0];
  target->true_pose[1] = source->true_pose[1];
  target->true_pose[2] = source->true_pose[2];
}


/*******************************************************************************
 * @brief This function uses the csm_icp result for correcting the amcl pose.
 * Given the icp output, this function express this correction in the
 * map frame. The result is the icp-corrected pose in the map frame.
 * @param[in,out] icp_corrected_pose
 * [geometry_msgs::PoseWithCovarianceStamped::Ptr&] The icp-corrected amcl pose
 * @return void
 */
void
PipelineLocalisation::correctAmclPose(
  geometry_msgs::PoseWithCovarianceStamped::Ptr& icp_corrected_pose)
{
  if (output_.valid && !nanInPose(icp_corrected_pose))
  {
    geometry_msgs::PoseWithCovarianceStamped::Ptr amcl_pose =
      boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(
        *icp_corrected_pose);

    tf::Transform map_to_base_tf;
    tf::poseMsgToTF(icp_corrected_pose->pose.pose, map_to_base_tf);

    // Now express the icp-corrected amcl pose in terms of the map frame ...
    tf::Transform icp_corrected_pose_tf = map_to_base_tf * f2b_;

    // ... and convert the transform into a message
    tf::poseTFToMsg(icp_corrected_pose_tf, icp_corrected_pose->pose.pose);

    // Make sure the orientation is in the [-π,π] interval
    wrapPoseOrientation(icp_corrected_pose);

    // At this point we have fully incorporated the whole icp transform on top
    // of the amcl pose. However, if the estimated orientation approximates the
    // true one better than the estimate about the position does the true
    // position, only incorporate the orientation:
    if (icp_incorporate_orientation_only_)
      icp_corrected_pose->pose.pose.position = amcl_pose->pose.pose.position;
  }
  else
  {
    // icp has failed: the icp-corrected pose falls back to the amcl pose
    ROS_ERROR("[PipelineLocalisation] ICP has failed; falling back to amcl pose");
  }
}


/*******************************************************************************
 * @brief Corrects the icp-corrected pose by means of the first coefficient of
 * a DFT of the signal which is the difference between a world scan and a
 * map scan.
 * @param[in,out] dft_corrected_pose [geometry_msgs::PoseWithCovarianceStamped::Ptr&]
 * When in, it is equal to the icp-corrected pose. When out, it is the
 * dft-corrected icp-corrected amcl pose.
 * @param[in] dft_coeff [std::vector<double>*] The DFT coefficients.
 * The first one is the real part of the first DFT coefficient. The second one
 * is the imaginary part of the first DFT coefficient.
 * @param[in] num_rays [const int&] The number of rays in the
 * scan that produced the dft_coeff vector
 * @return void
 */
void
PipelineLocalisation::correctICPPose(
  geometry_msgs::PoseWithCovarianceStamped::Ptr& dft_corrected_pose,
  const std::vector<double>& dft_coeff,
  const int& num_rays)
{
  if (nanInPose(dft_corrected_pose))
  {
    ROS_WARN("[PipelineLocalisation] Detected nan's in correction of icp pose");
    return;
  }

  if (dft_coeff[0] == 0.0)
  {
    ROS_WARN("[PipelineLocalisation] DFT-x returned 0.0");
    ROS_WARN("[PipelineLocalisation] ... not likely for ICP to be that accuate");
  }
  if (dft_coeff[1] == 0.0)
  {
    ROS_WARN("[PipelineLocalisation] DFT-y returned 0.0");
    ROS_WARN("[PipelineLocalisation] ... not likely for ICP to be that accuate");
  }

  // Extract yaw from pose
  double yaw = extractYawFromPose(dft_corrected_pose->pose.pose);

  // Set starting angle for scan ...
  double starting_angle;
  if (dft_do_fill_map_scan_)
    starting_angle = yaw - M_PI;
  else
    starting_angle = yaw + latest_world_scan_->angle_min;

  // ... and wrap within [-π,π]
  wrapAngle(starting_angle);

  // Turn the coefficients into x-wise and y-wise errors
  std::vector<double> errors = turnDFTCoeffsIntoErrors(
    dft_coeff, num_rays, starting_angle);

  double x_err = errors[0];
  double y_err = errors[1];

  // CORRECTION APPROACH A: simple
  dft_corrected_pose->pose.pose.position.x += x_err;

  if (laser_z_orientation_.compare("upwards") == 0)
    dft_corrected_pose->pose.pose.position.y += y_err;

  if (laser_z_orientation_.compare("downwards") == 0)
    dft_corrected_pose->pose.pose.position.y -= y_err;

/*
  // CORRECTION APPROACH B: proper; equivalent to A
  // The dft correction is expressed in the laser frame.
  // Do the correction in the laser frame and return the corrected robot's pose
  tf::Transform dft_corrected_pose_tf;
  tf::poseMsgToTF(dft_corrected_pose->pose.pose, dft_corrected_pose_tf);
  tf::Transform map_to_laser_pose_old_tf =
    dft_corrected_pose_tf * base_to_laser_;

  geometry_msgs::Pose map_to_laser_old_pose;
  tf::poseTFToMsg(map_to_laser_pose_old_tf, map_to_laser_old_pose);

  map_to_laser_old_pose.position.x += x_err;

  if (laser_z_orientation_.compare("upwards") == 0)
    map_to_laser_old_pose.position.y += y_err;

  if (laser_z_orientation_.compare("downwards") == 0)
    map_to_laser_old_pose.position.y -= y_err;

  tf::Transform map_to_laser_new_tf;
  tf::poseMsgToTF(map_to_laser_old_pose, map_to_laser_new_tf);

  // New position tf
  tf::Transform map_to_base_new_tf = map_to_laser_new_tf * laser_to_base_;
  tf::poseTFToMsg(map_to_base_new_tf, dft_corrected_pose->pose.pose);
*/
}


/*******************************************************************************
 * @brief Creates a cache for access to values of sin and cos for all values
 * in [scan_msg->angle_min, can_msg->angle_max].
 * @param[in] scan_msg [const sensor_msgs::LaserScan::Ptr&] A scan
 * @return void
 */
void
PipelineLocalisation::createCache(const sensor_msgs::LaserScan::Ptr& scan_msg)
{
  a_cos_.clear();
  a_sin_.clear();

  double angle_min = scan_msg->angle_min;
  double angle_inc = scan_msg->angle_increment;

  for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
  {
    double angle = angle_min + i * angle_inc;
    a_cos_.push_back(cos(angle));
    a_sin_.push_back(sin(angle));
  }

  input_.min_reading = scan_msg->range_min;
  input_.max_reading = scan_msg->range_max;
}


/*******************************************************************************
 * @brief Creates a transform from a 2D pose (x,y,theta)
 * @param[in] x [const double&] The x-wise coordinate of the pose
 * @param[in] y [const double&] The y-wise coordinate of the pose
 * @param[in] theta [const double&] The orientation of the pose
 * @param[in,out] t [tf::Transform&] The returned transform
 */
void
PipelineLocalisation::createTfFromXYTheta(
  const double& x,
  const double& y,
  const double& theta,
  tf::Transform& t)
{
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  q.normalize();
  t.setRotation(q);
}


/*******************************************************************************
 * @brief The map scan is published through this function, as a means of visual
 * debug
 * @param[in] amcl_pose [const geometry_msgs::PoseWithCovarianceStamped::Ptr]
 * The pose of the robot.
 * @param[in] world_scan [const sensor_msgs::LaserScan::Ptr&] The world scan
 * @param[in] map_scan [const sensor_msgs::LaserScan::Ptr&] The map scan
 * @param[in] world_scan_ldp [const LDP&] The world scan in LDP form
 * @param[in] map_scan_ldp [const LDP&] The map scan in LDP form
 * @return void
 */
void
PipelineLocalisation::debugICP(
  const geometry_msgs::PoseWithCovarianceStamped::Ptr amcl_pose,
  const sensor_msgs::LaserScan::Ptr& world_scan,
  const sensor_msgs::LaserScan::Ptr& map_scan,
  const LDP& world_scan_ldp,
  const LDP& map_scan_ldp)
{
  // Publish scans for visualisation
  if (icp_do_publish_scans_)
    visualiseScans(world_scan_ldp, map_scan_ldp);

  geometry_msgs::PoseArray map_scan_corrected_poses =
    transformLaserScan(amcl_pose, map_scan, f2b_);

  pipeline_map_scan_corrected_publisher_.publish(map_scan_corrected_poses);
}


/*******************************************************************************
 * @brief Given the ICP-corrected pose and a world scan, this function corrects
 * the pose by DFT-ing the difference between the world scan and a map scan
 * taken at the ICP-corrected pose.
 * @param[in] icp_corrected_pose
 * [const geometry_msgs::PoseWithCovarianceStamped::Ptr&]
 * The amcl pose corrected by the preceding ICP
 * @param[in] latest_world_scan [const sensor_msgs::LaserScan::Ptr&] The real
 * laser scan
 * @return [geometry_msgs::PoseWithCovarianceStamped::Ptr] The DFT-corrected
 * ICP-corrected pose
 */
geometry_msgs::PoseWithCovarianceStamped::Ptr
PipelineLocalisation::doDFT(
  const geometry_msgs::PoseWithCovarianceStamped::Ptr& icp_corrected_pose,
  const sensor_msgs::LaserScan::Ptr& latest_world_scan)
{
  // Measure execution time
  #if DEBUG_EXECUTION_TIMES == 1
  ros::Time start_dft = ros::Time::now();
  #endif

  geometry_msgs::PoseWithCovarianceStamped::Ptr dft_corrected_pose =
    boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(
      *icp_corrected_pose);

  for (unsigned int i = 0; i < dft_iterations_; i++)
  {
    // Obtain the map scan
    sensor_msgs::LaserScan::Ptr map_scan = scanMap(dft_corrected_pose,
      dft_map_scan_method_, dft_do_fill_map_scan_);

    // Convert the map and world scans into LDP and store it for the DFT module
    LDP dft_world_scan_ldp;
    laserScanToLDP(latest_world_scan, dft_world_scan_ldp);

    LDP dft_map_scan_ldp;
    laserScanToLDP(map_scan, dft_map_scan_ldp);

    // Preprocess the world and map scans. Conditionally:
    // 1. fill their blind spots
    // 2. undersample scans
    // 3. invalidate rays which differ more than a given threshold
    preprocessDFTScans(dft_world_scan_ldp, dft_map_scan_ldp);

    // Calculate the input signal of the DFT
    std::vector<double> rays_diff;
    int num_valid_rays =
      computeRaysDiff(dft_world_scan_ldp, dft_map_scan_ldp, &rays_diff);

    // No use in "correcting" the icp pose since there are no valid rays;
    // correcting loses its meaning in the process.
    if (num_valid_rays > dft_min_valid_rays_)
    {
      // Publish scans for visualisation
      if (dft_do_publish_scans_)
        visualiseScans(dft_world_scan_ldp, dft_map_scan_ldp);

      // Get the DFT X1 coefficient
      std::vector<double> dft_coeff_vector =
        DFTUtils::getFirstDFTCoefficient(rays_diff);

      double xe = dft_coeff_vector[0];
      double ye = dft_coeff_vector[1];

      // Correct the ICP-corrected pose by using the dft output
      correctICPPose(dft_corrected_pose, dft_coeff_vector, rays_diff.size());

      if (sqrtf(xe*xe + ye*ye) < 0.001)
        break;
    }

    // Delete the map scan ptr
    map_scan.reset();
    ld_free(dft_map_scan_ldp);
    ld_free(dft_world_scan_ldp);
  }

  //testDFTCircle();

  #if DEBUG_EXECUTION_TIMES == 1
  ROS_ERROR("doDFT() took %.2f ms",
    (ros::Time::now() - start_dft).toSec() * 1000);
  #endif

  return dft_corrected_pose;
}


/*******************************************************************************
 * @brief Given the amcl pose and a world scan, this function corrects
 * the pose by ICP-ing the world scan and a map scan taken at the amcl pose
 * @param[in] amcl_pose_msg [const geometry_msgs::PoseWithCovarianceStamped::Ptr&]
 * The amcl pose
 * @param[in] latest_world_scan [const sensor_msgs::LaserScan::Ptr&] The real
 * laser scan
 * @return [geometry_msgs::PoseWithCovarianceStamped::Ptr] The * ICP-corrected
 * pose
 */
geometry_msgs::PoseWithCovarianceStamped::Ptr
PipelineLocalisation::doICP(
  const geometry_msgs::PoseWithCovarianceStamped::Ptr& amcl_pose_msg,
  const sensor_msgs::LaserScan::Ptr& latest_world_scan)
{
  // Measure execution time
  #if DEBUG_EXECUTION_TIMES == 1
  ros::Time start_icp = ros::Time::now();
  #endif

  geometry_msgs::PoseWithCovarianceStamped::Ptr icp_corrected_pose =
    boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(*amcl_pose_msg);

  for (unsigned int i = 0; i < icp_iterations_; i++)
  {
    // Convert the latest world scan into LDP form
    LDP world_scan_ldp;
    laserScanToLDP(latest_world_scan, world_scan_ldp);

    // Obtain the map scan ...
    sensor_msgs::LaserScan::Ptr map_scan = scanMap(icp_corrected_pose,
      icp_map_scan_method_, icp_do_fill_map_scan_);

    // .. and turn into a suitable structure (LDP) for processing
    LDP map_scan_ldp;
    laserScanToLDP(map_scan, map_scan_ldp);

    // Preprocess the map and world scans. Conditionally:
    // 1. fill their blind spots
    // 2. clip above and below thresholds
    // 3. undersample to mitigate noise
    // 4. invalidate rays whose difference is greater than a set threshold
    preprocessICPScans(world_scan_ldp, map_scan_ldp);

    // Do the ICP thing
    processScan(world_scan_ldp, map_scan_ldp);

    // RVIzual debug
    if (icp_visual_debug_)
      debugICP(icp_corrected_pose, latest_world_scan, map_scan,
        world_scan_ldp, map_scan_ldp);

    // Correct the amcl pose given the icp output
    correctAmclPose(icp_corrected_pose);

    // Delete the map scan ptr and the LDPs
    map_scan.reset();
    ld_free(world_scan_ldp);
    ld_free(map_scan_ldp);
  }

  #if DEBUG_EXECUTION_TIMES == 1
  ROS_ERROR("doICP() took %.2f ms",
    (ros::Time::now() - start_icp).toSec() * 1000);
  #endif

  //testICPCircle();

  return icp_corrected_pose;
}


/*******************************************************************************
 * @brief Extracts the yaw component from the input pose's quaternion.
 * @param[in] pose [const geometry_msgs::Pose&] The input pose
 * @return [double] The pose's yaw
 */
double
PipelineLocalisation::extractYawFromPose(const geometry_msgs::Pose& pose)
{
  tf::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);

  tf::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  wrapAngle(yaw);

  return yaw;
}


/*******************************************************************************
 * @brief Receives two laser scans in LDP form as input.
 * Supposing that ldp.max_theta - ldp.min_theta < 2π, this function fills in the
 * missing range values of target_scan_ldp (missing in the sense that in the
 * end the target scan will be ranging over 2π) with the values of
 * source_scan_ldp that correspond to the missing angles.
 * Noise of a normal distribution is then added to these values.
 * @param[in] source_scan_ldp [const LDP&] The source scan (should be a map
 * scan)
 * @param[in,out] target_scan_ldp [LDP&] The target scan whose missing range
 * values will be filled-in (should be a world scan)
 * @param[in] operation [const std::string&] The origin operation; should be
 * either "icp" or "dft".
 * @return void
 */
void
PipelineLocalisation::fillLaserScanLDP(
  const LDP& source_scan_ldp,
  LDP& target_scan_ldp,
  const std::string& operation)
{
  double angle_min = target_scan_ldp->min_theta;
  double angle_max = target_scan_ldp->max_theta;

  // The unfilled scan's number of rays
  int prev_nrays = target_scan_ldp->nrays;

  // The filled scan's number of rays
  int two_pi_nrays =
    numRaysFromAngleRange(angle_min, angle_max, prev_nrays, 2*M_PI);

  // How many rays to fill in
  int total_rays_to_fill = two_pi_nrays - prev_nrays;

  // We assume a symmetrical distribution of rays around 0
  int half_nrays = total_rays_to_fill / 2;

  // This is the filled-in scan:
  // world_scan + map_scan (where world_scan is unfilled)
  LDP total_scan_ldp = ld_alloc_new(two_pi_nrays);

  // Random generators; one per operation
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> >
    generator_icp(boost::mt19937(time(0)),
      boost::normal_distribution<>(
        icp_fill_mean_value_, icp_fill_std_value_));

  boost::variate_generator<boost::mt19937, boost::normal_distribution<> >
    generator_dft(boost::mt19937(time(0)),
      boost::normal_distribution<>(
        dft_fill_mean_value_, dft_fill_std_value_));

  // Fill first missing segment
  for (unsigned int i = 0; i < half_nrays; i++)
  {
    // A random number that will be used to add noise to the rays of the map
    // scan that are missing from world scan
    double rn = 0.0;
    if (operation.compare(op_icp_) == 0)
      rn = generator_icp();
    else if (operation.compare(op_dft_) == 0)
      rn = generator_dft();
    else
    {
      ROS_ERROR("[Pipeline Localiser] Unknown operation. Aborting ...");
      return;
    }

    total_scan_ldp->valid[i] = source_scan_ldp->valid[i];
    total_scan_ldp->theta[i] = source_scan_ldp->theta[i];

    if (source_scan_ldp->readings[i] + rn <= latest_world_scan_->range_min)
      total_scan_ldp->readings[i] = latest_world_scan_->range_min;
    else if (source_scan_ldp->readings[i] + rn >= latest_world_scan_->range_max)
      total_scan_ldp->readings[i] = latest_world_scan_->range_max;
    else
      total_scan_ldp->readings[i] = source_scan_ldp->readings[i] + rn;
  }

  // Copy actual laser ranges
  for (unsigned int i = 0; i < prev_nrays; i++)
  {
    total_scan_ldp->valid[half_nrays + i] = target_scan_ldp->valid[i];
    total_scan_ldp->theta[half_nrays + i] = target_scan_ldp->theta[i];
    total_scan_ldp->readings[half_nrays + i] = target_scan_ldp->readings[i];
  }

  // Fill second missing segment
  for (unsigned int i = 0; i < half_nrays; i++)
  {
    // A random number that will be used to add noise to the rays of the map
    // scan that are missing from world scan
    double rn = 0.0;
    if (operation.compare(op_icp_) == 0)
      rn = generator_icp();
    else if (operation.compare(op_dft_) == 0)
      rn = generator_dft();
    else
    {
      ROS_ERROR("[Pipeline Localiser] Unknown operation. Aborting ...");
      return;
    }

    total_scan_ldp->valid[half_nrays + prev_nrays + i] =
      source_scan_ldp->valid[half_nrays + prev_nrays + i];

    total_scan_ldp->theta[half_nrays + prev_nrays + i] =
      source_scan_ldp->theta[half_nrays + prev_nrays + i];

    if (source_scan_ldp->readings[half_nrays + prev_nrays + i] + rn <=
      latest_world_scan_->range_min)
    {
      total_scan_ldp->readings[half_nrays + prev_nrays + i] =
        latest_world_scan_->range_min;
    }
    else if (source_scan_ldp->readings[half_nrays + prev_nrays + i] + rn >=
      latest_world_scan_->range_max)
    {
      total_scan_ldp->readings[half_nrays + prev_nrays + i] =
        latest_world_scan_->range_max;
    }
    else
    {
      total_scan_ldp->readings[half_nrays + prev_nrays + i] =
        source_scan_ldp->readings[half_nrays + prev_nrays + i] + rn;
    }
  }

  total_scan_ldp->min_theta = total_scan_ldp->theta[0];
  total_scan_ldp->max_theta = total_scan_ldp->theta[two_pi_nrays - 1];

  total_scan_ldp->nrays = two_pi_nrays;

  copyLDP(total_scan_ldp, target_scan_ldp);

  ld_free(total_scan_ldp);
}


/*******************************************************************************
 * @brief Finds the transform between the laser frame and the base frame
 * @param[in] frame_id [const::string&] The laser's frame id
 * @return [bool] True when the transform was found, false otherwise.
 */
bool
PipelineLocalisation::getBaseToLaserTf(const std::string& frame_id)
{
  ros::Time t = ros::Time::now();

  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener_.waitForTransform(base_frame_, frame_id, t, ros::Duration(1.0));

    // The direction of the transform returned will be from the base_frame_
    // to the frame_id. Which if applied to data, will transform data in
    // the frame_id into the base_frame_.
    tf_listener_.lookupTransform(base_frame_, frame_id, t, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("[Pipeline Localiser] Could not get initial transform from");
    ROS_WARN("base frame to %s: %s", frame_id.c_str(), ex.what());

    return false;
  }

  base_to_laser_ = base_to_laser_tf;
  laser_to_base_ = base_to_laser_.inverse();

  return true;
}


/*******************************************************************************
 * @brief Given the robot's pose in the map frame, this function returns the
 * laser's pose in the map frame.
 * @param[in] robot_pose [const geometry_msgs::Pose&] The robot's pose in the
 * map frame.
 * @return [geometry_msgs::Pose] The pose of the laser in the map frame.
 */
geometry_msgs::Pose
PipelineLocalisation::getCurrentLaserPose(const geometry_msgs::Pose& robot_pose)
{
  // Transform the robot_pose to a transform
  tf::Transform map_to_base_tf;
  tf::poseMsgToTF(robot_pose, map_to_base_tf);

  // Get the laser's pose in the map frame (as a transform)
  tf::Transform map_to_laser_tf = map_to_base_tf * base_to_laser_;

  // Convert the transform into a message
  geometry_msgs::Pose laser_pose;
  tf::poseTFToMsg(map_to_laser_tf, laser_pose);

  // Return the laser's pose
  return laser_pose;
}


/*******************************************************************************
 * @brief Returns the number of valid rays of a LDP scan
 * @param[in] scan [const LDP&] The input scan
 * @return [int] The number of valid rays of scan
 */
int
PipelineLocalisation::getNumberOfValidRays(const LDP& scan)
{
  int counter = 0;
  for (unsigned int i = 0; i < scan->nrays; i++)
  {
    if (scan->valid[i] == 1)
      counter++;
  }

  return counter;
}


/*******************************************************************************
 * @brief Calculates the compound velocity in (x,y,theta) from a twist message.
 * @param[in] twist_msg [const geometry_msgs::Twist&] The twist message
 * containing the linear and angular velocities of the robot
 * @return [double] The compound velocity
 */
double
PipelineLocalisation::getTotal3DVelocity(const geometry_msgs::Twist& twist_msg)
{
  return sqrt(
    twist_msg.linear.x * twist_msg.linear.x +
    twist_msg.linear.y * twist_msg.linear.y +
    twist_msg.angular.z * twist_msg.angular.z);
}


/*******************************************************************************
 */
void
PipelineLocalisation::groundTruthCallback(const nav_msgs::Odometry& ground_truth)
{
  nav_msgs::Odometry msg;
  msg.header = ground_truth.header;
  msg.pose = ground_truth.pose;
  msg.twist = ground_truth.twist;

  pipeline_ground_truth_publisher_.publish(msg);
}


/*******************************************************************************
 * @brief This is where all the magic happens
 * @param[in] pose_msg [const geometry_msgs::PoseWithCovarianceStamped::Ptr&]
 * The amcl pose wrt to the map frame
 * @return void
 */
void
PipelineLocalisation::handleInputPose(
  const geometry_msgs::PoseWithCovarianceStamped::Ptr& pose_msg)
{
  // Both the laser scan and the map ought to have been received for the
  // icp to commence.
  if (!received_scan_ || !received_map_)
  {
    //pipeline_dft_corrected_pose_publisher_.publish(*pose_msg);
    return;
  }

  // Construct the 3rd-party ray-casters AFTER both a scan and the map is
  // received
  received_both_scan_and_map_++;
  if (received_both_scan_and_map_ == 1)
    initRangeLibRayCasters();

  // Return if the pose received from amcl contains nan's
  if (nanInPose(pose_msg))
  {
    ROS_ERROR("[PipelineLocalisation] amcl pose contains nan's; aborting ...");
    return;
  }

  // Bail if already running // GUARD
  if (running_)
  {
    //pipeline_dft_corrected_pose_publisher_.publish(*pose_msg);
    ROS_ERROR("[PipelineLocalisation] Already running; aborting ...");
    return;
  }

  // Do not initialise amcl if the robot is not in motion
  if (close_loop_ && pipeline_feedback_init_pf_)
  {
    if (!robot_is_omw_)
    {
      ROS_INFO("[PipelineLocalisation] Closed loop is enabled and robot is stationary");
      ROS_INFO("[PipelineLocalisation] I will stop bugging amcl now ...");
      return;
    }
  }

  // Start clock: measure how much time it takes to execute the pipeline
  // from obtaining the amcl pose to publishing the pipeline-corrected pose.
  ros::Time start = ros::Time::now();

  // Set status to running to block further execution
  running_ = true;

  // ... and get a lock
  //boost::mutex::scoped_lock lock(mutex_);


  // The two pipeline modules may alter the contents of the world scan in its
  // LDP form, but they need to refer to the same physical scan.
  // Therefore make two distinct copies of it.
  sensor_msgs::LaserScan::Ptr latest_world_scan_icp =
    boost::make_shared<sensor_msgs::LaserScan>(*latest_world_scan_);
  sensor_msgs::LaserScan::Ptr latest_world_scan_dft =
    boost::make_shared<sensor_msgs::LaserScan>(*latest_world_scan_);

  // ---------------------------------------------------------------------------
  // --- ICP SEGMENT -----------------------------------------------------------
  // ---------------------------------------------------------------------------
  geometry_msgs::PoseWithCovarianceStamped::Ptr icp_corrected_pose =
    doICP(pose_msg, latest_world_scan_icp);

  // ---------------------------------------------------------------------------
  // --- DFT SEGMENT -----------------------------------------------------------
  // ---------------------------------------------------------------------------
  geometry_msgs::PoseWithCovarianceStamped::Ptr dft_corrected_pose =
    doDFT(icp_corrected_pose, latest_world_scan_dft);

  // Close the loop: feed the corrected pose to the amcl as its initial pose:
  if (close_loop_)
    closeLoop(*dft_corrected_pose);

  // GUARD stand-down
  running_ = false;

  // End clock: measure how much time it takes to execute the pipeline
  // from obtaining the amcl pose to publishing the pipeline-corrected pose.
  measureExecutionTime(start, ros::Time::now());

  // Publish the icp-corrected pose
  pipeline_icp_corrected_pose_publisher_.publish(*icp_corrected_pose);

  // Publish the dft-corrected pose
  pipeline_dft_corrected_pose_publisher_.publish(*dft_corrected_pose);
}


/*******************************************************************************
 * @brief Initializes parameters
 * @return void
 */
void
PipelineLocalisation::initParams()
{
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "base_footprint";
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "map";
  if (!nh_private_.getParam ("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if (!nh_private_.getParam ("scan_topic", scan_topic_))
    scan_topic_ = "/scan";
  if (!nh_private_.getParam ("map_topic", map_topic_))
    map_topic_ = "/map";
  if (!nh_private_.getParam ("odom_topic", odom_topic_))
    odom_topic_ = "/odom";
  if (!nh_private_.getParam ("input_pose_topic", input_pose_topic_))
    input_pose_topic_ = "/amcl_pose";
  if (!nh_private_.getParam ("laser_z_orientation", laser_z_orientation_))
    laser_z_orientation_ = "upwards";

  // String identifiers of the pipeline's internal operations
  op_icp_ = "icp";
  op_dft_ = "dft";

  // **** How to publish the output?
  // tf (fixed_frame->base_frame),
  // pose message (pose of base frame in the fixed frame)

  if (!nh_private_.getParam ("close_loop", close_loop_))
    close_loop_ = false;

  if (!nh_private_.getParam("position_covariance", position_covariance_))
  {
    position_covariance_.resize(3);
    std::fill(position_covariance_.begin(), position_covariance_.end(), 1e-9);
  }

  if (!nh_private_.getParam("orientation_covariance", orientation_covariance_))
  {
    orientation_covariance_.resize(3);
    std::fill(orientation_covariance_.begin(), orientation_covariance_.end(), 1e-9);
  }
  // **** CSM parameters - comments copied from algos.h (by Andrea Censi)

  // Maximum angular displacement between scans
  if (!nh_private_.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
    input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!nh_private_.getParam ("max_linear_correction", input_.max_linear_correction))
    input_.max_linear_correction = 0.50;

  // Maximum ICP cycle iterations
  if (!nh_private_.getParam ("max_iterations", input_.max_iterations))
    input_.max_iterations = 10;

  // A threshold for stopping (m)
  if (!nh_private_.getParam ("epsilon_xy", input_.epsilon_xy))
    input_.epsilon_xy = 0.000001;

  // A threshold for stopping (rad)
  if (!nh_private_.getParam ("epsilon_theta", input_.epsilon_theta))
    input_.epsilon_theta = 0.000001;

  // Maximum distance for a correspondence to be valid
  if (!nh_private_.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
    input_.max_correspondence_dist = 0.3;

  // Noise in the scan (m)
  if (!nh_private_.getParam ("sigma", input_.sigma))
    input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.
  if (!nh_private_.getParam ("use_corr_tricks", input_.use_corr_tricks))
    input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!nh_private_.getParam ("restart", input_.restart))
    input_.restart = 0;

  // Restart: Threshold for restarting
  if (!nh_private_.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
    input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!nh_private_.getParam ("restart_dt", input_.restart_dt))
    input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!nh_private_.getParam ("restart_dtheta", input_.restart_dtheta))
    input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!nh_private_.getParam ("clustering_threshold", input_.clustering_threshold))
    input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!nh_private_.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
    input_.orientation_neighbourhood = 20;

  // If 0, it's vanilla ICP
  if (!nh_private_.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
    input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!nh_private_.getParam ("do_alpha_test", input_.do_alpha_test))
    input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!nh_private_.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
    input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  if (!nh_private_.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
    input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  if (!nh_private_.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
    input_.outliers_adaptive_order = 0.7;

  if (!nh_private_.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
    input_.outliers_adaptive_mult = 2.0;

  // If you already have a guess of the solution, you can compute the polar angle
  // of the points of one scan in the new position. If the polar angle is not a monotone
  // function of the readings index, it means that the surface is not visible in the
  // next position. If it is not visible, then we don't use it for matching.
  if (!nh_private_.getParam ("do_visibility_test", input_.do_visibility_test))
    input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!nh_private_.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
    input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  if (!nh_private_.getParam ("do_compute_covariance", input_.do_compute_covariance))
    input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  if (!nh_private_.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
    input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  if (!nh_private_.getParam ("use_ml_weights", input_.use_ml_weights))
    input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  if (!nh_private_.getParam ("use_sigma_weights", input_.use_sigma_weights))
    input_.use_sigma_weights = 0;

  if (!nh_private_.getParam ("gpm_theta_bin_size_deg", input_.gpm_theta_bin_size_deg))
    input_.gpm_theta_bin_size_deg = 5;
  if (!nh_private_.getParam ("gpm_extend_range_deg", input_.gpm_extend_range_deg))
    input_.gpm_extend_range_deg = 15;
  if (!nh_private_.getParam ("gpm_interval", input_.gpm_interval))
    input_.gpm_interval = 1;


  // *** parameters introduced by us

  if (!nh_private_.getParam("map_png_file", map_png_file_))
    map_png_file_ = "";

  if (!nh_private_.getParam("pipeline_feedback_cutoff_velocity", pipeline_feedback_cutoff_velocity_))
    pipeline_feedback_cutoff_velocity_ = 0.001;

  if (!nh_private_.getParam("pipeline_feedback_init_pf", pipeline_feedback_init_pf_))
    pipeline_feedback_init_pf_ = false;

  if (!nh_private_.getParam("pipeline_feedback_particle_topic", pipeline_feedback_particle_topic_))
    pipeline_feedback_particle_topic_ = "particle_introduction";

  if (!nh_private_.getParam("map_scan_theta_disc", map_scan_theta_disc_))
    map_scan_theta_disc_ = 720;

  if (!nh_private_.getParam("icp_iterations", icp_iterations_))
    icp_iterations_ = 1;

  if (!nh_private_.getParam("icp_incorporate_orientation_only", icp_incorporate_orientation_only_))
    icp_incorporate_orientation_only_ = true;

  if (!nh_private_.getParam("icp_map_scan_method", icp_map_scan_method_))
    icp_map_scan_method_ = "vanilla";

  // Fill map for more information on matching
  if (!nh_private_.getParam("icp_visual_debug", icp_visual_debug_))
    icp_visual_debug_ = false;

  if (!nh_private_.getParam("icp_do_fill_map_scan", icp_do_fill_map_scan_))
    icp_do_fill_map_scan_ = false;

  if (!nh_private_.getParam("icp_scan_undersample_rate", icp_scan_undersample_rate_))
    icp_scan_undersample_rate_ = 1;

  if (!nh_private_.getParam("dft_scan_undersample_rate", dft_scan_undersample_rate_))
    dft_scan_undersample_rate_ = 1;

  if (!nh_private_.getParam("icp_do_clip_scans", icp_do_clip_scans_))
    icp_do_clip_scans_ = false;

  if (!nh_private_.getParam("icp_clip_sill", icp_clip_sill_))
    icp_clip_sill_ = 100.0;

  if (!nh_private_.getParam("icp_clip_lintel", icp_clip_lintel_))
    icp_clip_lintel_ = 0.0;

  if (!nh_private_.getParam("icp_fill_mean_value", icp_fill_mean_value_))
    icp_fill_mean_value_ = 0.0;

  if (!nh_private_.getParam("icp_fill_std_value", icp_fill_std_value_))
    icp_fill_std_value_ = 0.01;

  if (!nh_private_.getParam("icp_do_invalidate_diff_rays", icp_do_invalidate_diff_rays_))
    icp_do_invalidate_diff_rays_ = false;

  if (!nh_private_.getParam("icp_invalidate_diff_rays_epsilon", icp_invalidate_diff_rays_epsilon_))
    icp_invalidate_diff_rays_epsilon_ = false;

  if (!nh_private_.getParam("icp_do_publish_scans", icp_do_publish_scans_))
    icp_do_publish_scans_ = false;

  if (!nh_private_.getParam("dft_iterations", dft_iterations_))
    dft_iterations_ = 1;

  if (!nh_private_.getParam("dft_do_fill_map_scan", dft_do_fill_map_scan_))
    dft_do_fill_map_scan_ = false;

  if (!nh_private_.getParam("dft_map_scan_method", dft_map_scan_method_))
    dft_map_scan_method_ = "vanilla";

  if (!nh_private_.getParam("dft_do_publish_scans", dft_do_publish_scans_))
    dft_do_publish_scans_ = false;

if (!nh_private_.getParam("dft_min_valid_rays", dft_min_valid_rays_))
    dft_min_valid_rays_ = 0;

if (!nh_private_.getParam("dft_fill_mean_value", dft_fill_mean_value_))
    dft_fill_mean_value_ = 0.0;

  if (!nh_private_.getParam("dft_fill_std_value", dft_fill_std_value_))
    dft_fill_std_value_ = 0.00;


  // Invalidate rays between world and map scans. For rays of equal index,
  // if their range is more than do_invalidate_diff_rays_epsilon, then they will be
  // considered invalid by the scan matcher.
  if (!nh_private_.getParam("dft_do_invalidate_diff_rays", dft_do_invalidate_diff_rays_))
    dft_do_invalidate_diff_rays_ = false;

  if (!nh_private_.getParam("dft_invalidate_diff_rays_epsilon", dft_invalidate_diff_rays_epsilon_))
    dft_invalidate_diff_rays_epsilon_ = 0.05;
}


/*******************************************************************************
 * @brief Invalidates a LDP scan's rays. If these are above or below certain
 * thresholds, then the `valid` field of their LDP is marked with 0.
 * @param[in,out] scan [LDP&] The input scan.
 * @param[in] lintel [const double&] The threshold under which rays will be
 * invalidated.
 * @param[in] sill [const double&] The threshold over which rays will be
 * invalidated.
 */
void
PipelineLocalisation::invalidateByClippingScan(
  LDP& scan,
  const double& lintel,
  const double& sill)
{
  for (unsigned int i = 0; i < scan->nrays; i++)
  {
    if ( scan->readings[i] > sill )
      scan->valid[i] = 0;

    if ( scan->readings[i] < lintel )
      scan->valid[i] = 0;
  }
}


/*******************************************************************************
 * @brief Initialises the ray-casters from the RangeLib library. Since they
 * take as arguments the maximum range of the lidar and the resolution of the
 * map, and these are unknown before being received, this function should be
 * called ONCE, exactly after the first scan and the map are received.
 * @param void
 * @return void
 */
void PipelineLocalisation::initRangeLibRayCasters()
{
  // Max laser range in pixels
  float max_range_rc = latest_world_scan_->range_max / map_.info.resolution;

  // Init ray-casters
  if (icp_map_scan_method_.compare("bresenham") == 0 ||
    dft_map_scan_method_.compare("bresenham") == 0)
  {
    br_ = ranges::BresenhamsLine(omap_, max_range_rc);
  }

  if (icp_map_scan_method_.compare("ray_marching") == 0 ||
    dft_map_scan_method_.compare("ray_marching") == 0)
  {
    rm_ = ranges::RayMarching(omap_, max_range_rc);
  }

  if (icp_map_scan_method_.compare("cddt") == 0 ||
    dft_map_scan_method_.compare("cddt") == 0)
  {
    cddt_ = ranges::CDDTCast(omap_, max_range_rc, map_scan_theta_disc_);
  }
}


/*******************************************************************************
 * @brief Invalidates a world and a map scan by clipping them above and below.
 * @param[in,out] world_scan [LDP&] A world scan in LDP form
 * @param[in,out] map_scan [LDP&] A map scan in LDP form
 * @param[in] lintel [const double&] The threshold under which rays will be
 * invalidated.
 * @param[in] sill [const double&] The threshold over which rays will be
 * invalidated.
 * @return void
 */
void
PipelineLocalisation::invalidateByClippingScans(
  LDP& world_scan,
  LDP& map_scan,
  const double& lintel,
  const double& sill)
{
  invalidateByClippingScan(world_scan, lintel, sill);
  invalidateByClippingScan(map_scan, lintel, sill);
}


/*******************************************************************************
 * @brief Invalidates scans by checking the difference between their rays and
 * a known threshold, based on the origin operation.
 * @param[in] scan_1 [LDP&] A LDP scan (should be a world scan)
 * @param[in] scan_2 [LDP&] A LDP scan (should be a map scan)
 * @param[in] threshold [const double&] A threshold over which the difference
 * between rays of scan_1 and scan_2 will invalidate said rays if operation
 * is equal to "dft". If operation is equal to "icp", this function invalidates
 * the rays of scan_2 over [min_angle, max_angle] and those of scan_1 if
 * additionally scan_1 is to be filled.
 * @param[in] operation [const std::string&] The origin operation. Available
 * values are "icp" or "dft".
 * @return void
 */
void
PipelineLocalisation::invalidateDiffRays(
  LDP& scan_1,
  LDP& scan_2,
  const double& threshold,
  const std::string& operation)
{
  if (scan_1->nrays != scan_2->nrays)
  {
    ROS_ERROR("[Pipeline Localiser] diff rays invalidation impossible");
    return;
  }

  // If DFT: invalidate both rays as what is meaningful is the value of the
  // difference of the rays of two scans, and therefore it makes more sense to
  // invalidate both
  if (operation.compare(op_dft_) == 0)
  {
    double angle_min = latest_world_scan_->angle_min;
    double angle_max = latest_world_scan_->angle_max;

    // angle_low is the angle from which rays are going to be invalidated
    // if dft_do_fill_map_scan_ is false; angle_high is the angle until which
    // this is going to happen
    double angle_low = angle_max + M_PI;
    double angle_high = angle_min + M_PI;

    // wrap them in [-π,π]
    wrapAngle(angle_low);
    wrapAngle(angle_high);

    for (unsigned int i = 0; i < scan_1->nrays; i++)
    {
      // If the map is not to be filled (this is reasonable now -- it may be the
      // case that the fake rear laser measurements do not help as much as we
      // would wish they would) then invalidate portion of the scan on the front
      // side.
      if (dft_do_fill_map_scan_)
      {
        if ((scan_1->theta[i] >= angle_low && scan_1->theta[i] <= angle_high) ||
          (scan_1->theta[i] <= latest_world_scan_->angle_min &&
           scan_1->theta[i] >= -M_PI) ||
          (scan_1->theta[i] >= latest_world_scan_->angle_max &&
           scan_1->theta[i] <= M_PI))
        {
          scan_1->valid[i] = 0;
          scan_2->valid[i] = 0;
        }
      }

      if (scan_1->valid[i] && scan_2->valid[i])
      {
        if ( fabs(scan_1->readings[i] - scan_2->readings[i]) >= threshold )
        {
          scan_1->valid[i] = 0;
          scan_2->valid[i] = 0;
        }
      }
    }
  }
  else if (operation.compare(op_icp_) == 0)
  {
    if (!icp_do_fill_map_scan_)
    {
      for (unsigned int i = 0; i < scan_1->nrays; i++)
      {
        if (scan_1->valid[i] && scan_2->valid[i])
        {
          if ( fabs(scan_1->readings[i] - scan_2->readings[i]) >= threshold )
          {
            scan_1->valid[i] = 0;
            scan_2->valid[i] = 0;
          }
        }
      }
    }
    else
    {
      int original_size = latest_world_scan_->ranges.size();
      int new_size = scan_1->nrays;
      int added_rays = new_size - original_size;
      int half_rays = added_rays / 2;

      // Invalidate first filled-in segment; the world scan rays are invalidated
      for (unsigned int i = 0; i < half_rays; i++)
      {
        if (scan_1->valid[i] && scan_2->valid[i])
        {
          if ( fabs(scan_1->readings[i] - scan_2->readings[i]) >= threshold )
            scan_1->valid[i] = 0;
        }
      }

      // Invalidate the non-filled in segment; the map scan rays are invalidated
      for (unsigned int i = half_rays; i < new_size - half_rays; i++)
      {
        if (scan_1->valid[i] && scan_2->valid[i])
        {
          if ( fabs(scan_1->readings[i] - scan_2->readings[i]) >= threshold )
          {
            scan_1->valid[i] = 0;
            scan_2->valid[i] = 0;
          }
        }
      }

      // Invalidate second filled-in segment; the world scan rays are invalidated
      for (unsigned int i = new_size - half_rays; i < new_size; i++)
      {
        if (scan_1->valid[i] && scan_2->valid[i])
        {
          if ( fabs(scan_1->readings[i] - scan_2->readings[i]) >= threshold )
            scan_1->valid[i] = 0;
        }
      }
    }
  }
  else
    ROS_ERROR("[Pipeline Localiser] Invalid operation during invalidation");
}


/*******************************************************************************
 * @brief Converts a LaserScan laser scan to a LDP structure.
 * @param[in] scan_msg [const sensor_msgs::LaserScan::Ptr&] The input scan
 * @param[out] ldp [LDP&] The output LDP scan
 * @return void
 */
void
PipelineLocalisation::laserScanToLDP(
  const sensor_msgs::LaserScan::Ptr& scan_msg,
  LDP& ldp)
{
  unsigned int n = scan_msg->ranges.size();
  ldp = ld_alloc_new(n);

  float min_range = scan_msg->range_min;
  float max_range = scan_msg->range_max;
  float angle_min = scan_msg->angle_min;
  float angle_inc = scan_msg->angle_increment;

  for (unsigned int i = 0; i < n; i++)
  {
    double r = scan_msg->ranges[i];

    if (std::isfinite(r))
    {
      if (r >= min_range && r <= max_range)
        ldp->readings[i] = r;
      else if (r < min_range)
        ldp->readings[i] = min_range;
      else if (r > max_range)
        ldp->readings[i] = max_range;

      ldp->valid[i] = 1;
    }
    else
    {
      ldp->readings[i] = -1;  // for invalid range
      ldp->valid[i] = 0;
      //ROS_WARN("[Pipeline Localiser] Detected nan in input laser scan, beware");
    }

    ldp->cluster[i] = -1;
    ldp->theta[i] = angle_min + i * angle_inc;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[n-1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
}


/*******************************************************************************
 * @brief Converts a LDP structure to a LaserScan laser scan
 * @param[in] ldp [const LDP&] The input LDP laser scan
 * @return [sensor_msgs::LaserScan::Ptr] The output LaserScan laser scan
 */
sensor_msgs::LaserScan::Ptr
PipelineLocalisation::ldpTolaserScan(const LDP& ldp)
{
  sensor_msgs::LaserScan::Ptr laser_scan =
    boost::make_shared<sensor_msgs::LaserScan>(*latest_world_scan_);

  laser_scan->angle_min = ldp->min_theta;
  laser_scan->angle_max = ldp->max_theta;

  unsigned int n = ldp->nrays;

  laser_scan->angle_increment =
    (laser_scan->angle_max - laser_scan->angle_min) / n;

  laser_scan->ranges.resize(n);

  for (unsigned int i = 0; i < n; i++)
  {
    if (ldp->valid[i] == 1)
      laser_scan->ranges[i] = ldp->readings[i];
    else
      laser_scan->ranges[i] = 0.0;
  }

  return laser_scan;
}


/*******************************************************************************
 * @brief Stores the map upon receipt. (The map does not change through time)
 * @param[in] map_msg [const nav_msgs::OccupancyGrid] The map
 * @return void
 */
void
PipelineLocalisation::mapCallback(const nav_msgs::OccupancyGrid& map_msg)
{
  map_ = map_msg;
  received_map_ = true;
}


/*******************************************************************************
 * @brief Publishes the pipeline's latest execution time.
 * @param[in] start [const ros::Time&] The start time of the pipeline's
 * execution
 * @param[in] end [const ros::Time&] The end time of the pipeline's execution
 */
void
PipelineLocalisation::measureExecutionTime(
  const ros::Time& start,
  const ros::Time& end)
{
  ros::Duration d = end-start;
  std_msgs::Duration duration_msg;
  duration_msg.data = d;
  pipeline_execution_times_publisher_.publish(duration_msg);
}


/*******************************************************************************
 * @brief Checks if there are nan's in an input pose.
 * @param[in] pose_msg [const geometry_msgs::PoseWithCovarianceStamped::Ptr&]
 * The input pose
 * @return [bool] True if there is at least one nan in the input_pose, false
 * otherwise.
 */
bool
PipelineLocalisation::nanInPose(
  const geometry_msgs::PoseWithCovarianceStamped::Ptr& pose_msg)
{
  if (std::isnan(pose_msg->pose.pose.position.x) ||
      std::isnan(pose_msg->pose.pose.position.y) ||
      std::isnan(pose_msg->pose.pose.orientation.z) ||
      std::isnan(pose_msg->pose.pose.orientation.w))
    return true;
  else
    return false;
}


/*******************************************************************************
 * @brief Returns the number of rays that correspond to an angle range
 * based on known values of minimum and maximum angle, and the number of rays
 * that correspond to them.
 * @param[in] angle_min [const double&] The minimum angle
 * @param[in] angle_max [const double&] The maximum angle
 * @param[in] num_rays [const int&] The number of rays that correspond to the
 * interval [angle_min, angle_max]
 * @param[in] new_range [const double&] The angle range over which we seek the
 * number of rays
 * @return [int] The number of rays corresponding to new_range
 */
int
PipelineLocalisation::numRaysFromAngleRange(
  const double& angle_min,
  const double& angle_max,
  const int& num_rays,
  const double& new_range)
{
  double v = num_rays * new_range / (angle_max - angle_min);
  int v_int = static_cast<int>(v);

  // We shall return an even number of rays either way
  if (v_int % 2 == 0)
    return v;
  else
    return ceil(v);
}


/*******************************************************************************
 * @brief The odometry callback. Used to determine if the robot is on the move
 * or not.
 * @param[in] odom_msg [const nav_msgs::Odometry&] The odometry message.
 * @return void
 */
void
PipelineLocalisation::odomCallback(const nav_msgs::Odometry& odom_msg)
{
  if (getTotal3DVelocity(odom_msg.twist.twist) >=
    pipeline_feedback_cutoff_velocity_)
    robot_is_omw_ = true;
  else
    robot_is_omw_ = false;

  if (sqrt(odom_msg.twist.twist.linear.x * odom_msg.twist.twist.linear.x +
   odom_msg.twist.twist.linear.y * odom_msg.twist.twist.linear.y) < 0.1)
    dft_iterations_ = 0;
  else
    nh_private_.getParam("dft_iterations", dft_iterations_);
/*
*/
}


/*******************************************************************************
 * @brief The amcl pose callback. This is the point of entry to the pipeline
 * operation
 * @param[in] pose_msg [const geometry_msgs::PoseWithCovarianceStamped::Ptr&]
 * The amcl pose wrt to the map frame
 * @return void
 */
void
PipelineLocalisation::poseCallback(
  const geometry_msgs::PoseWithCovarianceStamped::Ptr& pose_msg)
{
  handleInputPose(pose_msg);
}


/*******************************************************************************
 * @brief Processes two scans (a world scan and a map scan) in LDP form prior
 * to the DFT operation.
 * @param[in,out] scan_1 [LDP&] The world scan
 * @param[in,out] scan_2 [LDP&] The map scan
 * @return [void]
 */
void
PipelineLocalisation::preprocessDFTScans(LDP& scan_1, LDP& scan_2)
{
  // Fill in the rest of the 360 - (max_theta - min_theta) angles with fake
  // values for the sake of completing 2π, which is needed for the fourier
  // transform that follows
  if (dft_do_fill_map_scan_)
    fillLaserScanLDP(scan_2, scan_1, op_dft_);

  // Undersample range scans
  if (dft_scan_undersample_rate_ > 1)
    undersampleLDPScans(scan_1, scan_2, dft_scan_undersample_rate_);

  // Discard rays for which ||scan_1(i) - scan_2(i)|| >= ε applies
  if (dft_do_invalidate_diff_rays_)
    invalidateDiffRays(scan_1, scan_2,
      dft_invalidate_diff_rays_epsilon_, op_dft_);
}


/*******************************************************************************
 * @brief Processes two scans (a world scan and a map scan) in LDP form prior
 * to the ICP operation.
 * @param[in,out] scan_1 [LDP&] The world scan
 * @param[in,out] scan_2 [LDP&] The map scan
 * return void
 */
void
PipelineLocalisation::preprocessICPScans(LDP& scan_1, LDP& scan_2)
{
  // If the map scan is over 2π, then fill the world scan as well, so that
  // invalidation of different scan rays is possible
  if (icp_do_fill_map_scan_)
    fillLaserScanLDP(scan_2, scan_1, op_icp_);

  // Only incorporate measurements whose range is lower than icp_clip_sill_
  // and higher than icp_clip_lintel_
  if (icp_do_clip_scans_)
    invalidateByClippingScans(scan_1, scan_2,
      icp_clip_lintel_, icp_clip_sill_);

  // Undersample range scans
  if (icp_scan_undersample_rate_ > 1)
    undersampleLDPScans(scan_1, scan_2, icp_scan_undersample_rate_);

  // Invalidate diff rays
  if (icp_do_invalidate_diff_rays_)
    invalidateDiffRays(scan_1, scan_2,
      icp_invalidate_diff_rays_epsilon_, op_icp_);
}


/*******************************************************************************
 * @brief The champion function of the ICP operation.
 * @param[in] world_scan_ldp [LDP&] The world scan in LDP form.
 * @param[in] map_scan_ldp [LDP&] The map scan in LDP form.
 * @return void
 */
void
PipelineLocalisation::processScan(
  LDP& world_scan_ldp,
  LDP& map_scan_ldp)
{
  ros::WallTime start = ros::WallTime::now();

  // CSM is used in the following way:
  // The scans are always in the laser frame
  // The reference scan (prevLDPcan_) has a pose of [0, 0, 0]
  // The new scan (currLDPScan) has a pose equal to the movement
  // of the laser in the laser frame since the last scan
  // The computed correction is then propagated using the tf machinery

  map_scan_ldp->odometry[0] = 0.0;
  map_scan_ldp->odometry[1] = 0.0;
  map_scan_ldp->odometry[2] = 0.0;

  map_scan_ldp->estimate[0] = 0.0;
  map_scan_ldp->estimate[1] = 0.0;
  map_scan_ldp->estimate[2] = 0.0;

  map_scan_ldp->true_pose[0] = 0.0;
  map_scan_ldp->true_pose[1] = 0.0;
  map_scan_ldp->true_pose[2] = 0.0;

  input_.laser_ref  = map_scan_ldp;
  input_.laser_sens = world_scan_ldp;

  input_.first_guess[0] = 0.0;
  input_.first_guess[1] = 0.0;
  input_.first_guess[2] = 0.0;

  // If they are non-Null, free covariance gsl matrices to avoid leaking memory
  if (output_.cov_x_m)
  {
    gsl_matrix_free(output_.cov_x_m);
    output_.cov_x_m = 0;
  }
  if (output_.dx_dy1_m)
  {
    gsl_matrix_free(output_.dx_dy1_m);
    output_.dx_dy1_m = 0;
  }
  if (output_.dx_dy2_m)
  {
    gsl_matrix_free(output_.dx_dy2_m);
    output_.dx_dy2_m = 0;
  }

  // *** scan match - using point to line icp from CSM

  sm_icp(&input_, &output_);

  if (output_.valid)
  {
    // the correction of the laser's position, in the laser frame
    tf::Transform corr_ch_l;
    createTfFromXYTheta(output_.x[0], output_.x[1], output_.x[2], corr_ch_l);

    // the correction of the base's position, in the base frame
    f2b_ = base_to_laser_ * corr_ch_l * laser_to_base_;
  }
  else
  {
    f2b_.setIdentity();
    ROS_WARN("[Pipeline Localiser] Error in scan matching");
  }

  // **** statistics

  double dur = (ros::WallTime::now() - start).toSec() * 1e3;
  ROS_DEBUG("[Pipeline Localiser] Scan matcher total duration: %.1f ms", dur);
}


/*******************************************************************************
 * @brief Calculate the distance between the endpoints of two rays whose
 * in-between angle has a cosine equal to `cos_angle`
 * @param[in] r1 [const  double&] The first laser range distance
 * @param[in] r2 [const  double&] The second laser range distance
 * @param[in] cos_angle [const  double&] The cosine of the angle between r1
 * and r2
 * @return [double] The distance between the endpoints of r1 and r2
 */
double
PipelineLocalisation::raysEndpointDistance(
  const double& r1,
  const double& r2,
  const double& cos_angle)
{
  return sqrt(r1*r1 + r2*r2 - 2*r1*r2*cos_angle);
}


/*******************************************************************************
 * @brief Transforms a single ray from a range scan into a point in the
 * laser_pose's frame of reference.
 * @param[in] laser_pose [const geometry_msgs::Pose&] The pose of the laser in
 * the map frame.
 * @param[in] scan [const sensor_msgs::LaserScan::Ptr&] The input scan
 * @param[in] ray_id [const int&] The index of the ray within the input scan
 * ranges structure.
 * @return [std::pair<double,double>] The ray as a point (x,y)
 */
std::pair<double,double>
PipelineLocalisation::rayToPoint(
  const geometry_msgs::Pose& laser_pose,
  const sensor_msgs::LaserScan::Ptr& scan,
  const int& ray_id)
{
  // Extract yaw from pose
  double yaw = extractYawFromPose(laser_pose);

  // Calculate the position of end-point of the ray
  double r_x = laser_pose.position.x + scan->ranges[ray_id] *
    cos(yaw + (scan->angle_min + ray_id * scan->angle_increment));
  double r_y = laser_pose.position.y + scan->ranges[ray_id] *
    sin(yaw + (scan->angle_min + ray_id * scan->angle_increment));

  // Return point
  std::pair<double,double> p(r_x,r_y);
  return p;
}


/*******************************************************************************
 * @brief The laser scan callback
 * @param[in] scan_msg [const sensor_msgs::LaserScan::Ptr&] The input scan
 * message
 * @return void
 */
void
PipelineLocalisation::scanCallback(
  const sensor_msgs::LaserScan::Ptr& scan_msg)
{
  // **** if first scan, cache the tf from base to the scanner

  if (!received_scan_)
  {
    createCache(scan_msg);    // caches the sin and cos of all angles

    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg->header.frame_id))
    {
      ROS_WARN("[Pipeline Localiser] Skipping scan");
      return;
    }

    received_scan_ = true;
  }

  // Store the latest scan
  //boost::mutex::scoped_lock lock(mutex_);
  latest_world_scan_ =
    boost::make_shared<sensor_msgs::LaserScan>(*scan_msg);
}


/*******************************************************************************
 * @brief Given the robot's pose and a choice to scan over an angle of 2π,
 * this function simulates a range scan that has the physical world substituted
 * for the map.
 * @param[in] robot_pose [const geometry_msgs::PoseWithCovarianceStamped::Ptr&]
 * The robot's pose.
 * @param[in] scan_method [const std::string&] Which method to use for
 * scanning the map. Currently supports vanilla (Bresenham's method)
 * and ray_marching.
 * @param[in] do_fill_map_scan [const bool&] A choice to scan over an angle of
 * 2π.
 * @return [sensor_msgs::LaserScan::Ptr] The map scan in LaserScan form.
 */
sensor_msgs::LaserScan::Ptr
PipelineLocalisation::scanMap(
  const geometry_msgs::PoseWithCovarianceStamped::Ptr& robot_pose,
  const std::string& scan_method,
  const bool& do_fill_map_scan)
{
  #if DEBUG_EXECUTION_TIMES == 1
  ros::Time start = ros::Time::now();
  #endif

  // Get the laser's pose: the map scan needs that one, not the pose of the
  // robot!
  const geometry_msgs::Pose current_laser_pose =
    getCurrentLaserPose(robot_pose->pose.pose);

  sensor_msgs::LaserScan::Ptr laser_scan_info =
    boost::make_shared<sensor_msgs::LaserScan>(*latest_world_scan_);

  // Scan the map over 2π -- we get more information that way
  if (do_fill_map_scan)
  {
    laser_scan_info->angle_min = -M_PI;
    laser_scan_info->angle_max = M_PI;
  }


  sensor_msgs::LaserScan::Ptr map_scan;
  double min_a = laser_scan_info->angle_min;
  double inc_a = laser_scan_info->angle_increment;

  // Scan the map using the occupancy_grid_utils method
  if (scan_method.compare("vanilla") == 0)
  {
    map_scan = occupancy_grid_utils::simulateRangeScan(map_, current_laser_pose,
        *laser_scan_info);
  }
  else if (scan_method.compare("ray_marching") == 0 ||
           scan_method.compare("bresenham") == 0 ||
           scan_method.compare("cddt") == 0)
  {
    // Convert laser position to grid coordinates
    float x = current_laser_pose.position.x / map_.info.resolution;
    float y = map_.info.height - 1 -
      current_laser_pose.position.y / map_.info.resolution;
    float a = extractYawFromPose(current_laser_pose);

    // How many rays?
    int num_rays = numRaysFromAngleRange(latest_world_scan_->angle_min,
      latest_world_scan_->angle_max, laser_scan_info->ranges.size(),
      laser_scan_info->angle_max - laser_scan_info->angle_min);

    map_scan = boost::make_shared<sensor_msgs::LaserScan>(*laser_scan_info);
    map_scan->ranges.clear();

    // The angular progression of scanning depends on how the laser is mounted
    // on the robot:
    // a. On the turtlebot the laser faces upwards;
    // b. On the rb1 the laser faces downwards
    int sgn = 0;
    if (laser_z_orientation_.compare("upwards") == 0)
      sgn = -1;
    else if (laser_z_orientation_.compare("downwards") == 0)
      sgn = 1;
    else
    {
      ROS_ERROR("[Pipeline Localiser] Please provide a valid value");
      ROS_ERROR("                     for param laser_z_orientation");
    }

    // Iterate over all angles.
    // Calculate range for every angle (calc_range returns range in pixels).
    // The angles start counting negatively.
    // For details see
    // https://github.com/kctess5/range_libc/blob/deploy/docs/RangeLibcUsageandInformation.pdf

    if (scan_method.compare("ray_marching") == 0)
    {
      for (unsigned int i = 0; i < num_rays; i++)
        map_scan->ranges.push_back(map_.info.resolution *
          rm_.calc_range(x,y, -a + sgn * (min_a + i * inc_a)));
    }
    else if (scan_method.compare("cddt") == 0)
    {
      for (unsigned int i = 0; i < num_rays; i++)
        map_scan->ranges.push_back(map_.info.resolution *
          cddt_.calc_range(x,y, -a + sgn * (min_a + i * inc_a)));
    }
    else if (scan_method.compare("bresenham") == 0)
    {
      for (unsigned int i = 0; i < num_rays; i++)
        map_scan->ranges.push_back(map_.info.resolution *
          br_.calc_range(x,y, -a + sgn * (min_a + i * inc_a)));
    }
  }
  else // Default to vanilla
  {
    map_scan = occupancy_grid_utils::simulateRangeScan(map_, current_laser_pose,
        *laser_scan_info);
  }

  map_scan->header.stamp = robot_pose->header.stamp;

  #if DEBUG_EXECUTION_TIMES == 1
  ROS_ERROR("scanMap() took %.2f ms", (ros::Time::now() - start).toSec() * 1000);
  #endif

  return map_scan;
}


/*******************************************************************************
 */
void
PipelineLocalisation::testICPCircle()
{
  // Let the robot be placed at (c0_x, c0_y). (This is its true position)
  // Make a circle at (c0_x, c0_y) of radius r consisting of num_rays points
  // starting from starting_angle + theta (just like the turtlebot's scan).
  double theta = 0.0 * M_PI / 180;
  double angle_min = -130 * M_PI / 180;
  double angle_span = -2 * angle_min;
  double c0_x = 0.0;
  double c0_y = 0.0;
  double r = 5.0;
  int num_rays = 640;
  std::vector<std::pair<double,double> > circle;
  for (unsigned int i = 0; i < num_rays; i++)
  {
    double x = c0_x + r * cos( angle_span*i / num_rays + angle_min + theta );
    double y = c0_y + r * sin( angle_span*i / num_rays + angle_min + theta );

    std::pair<double,double> p(x,y);
    circle.push_back(p);
  }


  // Construct a world laser scan
  sensor_msgs::LaserScan::Ptr world_scan =
    boost::make_shared<sensor_msgs::LaserScan>();

  world_scan->angle_min = angle_min;
  world_scan->angle_min = -angle_min;
  world_scan->angle_increment = angle_span / num_rays;
  world_scan->range_min = 0.8;
  world_scan->range_max = 10.0;

  // calculate ranges from (c0_x, c0_y)
  for (unsigned int i = 0; i < num_rays; i++)
  {
    double xs = (c0_x - circle[i].first) * (c0_x - circle[i].first);
    double ys = (c0_y - circle[i].second) * (c0_y - circle[i].second);
    world_scan->ranges.push_back(sqrt(xs + ys));
  }

  // The estimated pose
  double e_x = 1.00;
  double e_y = 0.00;
  double e_a = 0.0 * M_PI / 180;

  // Construct a map laser scan
  sensor_msgs::LaserScan::Ptr map_scan =
    boost::make_shared<sensor_msgs::LaserScan>();

  map_scan->angle_min = angle_min;
  map_scan->angle_min = -angle_min;
  map_scan->angle_increment = angle_span / num_rays;
  map_scan->range_min = 0.8;
  map_scan->range_max = 10.0;


  // construct an arc from A_1 to A_2, where A_{1,2} are the points where the
  // original circle intersects the starting and ending rays of the robot if
  // it were at the estimated position

  // For the first ray
  double l = tan(e_a + angle_min);
  double a = 1 + l*l;
  double b = 2*l*(e_y-c0_y) -2*c0_x - 2*l*l*e_x;
  double c =
    c0_x*c0_x + (e_y-c0_y)*(e_y-c0_y) - 2*l*e_x*(e_y-c0_y) + l*l*e_x*e_x - r*r;
  double D = b*b - 4*a*c;
  double start_angle_1 = 0.0;
  double end_angle_1 = 0.0;
  if (D >= 0)
  {
    double x_1 = (-b + sqrt(D)) / (2*a);
    double x_2 = (-b - sqrt(D)) / (2*a);
    double y_1 = e_y + l * (x_1 - e_x);
    double y_2 = e_y + l * (x_2 - e_x);
    start_angle_1 = atan2((y_1 - c0_y), (x_1 - c0_x));
    end_angle_1 = atan2((y_2 - c0_y), (x_2 - c0_x));
  }
  else
    ROS_ERROR("NOOOO");

  // For the last ray
  l = tan(e_a + angle_min + angle_span);
  a = 1 + l*l;
  b = 2*l*(e_y-c0_y) -2*c0_x - 2*l*l*e_x;
  c =
    c0_x*c0_x + (e_y-c0_y)*(e_y-c0_y) - 2*l*e_x*(e_y-c0_y) + l*l*e_x*e_x - r*r;
  D = b*b - 4*a*c;
  double start_angle_2 = 0.0;
  double end_angle_2 = 0.0;
  if (D >= 0)
  {
    double x_1 = (-b + sqrt(D)) / (2*a);
    double x_2 = (-b - sqrt(D)) / (2*a);
    double y_1 = e_y + l * (x_1 - e_x);
    double y_2 = e_y + l * (x_2 - e_x);
    start_angle_2 = atan2((y_1 - c0_y), (x_1 - c0_x));
    end_angle_2 = atan2((y_2 - c0_y), (x_2 - c0_x));
  }
  else
    ROS_ERROR("NOOOO");

  double start_angle = std::min(start_angle_1, end_angle_1);
  double end_angle = std::max(start_angle_2, end_angle_2);
  double span = end_angle - start_angle;

  ROS_ERROR("start angle = %f", start_angle * 180 / M_PI);
  ROS_ERROR("end angle = %f", end_angle * 180 / M_PI);


  // Construct a circular arc, centered at (c0_x, c0_y)
  // from starting_angle to ending_angle
  std::vector<std::pair<double,double> > map_circle;
  for (unsigned int i = 0; i < num_rays; i++)
  {
    double x = c0_x + r * cos( span*i / num_rays + start_angle );
    double y = c0_y + r * sin( span*i / num_rays + start_angle );

    std::pair<double,double> p(x,y);
    map_circle.push_back(p);
  }

  // calculate ranges from (e_x, e_y)
  for (unsigned int i = 0; i < num_rays; i++)
  {
    double xs = (e_x - map_circle[i].first) * (e_x - map_circle[i].first);
    double ys = (e_y - map_circle[i].second) * (e_y - map_circle[i].second);
    map_scan->ranges.push_back(sqrt(xs + ys));
  }

  // process
  LDP world_scan_ldp;
  laserScanToLDP(world_scan, world_scan_ldp);
  LDP map_scan_ldp;
  laserScanToLDP(map_scan, map_scan_ldp);

  processScan(world_scan_ldp, map_scan_ldp);
  geometry_msgs::Pose vec;
  tf::poseTFToMsg(f2b_, vec);
  ROS_ERROR("x = %f", vec.position.x);
  ROS_ERROR("y = %f", vec.position.y);
  ROS_ERROR("a = %f", extractYawFromPose(vec) * 180 / M_PI);

}


/*******************************************************************************
 * @brief Transforms a laser scan into the points of its ray-ends. Used to
 * visualise the transform of the map_scan and compare it against the world scan
 * @param[in] amcl_pose [const geometry_msgs::PoseWithCovarianceStamped::Ptr&]
 * The robot's pose
 * @param[in] scan [const sensor_msgs::LaserScan::Ptr&] The laser scan
 * (should be a map scan)
 * @param[in] transform [const tf::Transform&] The transform from the map scan
 * to the world scan
 * @return [geometry_msgs::PoseArray] An array of poses. The poses are the points
 * of the end of the rays of the map scan in the map frame.
 */
geometry_msgs::PoseArray
PipelineLocalisation::transformLaserScan(
  const geometry_msgs::PoseWithCovarianceStamped::Ptr& amcl_pose,
  const sensor_msgs::LaserScan::Ptr& scan,
  const tf::Transform& transform)
{
  // Get the laser pose: the scan is expressed in the laser frame
  geometry_msgs::Pose laser_pose = getCurrentLaserPose(amcl_pose->pose.pose);

  // The scan transformed into an array of poses
  geometry_msgs::PoseArray corrected_poses;
  corrected_poses.header.frame_id = fixed_frame_;

  for (unsigned int i = 0; i < scan->ranges.size(); i++)
  {
    // Transform ray to point in map frame
    std::pair<double,double> p = rayToPoint(laser_pose, scan, i);

    geometry_msgs::Pose ray_pose;
    ray_pose.position.x = p.first;
    ray_pose.position.y = p.second;
    ray_pose.orientation.x = 0.0;
    ray_pose.orientation.y = 0.0;
    ray_pose.orientation.z = 0.0;
    ray_pose.orientation.w = 1.0;

    tf::Transform tr;
    tf::poseMsgToTF(ray_pose, tr);

    tf::Transform point_corrected_tf = tr * transform;

    geometry_msgs::Pose pose_corrected;
    tf::poseTFToMsg(point_corrected_tf, pose_corrected);
    corrected_poses.poses.push_back(pose_corrected);
  }

  return corrected_poses;
}


/*******************************************************************************
 * @brief Given the first coefficient of the DFT, this function turns them into
 * errors in the x and y directions.
 * @param[in] dft_coeff [const std::vector<double>&] A vector holding the real
 * part of the first coefficient of the dft in its first position, and the
 * imaginary in the second.
 * @param[in] num_valid_rays [const int&] The number of jointly valid rays.
 * @param[in] starting_angle [const double&] The orientation of the robot.
 * return [std::vector<double>] The errors in the x (in the first position)
 * and y (in the second position) directions.
 */
std::vector<double>
PipelineLocalisation::turnDFTCoeffsIntoErrors(
  const std::vector<double>& dft_coeff,
  const int& num_valid_rays,
  const double& starting_angle)
{
  double x_err = 0.0;
  double y_err = 0.0;

  if (num_valid_rays > 0)
  {
    // The error in the x- direction
    x_err = 1.0 / num_valid_rays *
      (-dft_coeff[0] * cos(starting_angle)
       -dft_coeff[1] * sin(starting_angle));

    // The error in the y- direction
    y_err = 1.0 / num_valid_rays *
      (-dft_coeff[0] * sin(starting_angle)
       +dft_coeff[1] * cos(starting_angle));
  }

  std::vector<double> errors;
  errors.push_back(x_err);
  errors.push_back(y_err);

  return errors;
}


/*******************************************************************************
 * @brief Undersamples a LDP scan.
 * @param[in,out] scan [LDP&] The input scan
 * @param[in] rate [const int&] Take account of one out of every rate rays of
 * input scan
 * @return void
 */
void
PipelineLocalisation::undersampleLDPScan(
  LDP& scan,
  const int& rate)
{
  // The undersampled scan will have scan.nrays / rate number of rays
  int nrays = static_cast<int>(static_cast<double>(scan->nrays) / rate);

  LDP subed_scan = ld_alloc_new(nrays);

  int s = 0;
  for (unsigned int i = 0; i < nrays; i++)
  {
    subed_scan->valid[i] = scan->valid[s];
    subed_scan->theta[i] = scan->theta[s];
    subed_scan->cluster[i] = scan->cluster[s];
    subed_scan->readings[i] = scan->readings[s];

    s += rate;
  }

  subed_scan->nrays = nrays;
  subed_scan->min_theta = subed_scan->theta[0];
  subed_scan->max_theta = subed_scan->theta[nrays-1];

  subed_scan->odometry[0] = scan->odometry[0];
  subed_scan->odometry[1] = scan->odometry[1];
  subed_scan->odometry[2] = scan->odometry[2];

  subed_scan->true_pose[0] = scan->true_pose[0];
  subed_scan->true_pose[1] = scan->true_pose[1];
  subed_scan->true_pose[2] = scan->true_pose[2];

  copyLDP(subed_scan, scan);
  ld_free(subed_scan);
}


/*******************************************************************************
 * @brief Undersamples a world and a map scan in LDP scan.
 * @param[in,out] world_scan [LDP&] A world scan in LDP form
 * @param[in,out] map_scan [LDP&] A map scan in LDP form
 * @param[in] rate [const int&] Take account of one out of every rate rays of
 * the input scans
 * @return void
 */
void
PipelineLocalisation::undersampleLDPScans(
  LDP& world_scan,
  LDP& map_scan,
  const int& rate)
{
  undersampleLDPScan(world_scan, rate);
  undersampleLDPScan(map_scan, rate);
}


/*******************************************************************************
 * @brief Checks whether a value exists within the ranges structure of a
 * geometry_msgs::LaserScan laser scan.
 * @param[in] scan [const sensor_msgs::LaserScan&] The scan
 * @param[in] value [const double&] The value to be searched in the scan
 * @return [bool]
 */
bool
PipelineLocalisation::valueExistsInLaserScan(
  const sensor_msgs::LaserScan::Ptr& scan,
  const double& value)
{
  bool value_exists = false;

  for (std::vector<float>::iterator it = scan->ranges.begin();
    it != scan->ranges.end(); it++)
  {
    if (*it == value)
      value_exists = true;
  }

  return value_exists;
}

/*******************************************************************************
 * @brief Visualisation of world and map scans
 * @param[in] world_scan [const LDP&] The world scan in LDP form
 * @param[in] map_scan [const LDP&] The map scan in LDP form
 * @return void
 */
void
PipelineLocalisation::visualiseScans(
  const LDP& world_scan,
  const LDP& map_scan)
{
  sensor_msgs::LaserScan world_laser_scan = *ldpTolaserScan(world_scan);
  sensor_msgs::LaserScan map_laser_scan = *ldpTolaserScan(map_scan);

  pipeline_world_scan_publisher_.publish(world_laser_scan);
  pipeline_map_scan_publisher_.publish(map_laser_scan);
}


/*******************************************************************************
 * @brief Wraps an angle in the [-π, π] interval
 * @param[in,out] angle [double&] The angle to be expressed in [-π,π]
 * @return void
 */
void
PipelineLocalisation::wrapAngle(double& angle)
{
  angle = fmod(angle + 5*M_PI, 2*M_PI) - M_PI;
}

/*******************************************************************************
 * @brief Wraps a pose's orientation in the [-π,π] interval
 * @param[in,out] pose [geometry_msgs::PoseWithCovarianceStamped::Ptr&] The pose
 * whose input will be wrapped in [-π,π]
 * @return void
 */
void
PipelineLocalisation::wrapPoseOrientation(
  geometry_msgs::PoseWithCovarianceStamped::Ptr& pose)
{
  // Extract yaw ...
  double yaw = extractYawFromPose(pose->pose.pose);

  // ... and wrap between [-π, π]
  wrapAngle(yaw);

  tf::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  tf::quaternionTFToMsg(q, pose->pose.pose.orientation);
}
