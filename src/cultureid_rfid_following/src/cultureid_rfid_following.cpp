#include "cultureid_rfid_following.h"

/*******************************************************************************
 * Constructor. Initializes params / subscribers.
 */
CRFIDFollow::CRFIDFollow(void):
  received_scan_(false),
  received_map_(false),
  do_autorotation_(false),
  is_goal_active_(false),
  in_goal_radius_(false),
  poses_counter_(0),
  move_base_goal_client_("move_base", true),
  latest_orientation_(0.0),
  latest_reliable_orientation_(0.0),
  latest_rssi_(0.0),
  omap_(ranges::OMap(1,1)),
  rm_(ranges::RayMarching(omap_, 1))
{
  loadParams();

  // Reads the latest desired yaw and publishes a suitable goal to move_base
  periodic_callback_timer_ = nodehandle_.createTimer(
    ros::Duration(callback_frequency_), &CRFIDFollow::periodicCallback, this);

  // Dummy subscriber for early tests. The orientation is requested via a
  // service at the moment (22/06/2021)
  orientation_subscriber_ = nodehandle_.subscribe("aoa_orientation_estimate", 1,
    &CRFIDFollow::aoaOrientationCallback, this);

  // The publisher of the free goal; for visualisation purposes
  free_goal_pub_ = nodehandle_.advertise<geometry_msgs::PoseStamped>(
    "aoa_free_goal", 1, true);

  // The publisher to move_base; sends goal
  goal_pub_ = nodehandle_.advertise<geometry_msgs::PoseStamped>(
    "/move_base_simple/goal", 1);

  // The publisher to move_base; aborts goal
  goal_abortion_pub_ = nodehandle_.advertise<actionlib_msgs::GoalID>(
    "/move_base/cancel", 1);

  // When the rfid tag is lost, this publisher publishes an rfid detection
  // initiation message
  request_lost_tag_detection_pub_ = nodehandle_.advertise<std_msgs::Empty>(
    request_lost_tag_topic_, 1);

  // Publishes velocity commands for autorotation
  velocity_pub_ = nodehandle_.advertise<geometry_msgs::Twist>(
    velocity_topic_, 1);

  // Visualisation of all available feasible goals
  feasible_goals_vis_pub_ = nodehandle_.advertise<visualization_msgs::MarkerArray>(
    "feasible_goals_vis", 1000, true);

  // When the rfid tag is found, this subscriber is called in order for the
  // robot to stop autorotating
  lost_tag_detected_sub_ = nodehandle_.subscribe(lost_tag_detected_topic_, 1,
    &CRFIDFollow::lostTagDetectedCallback, this);

  // global_planner's make_plan client and not move_base's.
  // See  https://github.com/ros-planning/navigation/issues/12  for why not
  // Requests a plan from A to B. If B is a
  // feasible configuration the robot may move there.
  // ** It does not actually send the goal, it just asks for the plan **
  // We register global_planner's make_plan service and not move_base's because
  // "move_base must be in an inactive state to make a plan for an external
  // user", and therefore if you call its service rather than global_planner's
  // while move_base is navigating towards a goal, then it will not take it into
  // consideration
  //make_plan_client_ = nodehandle_.serviceClient<nav_msgs::GetPlan>(
    //"move_base/NavfnROS/make_plan");
    //"move_base/GlobalPlanner/make_plan");
  //make_plan_client_.waitForExistence();


  // AoA service client
  aoa_client_ = nodehandle_.serviceClient<cultureid_rfid_following::AoASrv>(
    "cultureid_rfid_aoa_server_node_aoa_srv");
  aoa_client_.waitForExistence();

  // Clear costmaps service client
  clear_costmaps_client_ = nodehandle_.serviceClient<std_srvs::Empty>(
    "/move_base/clear_costmaps");
  clear_costmaps_client_.waitForExistence();

  // Subscribes to the robot's pose
  pose_sub_ = nodehandle_.subscribe(pose_topic_, 10,
    &CRFIDFollow::poseCallback, this);

  // Subscribes to the robot's velocity
  velocity_sub_ = nodehandle_.subscribe(velocity_topic_, 1,
    &CRFIDFollow::velocityCallback, this);

  // Subscribes to the lidar's measurements
  scan_sub_ = nodehandle_.subscribe(scan_topic_, 1,
    &CRFIDFollow::scanCallback, this);

  // Subscribes to the map
  map_sub_ = nodehandle_.subscribe(map_topic_, 1,
    &CRFIDFollow::mapCallback, this);

  // Publish feasible goals for visualisation
  publishAllFeasibleGoalsMarkers();

  initLogfiles();

  // Store into a file the polarity of each antenna of each reader, e.g.
  // if it faces left, its polarity is positive (+)
  // if it faces right, its polarity is negative (-)
  // (we use the robot's native reference frame for reference)
  //writeReadersAntennasPolarities();

  // Re-set the map png file
  omap_ = ranges::OMap(map_png_file_);

  ROS_INFO("[CRFIDFollow] Node initialised");
}


/*******************************************************************************
 * Destructor
 */
CRFIDFollow::~CRFIDFollow(void)
{
  ROS_INFO("[CRFIDFollow] Node destroyed");
}


/*******************************************************************************
 * Aborts a goal if the robot distance to the RFID tag fulfills certain
 * criteria. No checks for these criteria are performed here; only the goal
 * abortion is made
 */
void CRFIDFollow::abortGoal()
{
  actionlib_msgs::GoalID msg;
  msg.stamp = ros::Time::now();
  msg.id = "";

  goal_abortion_pub_.publish(msg);

  ROS_INFO("[CRFIDFollow] GOAL ABORTED");
}


/*******************************************************************************
 * Dummy orientation estimate callback for tests
 */
void CRFIDFollow::aoaOrientationCallback(const std_msgs::Float64& msg)
{
  ROS_INFO("[CRFIDFollow] Got %f degrees", msg.data);

  aoa_estimates_.push_back(msg.data);
}


/*******************************************************************************
 * Associate a free goal to a feasible goal. Return the index of the feasible
 * goal within the feasible_goals vector
 */
unsigned int CRFIDFollow::associateFreeGoalToFeasibleGoal(
  const std::pair<double,double>& free_goal,
  const std::vector< std::pair<double,double> >& feasible_goals)
{
  // The index of the feasible goal that has the least distance to the free
  // goal. This is what is going to be returned
  unsigned int min_idx = std::numeric_limits<unsigned int>::max();


  double min_dist = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < feasible_goals.size(); i++)
  {
    // Do not consider the current goal being an available next goal. The robot
    // might get stuck to the same pose over and over
    if (i != current_feasible_goal_idx_)
    {
      double dist_i = pairSQDistance(free_goal, feasible_goals[i]);
      if (dist_i < min_dist)
      {
        min_dist = dist_i;
        min_idx = i;
      }
    }
  }

  return min_idx;
}


/*******************************************************************************
 * When the tag is lost perform an in-place rotation until you find the tag
 * again
 */
void CRFIDFollow::autorotate()
{
  do_autorotation_ = true;

  ROS_INFO("[CRFIDFollow] Aborting Goal");
  abortGoal();

  ROS_INFO("[CRFIDFollow] Requesting lost tag be found");
  requestLostTagDetection();

  // Compute which way to rotate
  unsigned int num_estimates = aoa_estimates_.size();
  double sign = 1.0;
  if (num_estimates >= 3)
  {
    sign = aoa_estimates_[num_estimates-3];

    // Saturate sign
    if (sign >= 0)
      sign = 1;
    if (sign < 0)
      sign = -1;
  }

  // Autorotation twist msg
  geometry_msgs::Twist autorot_msg;
  autorot_msg.linear.x = 0.0;
  autorot_msg.linear.y = 0.0;
  autorot_msg.linear.z = 0.0;
  autorot_msg.angular.x = 0.0;
  autorot_msg.angular.y = 0.0;
  autorot_msg.angular.z = sign * 0.1;

  ROS_INFO("[CRFIDFollow] Initiating autorotation");
  while(do_autorotation_)
    velocity_pub_.publish(autorot_msg);
}


/*******************************************************************************
 * Route planning result callback
 */
std::vector<geometry_msgs::PoseStamped> CRFIDFollow::calculateAntennaPose(
  const geometry_msgs::PoseWithCovarianceStamped& robot_pose)
{
  // Robot pose as tf
  tf::Transform robot_pose_tf;

  robot_pose_tf.setOrigin(
    tf::Vector3(
      robot_pose.pose.pose.position.x,
      robot_pose.pose.pose.position.y,
      robot_pose.pose.pose.position.z));

  // Turn quaternion into yaw
  tf::Quaternion q_r(
    robot_pose.pose.pose.orientation.x,
    robot_pose.pose.pose.orientation.y,
    robot_pose.pose.pose.orientation.z,
    robot_pose.pose.pose.orientation.w);

  robot_pose_tf.setRotation(q_r);

  std::vector<geometry_msgs::PoseStamped> return_vector;

  // Create pose per antenna
  for (int a = 0; a < 2; a++)
  {
    if (readers_active_antennas_[0][a])
    {
      // Relative antenna pose as tf
      tf::Transform antenna_pose_tf_rel;

      antenna_pose_tf_rel.setOrigin(
        tf::Vector3(
          reader_antenna_poses_[0][a][0],
          reader_antenna_poses_[0][a][1],
          reader_antenna_poses_[0][a][2]));

      tf::Quaternion q_a;
      q_a.setRPY(0.0, 0.0, 0.0);
      q_a.normalize();
      antenna_pose_tf_rel.setRotation(q_a);

      // Absolute antenna pose as tf
      tf::Transform antenna_pose_tf_abs = robot_pose_tf * antenna_pose_tf_rel;

      geometry_msgs::PoseStamped antenna_pose;
      antenna_pose.header.stamp = robot_pose.header.stamp;
      antenna_pose.pose.position.x = antenna_pose_tf_abs.getOrigin().getX();
      antenna_pose.pose.position.y = antenna_pose_tf_abs.getOrigin().getY();
      antenna_pose.pose.position.z = antenna_pose_tf_abs.getOrigin().getZ();

      antenna_pose.pose.orientation.x = antenna_pose_tf_abs.getRotation().getX();
      antenna_pose.pose.orientation.y = antenna_pose_tf_abs.getRotation().getY();
      antenna_pose.pose.orientation.z = antenna_pose_tf_abs.getRotation().getZ();
      antenna_pose.pose.orientation.w = antenna_pose_tf_abs.getRotation().getW();

      return_vector.push_back(antenna_pose);



      /*
         tf::Quaternion q_a_abs(
         antenna_pose.orientation.x,
         antenna_pose.orientation.y,
         antenna_pose.orientation.z,
         antenna_pose.orientation.w);

         tf::Matrix3x3 m(q_a_abs);
         double roll, pitch, yaw;
         m.getRPY(roll, pitch, yaw);

         std::string filename =
         pose_filename_ + "_reader_" + readers_macs_[r] + "_antenna_" + std::to_string(a+1) + ".txt";

         std::ofstream pose_file(filename.c_str(), std::ios::app);

         if (pose_file.is_open())
         {
         pose_file << msg.header.stamp.sec;
         pose_file << ".";
         pose_file << nsec_str;
         pose_file << ", ";
         pose_file << antenna_pose.position.x;
         pose_file << ", ";
         pose_file << antenna_pose.position.y;
         pose_file << ", ";
         pose_file << antenna_pose.position.z;
         pose_file << ", ";
         pose_file << yaw;
         pose_file << std::endl;

         pose_file.close();
         }
         */
    }
  }


  return return_vector;
}


/*******************************************************************************
 * Route planning result callback
 */
bool CRFIDFollow::callPlanningService(ros::ServiceClient serviceClient,
  nav_msgs::GetPlan& path_plan_srv)
{
  bool result = false;
  if (serviceClient.call(path_plan_srv))
  {
    if (!path_plan_srv.response.plan.poses.empty())
    {
      ROS_INFO("[CRFIDFollow] make_plan success!");
      result = true;
    }
    else
      ROS_WARN("[CRFIDFollow] make_plan returned empty plan");
  }
  else
    ROS_WARN("[CRFIDFollow] make_plan call failed");

  return result;
}

/*******************************************************************************
 * Constructs the vector of inputs as a service request vector
 */
cultureid_rfid_following::AoASrv CRFIDFollow::constructAOARequest()
{
  // Calculate antenna poses from the current and past robot pose
  std::vector<geometry_msgs::PoseStamped> previous_antennas_poses =
    calculateAntennaPose(previous_pose_);
  std::vector<geometry_msgs::PoseStamped> latest_antennas_poses =
    calculateAntennaPose(latest_pose_);

  // From pose stamped to a two-sized (two antennas) vector of four vector
  // elements (timestamp, x, y, theta)
  std::vector< std::vector<double> > previous_antenna_poses;
  std::vector< std::vector<double> > latest_antenna_poses;
  for (unsigned int i = 0; i < previous_antennas_poses.size(); i++)
  {
    previous_antenna_poses.push_back(
      unwrapAntennaPose(previous_antennas_poses[i]));
    latest_antenna_poses.push_back(
      unwrapAntennaPose(latest_antennas_poses[i]));
  }

  // Construct latest and previous AoAMsgs for latest and previous antenna poses
  cultureid_rfid_following::AoAMsg latest_antenna_poses_aoa;
  cultureid_rfid_following::AoAMsg previous_antenna_poses_aoa;
  for (unsigned int i = 0; i < latest_antenna_poses.size(); i++)
  {
    for (unsigned int j = 0; j < latest_antenna_poses[0].size(); j++)
    {
      latest_antenna_poses_aoa.aoa.push_back(latest_antenna_poses[i][j]);
      previous_antenna_poses_aoa.aoa.push_back(previous_antenna_poses[i][j]);
    }
  }

  // Fill also with the past estimates
  cultureid_rfid_following::AoAMsg input_aoa_estimates;

  if (aoa_estimates_.size() > 0)
    input_aoa_estimates.aoa.push_back(aoa_estimates_.back());
  else
    input_aoa_estimates.aoa.push_back(0.0);

  if (aoa_past_estimates_.size() > 0)
    input_aoa_estimates.aoa.push_back(aoa_past_estimates_.back());
  else
    input_aoa_estimates.aoa.push_back(0.0);

  // The service returned
  cultureid_rfid_following::AoASrv aoa_srv;
  aoa_srv.request.past_aoas.push_back(previous_antenna_poses_aoa);
  aoa_srv.request.past_aoas.push_back(latest_antenna_poses_aoa);
  aoa_srv.request.past_aoas.push_back(input_aoa_estimates);

  return aoa_srv;
}

/*******************************************************************************
 * @brief Given the robot's pose, an estimate of the difference in orientation
 * between the robot's orientation and the RFID tag's orientation, and a target
 * distance, this function calculates a goal for the robot that is free: the
 * goal does not fall into one of the predetermined feasible goal configurations
 * @param[in] current_pose [const geometry_msgs::PoseWithCovarianceStamped&]
 * The robot's current pose
 * @param[in] free_goal_distance [const double&] The distance at which the free
 * goal will be placed with respect to `current_pose`
 * @param[in] aoa_dyaw [const double&]: The difference in orientation between
 * the robot and the RFID tag
 * @return [std::pair<double,double>] The coordinates of the free goal
 */
std::pair<double,double> CRFIDFollow::determineFreeGoal(
  const geometry_msgs::PoseWithCovarianceStamped& current_pose,
  const double& free_goal_distance,
  const double& aoa_dyaw)
{
  // Turn quaternion into scalar orientation
  double robot_yaw = extractYawFromPose(current_pose);

  // The robot's desired yaw in the absolute frame of the map
  double desired_yaw = robot_yaw + aoa_dyaw;
  wrapAngle(desired_yaw);

  // The starting position is the robot's absolute position
  double plan_start_x = current_pose.pose.pose.position.x;
  double plan_start_y = current_pose.pose.pose.position.y;

  // Determine the free goal
  std::pair<double,double> free_goal;
  free_goal.first  = plan_start_x + free_goal_distance * cos(desired_yaw);
  free_goal.second = plan_start_y + free_goal_distance * sin(desired_yaw);

  return free_goal;
}


/*******************************************************************************
 * @brief Extracts the yaw component from the input pose's quaternion.
 * @param[in] pose [const geometry_msgs::Pose&] The input pose
 * @return [double] The pose's yaw
 */
double CRFIDFollow::extractYawFromPose(const geometry_msgs::Pose& pose)
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
 * @brief Extracts the yaw component from the input pose's quaternion.
 * @param[in] pose [const geometry_msgs::PoseWithCovarianceStamped&] The input
 *   pose
 * @return [double] The pose's yaw
 */
double CRFIDFollow::extractYawFromPose(
  const geometry_msgs::PoseWithCovarianceStamped& pose)
{
  tf::Quaternion q(
    pose.pose.pose.orientation.x,
    pose.pose.pose.orientation.y,
    pose.pose.pose.orientation.z,
    pose.pose.pose.orientation.w);

  tf::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  wrapAngle(yaw);

  return yaw;
}


/*******************************************************************************
*/
void CRFIDFollow::fillPathRequest(nav_msgs::GetPlan::Request* request,
  const double& start_x, const double& start_y,
  const double& goal_x, const double& goal_y)
{
  request->start.header.frame_id = "map";
  request->start.pose.position.x = start_x;
  request->start.pose.position.y = start_y;
  request->start.pose.orientation.w = 1.0;

  request->goal.header.frame_id = "map";
  request->goal.pose.position.x = goal_x;
  request->goal.pose.position.y = goal_y;
  request->goal.pose.orientation.w = 1.0;

  // If the goal cannot be reached, the nearest available constraint
  // The tolerance is ignored if the make_plan service is GlobalPlanner's
  // see https://github.com/ros-planning/navigation/issues/262
  // If NavFN is used then the tolerance you put here does not matter;
  // the tolerance of the plan is actually controlled by NavFN's
  // `default_tolerance` in its configuration file. *CAUTION*
  request->tolerance = 10.0;
}


/*******************************************************************************
 * @brief Finds the transform between the laser frame and the base frame
 * @param[in] frame_id [const::string&] The laser's frame id
 * @return [bool] True when the transform was found, false otherwise.
 */
bool CRFIDFollow::getBaseToLaserTf(const std::string& frame_id)
{
  ros::Time t = ros::Time::now();
  std::string base_frame = "base_footprint";

  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener_.waitForTransform(base_frame, frame_id, t, ros::Duration(1.0));

    // The direction of the transform returned will be from the base_frame
    // to the frame_id. Which if applied to data, will transform data in
    // the frame_id into the base_frame.
    tf_listener_.lookupTransform(base_frame, frame_id, t, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("[CRFIDFollow] Could not get initial transform from");
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
CRFIDFollow::getCurrentLaserPose(const geometry_msgs::Pose& robot_pose)
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
*/
std::istream& CRFIDFollow::ignoreline(
  std::ifstream& in, std::ifstream::pos_type& pos)
{
  pos = in.tellg();
  return in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}


/*******************************************************************************
 * Create the logfiles and register the headers.
 */
void CRFIDFollow::initLogfiles(void)
{
  // Create the robot pose logfile ---------------------------------------------
  std::ofstream pose_robot_file(pose_robot_filename_.c_str());

  if (pose_robot_file.is_open())
    pose_robot_file.close();
  else
    ROS_ERROR("[CRFIDFollow] Robot pose file not open");

  // Create the velocity logfile -----------------------------------------------
  std::ofstream velocity_file(velocity_filename_.c_str());

  if (velocity_file.is_open())
    velocity_file.close();
  else
    ROS_ERROR("[CRFIDFollow] Velocity file not open");

  // Create pose logfiles per reader and per antenna ---------------------------
  for (int r = 0; r < num_readers_; r++)
  {
    if (active_readers_[r])
    {
      for (int a = 0; a < 4; a++)
      {
        if (readers_active_antennas_[r][a])
        {
          // amcl
          std::string filename =
            pose_filename_ + "_reader_" + readers_macs_[r] + "_antenna_" + std::to_string(a+1) + ".txt";

          std::ofstream fil(filename.c_str());

          if (fil.is_open())
            fil.close();
          else
            ROS_ERROR("[CRFIDFollow] Pose file not open");
        }
      }
    }
  }

  // Create antennas polarities file per reader and per antenna ----------------
  std::ofstream pfil(antennas_polarities_filename_.c_str());

  if (pfil.is_open())
    pfil.close();
  else
    ROS_ERROR("[CRFIDFollow] Antennas' polarities file not open");
}


/*******************************************************************************
 * @brief
 * @param[in] robot_pose [const geometry_msgs::PoseWithCovarianceStamped::Ptr&]
 * The robot's pose.
 * @return [sensor_msgs::LaserScan::Ptr] The map scan in LaserScan form.
 */
bool CRFIDFollow::isGoalFeasible(const std::pair<double,double>& goal,
  const bool& do_return_plan, nav_msgs::Path::Ptr plan_to_goal)
{
  double plan_start_x = latest_pose_.pose.pose.position.x;
  double plan_start_y = latest_pose_.pose.pose.position.y;

  double plan_goal_x = goal.first;
  double plan_goal_y = goal.second;

  // Is there a plan from (plan_start_) to (plan_goal_) ?
  // Check that the goal is a valid configuration for the robot in space,
  // i.e. the configuration of the robot at the goal is feasible;
  // otherwise the global planner will not permit an infeasible configuration
  // and it will deny navigation
  // https://www.programmersought.com/article/85495009501/
  // Another path: https://answers.ros.org/question/303644/check-that-a-pose-is-free-in-a-costmap/


  // Fill the service with these start and goal poses
  // The `/move_base/make_plan` service will be subsequently called with
  // these start and goal poses
  nav_msgs::GetPlan path_plan_srv;
  fillPathRequest(&path_plan_srv.request,
    plan_start_x, plan_start_y, plan_goal_x, plan_goal_y);


  // Call the `make_plan` service
  ROS_INFO("[CRFIDFollow] Calling planning service for goal feasibility");
  bool path_exists = callPlanningService(make_plan_client_, path_plan_srv);

  if (path_exists && do_return_plan)
  {
    plan_to_goal =
      boost::make_shared<nav_msgs::Path>(path_plan_srv.response.plan);
    /*
    // Go to the last point provided by the path planning service:
    // the goal set may actually be infeasible and the planning service
    // itself includes a tolerance---the last goal is feasible if the path exists
    unsigned int path_size = path_plan_srv.response.plan.poses.size();
    double goal_x =
    path_plan_srv.response.plan.poses[path_size-1].pose.position.x;
    double goal_y =
    path_plan_srv.response.plan.poses[path_size-1].pose.position.y;
    */
  }
  else
    ROS_WARN("[CRFIDFollow] Goal is not feasible");

  return path_exists;
}



/*******************************************************************************
 * Param loader
 */
void CRFIDFollow::loadParams(void)
{
  nodehandle_.getParam(ros::this_node::getName() + "/goal_orientation_filename",
    goal_orientation_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/pose_robot_filename",
    pose_robot_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/pose_filename",
    pose_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/velocity_filename",
    velocity_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/map_png_file",
    map_png_file_);
  nodehandle_.getParam(ros::this_node::getName() +
    "/antennas_polarities_filename", antennas_polarities_filename_);

  nodehandle_.getParam(ros::this_node::getName() + "/pose_topic",
    pose_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/velocity_topic",
    velocity_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/scan_topic",
    scan_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/map_topic",
    map_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/request_lost_tag_topic",
    request_lost_tag_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/lost_tag_detected_topic",
    lost_tag_detected_topic_);

  // The amplitude of the vector defining the target distance
  nodehandle_.getParam(ros::this_node::getName() + "/free_goal_distance",
    free_goal_distance_);

  nodehandle_.getParam(ros::this_node::getName() +
    "/robot_position_to_goal_distance_threshold",
    robot_position_to_goal_distance_threshold_);

  nodehandle_.getParam(ros::this_node::getName() + "/robot_type",
    robot_type_);

  nodehandle_.getParam(ros::this_node::getName() + "/callback_frequency",
    callback_frequency_);

  nodehandle_.getParam(ros::this_node::getName() + "/laser_z_orientation",
    laser_z_orientation_);

  // Load predetermined feasible goals and publish them as markers in rviz
  loadFeasibleGoalsList();

  loadReaderAndAntennaParams();
}


/*******************************************************************************
 * Readers' and antennas' param loader.
 */
void CRFIDFollow::loadReaderAndAntennaParams()
{
  if (robot_type_.compare("rb1") == 0)
    num_readers_ = 2;
  else if (robot_type_.compare("turtlebot") == 0)
    num_readers_ = 1;
  else
    ROS_ERROR("[CRFIDFollow] FALSE ROBOT TYPE");

  // Active readers ------------------------------------------------------------
  bool reader_1_active;
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/active",
    reader_1_active);
  active_readers_.push_back(reader_1_active);

  // MAC address of reader_1 ---------------------------------------------------
  std::string reader_1_mac;
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/sn",
    reader_1_mac);
  readers_macs_.push_back(reader_1_mac);

  // rb1 is equipped with 2 readers; turtlebot with 1
  if (robot_type_.compare("rb1") == 0)
  {
    bool reader_2_active;
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/active",
      reader_2_active);
    active_readers_.push_back(reader_2_active);

    std::string reader_2_mac;
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/sn",
      reader_2_mac);
    readers_macs_.push_back(reader_2_mac);
  }


  // Reader 1, active antennas -------------------------------------------------
  std::vector<bool> reader_1_active_antennas;

  bool reader_1_active_antenna_1;
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_1/active",
    reader_1_active_antenna_1);
  reader_1_active_antennas.push_back(reader_1_active_antenna_1);

  bool reader_1_active_antenna_2;
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_2/active",
    reader_1_active_antenna_2);
  reader_1_active_antennas.push_back(reader_1_active_antenna_2);

  bool reader_1_active_antenna_3;
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_3/active",
    reader_1_active_antenna_3);
  reader_1_active_antennas.push_back(reader_1_active_antenna_3);

  bool reader_1_active_antenna_4;
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_4/active",
    reader_1_active_antenna_4);
  reader_1_active_antennas.push_back(reader_1_active_antenna_4);

  readers_active_antennas_.push_back(reader_1_active_antennas);


  // Reader 2, active antennas -------------------------------------------------
  if (robot_type_.compare("rb1") == 0)
  {
    std::vector<bool> reader_2_active_antennas;

    bool reader_2_active_antenna_1;
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_1/active",
      reader_2_active_antenna_1);
    reader_2_active_antennas.push_back(reader_2_active_antenna_1);

    bool reader_2_active_antenna_2;
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_2/active",
      reader_2_active_antenna_2);
    reader_2_active_antennas.push_back(reader_2_active_antenna_2);

    bool reader_2_active_antenna_3;
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_3/active",
      reader_2_active_antenna_3);
    reader_2_active_antennas.push_back(reader_2_active_antenna_3);

    bool reader_2_active_antenna_4;
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_4/active",
      reader_2_active_antenna_4);
    reader_2_active_antennas.push_back(reader_2_active_antenna_4);

    readers_active_antennas_.push_back(reader_2_active_antennas);
  }


  // Antenna poses -------------------------------------------------------------
  for (int r = 0; r < num_readers_; r++)
  {
    std::vector< std::vector<double> > dv2;
    for (int a = 0; a < 4; a++)
    {
      std::vector<double> dv1;
      for (int i = 0; i < 3; i++)
      {
        dv1.push_back(0.0);
      }

      dv2.push_back(dv1);
    }

    reader_antenna_poses_.push_back(dv2);
  }

  // Reader 1, antenna 1
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_1/x",
    reader_antenna_poses_[0][0][0]);
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_1/y",
    reader_antenna_poses_[0][0][1]);
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_1/z",
    reader_antenna_poses_[0][0][2]);

  // Reader 1, antenna 2
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_2/x",
    reader_antenna_poses_[0][1][0]);
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_2/y",
    reader_antenna_poses_[0][1][1]);
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_2/z",
    reader_antenna_poses_[0][1][2]);

  // Reader 1, antenna 3
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_3/x",
    reader_antenna_poses_[0][2][0]);
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_3/y",
    reader_antenna_poses_[0][2][1]);
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_3/z",
    reader_antenna_poses_[0][2][2]);

  // Reader 1, antenna 4
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_4/x",
    reader_antenna_poses_[0][3][0]);
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_4/y",
    reader_antenna_poses_[0][3][1]);
  nodehandle_.getParam(ros::this_node::getName() + "/reader_1/antenna_4/z",
    reader_antenna_poses_[0][3][2]);


  if (robot_type_.compare("rb1") == 0)
  {
    // Reader 2, antenna 1
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_1/x",
      reader_antenna_poses_[1][0][0]);
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_1/y",
      reader_antenna_poses_[1][0][1]);
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_1/z",
      reader_antenna_poses_[1][0][2]);

    // Reader 2, antenna 2
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_2/x",
      reader_antenna_poses_[1][1][0]);
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_2/y",
      reader_antenna_poses_[1][1][1]);
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_2/z",
      reader_antenna_poses_[1][1][2]);

    // Reader 2, antenna 3
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_3/x",
      reader_antenna_poses_[1][2][0]);
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_3/y",
      reader_antenna_poses_[1][2][1]);
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_3/z",
      reader_antenna_poses_[1][2][2]);

    // Reader 2, antenna 4
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_4/x",
      reader_antenna_poses_[1][3][0]);
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_4/y",
      reader_antenna_poses_[1][3][1]);
    nodehandle_.getParam(ros::this_node::getName() + "/reader_2/antenna_4/z",
      reader_antenna_poses_[1][3][2]);
  }
}


/*******************************************************************************
 * Loads the allowable goals from the parameter server as a vector of pairs
 */
void CRFIDFollow::loadFeasibleGoalsList()
{
  XmlRpc::XmlRpcValue xml_goals;
  nodehandle_.getParam(ros::this_node::getName() + "/feasible_goals", xml_goals);

  if(xml_goals.getType() != XmlRpc::XmlRpcValue::TypeArray)
    ROS_ERROR("param 'feasible_goals' is not a list");
  else
  {
    for(unsigned int i = 0; i < xml_goals.size(); i++)
    {
      if(xml_goals[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
        ROS_ERROR("feasible_goals[%d] is not a list", i);
      else
      {
        if(xml_goals[i].size() != 2)
          ROS_ERROR("feasible_goals[%d] is not a pair", i);
        else if(xml_goals[i][0].getType() != XmlRpc::XmlRpcValue::TypeDouble ||
          xml_goals[i][1].getType() != XmlRpc::XmlRpcValue::TypeDouble)
          ROS_ERROR("goals[%d] is not a pair of doubles", i);
        else
        {
          double x = xml_goals[i][0];
          double y = xml_goals[i][1];
          feasible_goals_.push_back(std::make_pair(x,y));
        }
      }
    }
  }
}


/*******************************************************************************
 * Stores the map upon receipt. (The map does not change through time)
 */
void CRFIDFollow::mapCallback(const nav_msgs::OccupancyGrid& map_msg)
{
  map_ = map_msg;
  received_map_ = true;
}


/*******************************************************************************
 * callback. when lost tag is detected, then it makes the autorotation flag
 * false
 */
void CRFIDFollow::lostTagDetectedCallback(const std_msgs::Empty& msg)
{
  ROS_INFO("[CRFIDFollow] Requesting autorotation ceases");
  do_autorotation_ = false;
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
unsigned int CRFIDFollow::numRaysFromAngleRange(
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
 * @brief Returns the SQUARED distance between two pairs of points
 * @param[in] p1 [const std::pair<double,double>&] The first point
 * @param[in] p2 [const std::pair<double,double>&] The second point
 * @return [double] The two points' squared distance
 */
double CRFIDFollow::pairSQDistance(const std::pair<double,double>& p1,
  const std::pair<double,double>& p2)
{
  return (p1.first-p2.first)*(p1.first-p2.first) +
    (p1.second-p2.second)*(p1.second-p2.second);
}


/*******************************************************************************
*/
void CRFIDFollow::periodicCallback(const ros::TimerEvent& event)
{
  ROS_INFO("[CRFIDFollow Periodic Callback]");


  // We need two past poses; if at least one is empty then it does not make
  // sense to calculate an angle
  //if (poses_counter_ < 2)
  //return;

  bool generate_new_goal = false;

  // If this is the first call then there are no goals. Therefore the distance
  // between the robot's current pose and the current goal does not exist:
  // calculate a goal anyway.
  if (!is_goal_active_)
    generate_new_goal = true;
  else if ((poses_counter_ > 0 && is_goal_active_))
  {
    double robot_x = latest_pose_.pose.pose.position.x;
    double robot_y = latest_pose_.pose.pose.position.y;
    std::pair<double,double> robot_position = std::make_pair(robot_x, robot_y);

    // If the robot is far away from its current goal then do not generate
    // a new goal
    if (pairSQDistance(
        robot_position, feasible_goals_[current_feasible_goal_idx_]) <=
      robot_position_to_goal_distance_threshold_*
      robot_position_to_goal_distance_threshold_)
    {
      generate_new_goal = true;
      in_goal_radius_ = true;
    }
  }


  ROS_WARN("goal is active: %d", is_goal_active_);
  ROS_WARN("in goal radius: %d", in_goal_radius_);
  ROS_WARN("generating new goal: %d", generate_new_goal);


  // Read the last line from the file that stores the orientations
  ros::Time timestamp;
  double orientation = 0.0;
  //readLastLineFromGoalOrientationsFile(goal_orientation_filename_.c_str(),
  //&timestamp, &orientation);



  // Call aoa service; fill request; dummy example
  /*
     cultureid_rfid_following::AoAMsg p;
     p.aoa.push_back(0.0);

     cultureid_rfid_following::AoASrv aoa_srv;
     aoa_srv.request.past_aoas.push_back(p);
   */

  // Call aoa service; fill the request with past poses and estimates
  cultureid_rfid_following::AoASrv aoa_srv = constructAOARequest();

  if (aoa_client_.call(aoa_srv))
  {
     //aoa_estimates_.push_back(aoa_srv.response.out_aoa.aoa[0]);
     //aoa_alternative_estimates_.push_back(aoa_srv.response.out_aoa.aoa[1]);
     //latest_rssi_ = aoa_srv.response.out_aoa.aoa[2];
     //aoa_past_estimates_.push_back(aoa_srv.response.out_aoa.aoa[3]);


     //ROS_INFO("est 1: %f", 180*aoa_estimates_.back()/M_PI);
     //ROS_INFO("est 2: %f", 180*aoa_alternative_estimates_.back()/M_PI);
     //ROS_INFO("rssi :%f", latest_rssi_);
     //ROS_INFO("past est: %f", 180*aoa_past_estimates_.back()/M_PI);

    aoa_estimates_.push_back(aoa_srv.response.out_aoa.aoa[0]);
    latest_rssi_ = aoa_srv.response.out_aoa.aoa[1];
    bool latest_reliability = aoa_srv.response.out_aoa.aoa[2];

    ROS_WARN("aoa estimate = %f", 180/M_PI * aoa_estimates_.back());
    ROS_WARN("aoa rssi     = %f", latest_rssi_);
    ROS_WARN("aoa reliable = %d", latest_reliability);

    // Store the latest reliable orientation and its reliability
    if (latest_reliability > 0.5)
    {
      g2g_reliability_flags_.push_back(true);
      latest_reliable_orientation_ = aoa_srv.response.out_aoa.aoa[0];
      latest_reliable_pose_ = latest_pose_;
    }
    else
      g2g_reliability_flags_.push_back(false);


    timestamp = ros::Time::now();
  }
  else
  {
    ROS_ERROR("[CRFIDFollow] Failed to call aoa service");
    return;
  }


  // TODO; questionable
  //orientation = aoa_estimates_.back();

  // ---- THE BELOW IS FALSE ----
  // ---- Clearing the costmap does not occur instantaneously, and this causes
  // the global planner to lack the obstacle layer momentarily. But only a
  // moment is needed for the global planner to plan **through** obstacles when
  // lacking the obstacle layer. The obstacle layer was swapped out in favor of
  // a spatio-temporal voxel layer. (The culprit was the RGBD's layer and not
  // the  laser's obstacle layer)
  //
  // Clear costmaps: as the structure supporting the RFID tag moves (the person)
  // *in front of the robot*, its presence is detected in the LIDAR
  // measurements and hence the local costmap is filled with its (vanishing)
  // presence. This may result in regions being marked as occupied in the
  // local costmap and hence no valid plan may be found, or the plan may result
  // in zig-zags. So clear the costmaps at every goal calculation.
  //std_srvs::Empty mt_srv;
  //clear_costmaps_client_.call(mt_srv);


  // Do not execute anything below this instruction if there is no goal
  if (!generate_new_goal)
    return;

  // If execution has reached this point then either
  // 1. This is the first time a goal is to be calculated, or
  // 2. The robot has exited autorotation, or
  // 3. The robot has reached its goal within a given radius

  // If there is no goal, then wait for the first reliable estimate;
  // else ignore
  if (!is_goal_active_)
  {
    ROS_WARN("I have no goal");

    if (g2g_reliability_flags_.back())
    {
      orientation = latest_reliable_orientation_;
      ROS_WARN("orientation = %f", orientation);
    }
    else
    {
      ROS_WARN("No reliable estimate; returning...");
      return;
    }
  }

  // If the robot is in the vicinity of its goal, then
  // sum the reliability flags for this round of goal-to-goal navigation.
  // If there was even one reliable in those, then compute the next goal given
  // that reliable orientation
  if (in_goal_radius_)
  {
    int num_reliables = std::accumulate(g2g_reliability_flags_.begin(),
      g2g_reliability_flags_.end(), 0);

    ROS_WARN("Within goal radius");
    ROS_WARN("Reliable estimates so far: %u/%lu", num_reliables, g2g_reliability_flags_.size());

    if (num_reliables > 0)
    {
      // If there is at least one reliable estimate then use it
      // as the orientation of the robot for the next goal
      orientation = latest_reliable_orientation_;
    }
    else
    {
      ROS_WARN("AUTOROTATING");
      is_goal_active_ = false;
      autorotate();
      return;
    }
  }


  ROS_WARN("Calculating goal");



  // Given relative orientation between the robot and the RFID tag
  // (`orientation`) determine where the goal falls. The free goal does not
  // belong in the list of feasible goals
  std::pair<double,double> free_goal =
    determineFreeGoal(latest_reliable_pose_, free_goal_distance_, orientation);

  // Publish the free goal for visualisation purposes
  double robot_yaw = extractYawFromPose(latest_reliable_pose_);
  publishGoalWithOrientation(free_goal, robot_yaw+orientation, free_goal_pub_);


  // Which feasible goal is closest to the free goal?
  unsigned int feasible_goal_idx =
    associateFreeGoalToFeasibleGoal(free_goal, feasible_goals_);

  // false = do not fill in path `plan_to_goal`
  nav_msgs::Path::Ptr plan_to_goal;
  //if (isGoalFeasible(feasible_goals_[feasible_goal_idx], false, plan_to_goal))
  if (true)
  {
    ROS_INFO("[CRFIDFollow] Feasible map-goal is feasible in real world too");

    // Determine orientation at goal
    // Let's say that the orientation at the goal is determined by the current
    // position and the goal position
    double numer = feasible_goals_[feasible_goal_idx].second -
      latest_reliable_pose_.pose.pose.position.y;
    double denom = feasible_goals_[feasible_goal_idx].first -
      latest_reliable_pose_.pose.pose.position.x;
    double goal_yaw = atan2(numer,denom);
    wrapAngle(goal_yaw);

    // Send the goal
    sendGoalSimple(
      feasible_goals_[feasible_goal_idx].first,
      feasible_goals_[feasible_goal_idx].second,
      goal_yaw);

    is_goal_active_ = true;
    in_goal_radius_ = false;
    g2g_reliability_flags_.clear();
    current_feasible_goal_idx_ = feasible_goal_idx;
  }
}


/*******************************************************************************
 * Logs the pose of the robot.
 */
void CRFIDFollow::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  poses_counter_++;

  previous_pose_ = latest_pose_;
  latest_pose_ = msg;


  return;

  // Compute decimals of timestamp
  std::string prefix;

  if (static_cast<double>(msg.header.stamp.nsec) / 10 < 1)
    prefix = "00000000";
  else if (static_cast<double>(msg.header.stamp.nsec) / 100 < 1)
    prefix = "0000000";
  else if (static_cast<double>(msg.header.stamp.nsec) / 1000 < 1)
    prefix = "000000";
  else if (static_cast<double>(msg.header.stamp.nsec) / 10000 < 1)
    prefix = "00000";
  else if (static_cast<double>(msg.header.stamp.nsec) / 100000 < 1)
    prefix = "0000";
  else if (static_cast<double>(msg.header.stamp.nsec) / 1000000 < 1)
    prefix = "000";
  else if (static_cast<double>(msg.header.stamp.nsec) / 10000000 < 1)
    prefix = "00";
  else if (static_cast<double>(msg.header.stamp.nsec) / 100000000 < 1)
    prefix = "0";

  std::string nsec_str = prefix + std::to_string(msg.header.stamp.nsec);

  // Turn quaternion into scalar orientation
  tf::Quaternion q_r_abs(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w);

  tf::Matrix3x3 m_r(q_r_abs);
  double r_roll, r_pitch, r_yaw;
  m_r.getRPY(r_roll, r_pitch, r_yaw);

  // Store the robot's own pose ------------------------------------------------
  std::ofstream pose_robot_file(pose_robot_filename_.c_str(), std::ios::app);

  if (pose_robot_file.is_open())
  {
    pose_robot_file << msg.header.stamp.sec;
    pose_robot_file << ".";
    pose_robot_file << nsec_str;
    pose_robot_file << ", ";
    pose_robot_file << msg.pose.pose.position.x;
    pose_robot_file << ", ";
    pose_robot_file << msg.pose.pose.position.y;
    pose_robot_file << ", ";
    pose_robot_file << msg.pose.pose.position.z;
    pose_robot_file << ", ";
    pose_robot_file << r_yaw;
    pose_robot_file << std::endl;

    pose_robot_file.close();
  }

  // Store pose for each antenna------------------------------------------------
  // Robot pose as tf
  tf::Transform robot_pose_tf;

  robot_pose_tf.setOrigin(
    tf::Vector3(
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z));

  // Turn quaternion into yaw
  tf::Quaternion q_r(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w);

  robot_pose_tf.setRotation(q_r);

  // Create pose per antenna
  for (int r = 0; r < num_readers_; r++)
  {
    if (active_readers_[r])
    {
      for (int a = 0; a < 4; a++)
      {
        if (readers_active_antennas_[r][a])
        {
          // Relative antenna pose as tf
          tf::Transform antenna_pose_tf_rel;

          antenna_pose_tf_rel.setOrigin(
            tf::Vector3(
              reader_antenna_poses_[r][a][0],
              reader_antenna_poses_[r][a][1],
              reader_antenna_poses_[r][a][2]));

          tf::Quaternion q_a;
          q_a.setRPY(0.0, 0.0, 0.0);
          q_a.normalize();
          antenna_pose_tf_rel.setRotation(q_a);

          // Absolute antenna pose as tf
          tf::Transform antenna_pose_tf_abs = robot_pose_tf * antenna_pose_tf_rel;

          geometry_msgs::Pose antenna_pose;
          antenna_pose.position.x = antenna_pose_tf_abs.getOrigin().getX();
          antenna_pose.position.y = antenna_pose_tf_abs.getOrigin().getY();
          antenna_pose.position.z = antenna_pose_tf_abs.getOrigin().getZ();

          antenna_pose.orientation.x = antenna_pose_tf_abs.getRotation().getX();
          antenna_pose.orientation.y = antenna_pose_tf_abs.getRotation().getY();
          antenna_pose.orientation.z = antenna_pose_tf_abs.getRotation().getZ();
          antenna_pose.orientation.w = antenna_pose_tf_abs.getRotation().getW();

          tf::Quaternion q_a_abs(
            antenna_pose.orientation.x,
            antenna_pose.orientation.y,
            antenna_pose.orientation.z,
            antenna_pose.orientation.w);

          tf::Matrix3x3 m(q_a_abs);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);

          std::string filename =
            pose_filename_ + "_reader_" + readers_macs_[r] + "_antenna_" + std::to_string(a+1) + ".txt";

          std::ofstream pose_file(filename.c_str(), std::ios::app);

          if (pose_file.is_open())
          {
            pose_file << msg.header.stamp.sec;
            pose_file << ".";
            pose_file << nsec_str;
            pose_file << ", ";
            pose_file << antenna_pose.position.x;
            pose_file << ", ";
            pose_file << antenna_pose.position.y;
            pose_file << ", ";
            pose_file << antenna_pose.position.z;
            pose_file << ", ";
            pose_file << yaw;
            pose_file << std::endl;

            pose_file.close();
          }
        }
      }
    }
  }
}


/*******************************************************************************
 * Publishes feasible goals as markers in rviz
 */
void CRFIDFollow::publishAllFeasibleGoalsMarkers()
{
  // The marker array to be published
  visualization_msgs::MarkerArray marker_array;

  for (unsigned int i = 0; i < feasible_goals_.size(); i++)
  {
    // Create the marker
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();

    // Set the namespace and id for this marker.
    // This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "all_feasible_goals";
    marker.id = i;

    // Set the marker type
    marker.type = visualization_msgs::Marker::CYLINDER;

    // Set the marker action
    // Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.
    // This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = feasible_goals_[i].first;
    marker.pose.position.y = feasible_goals_[i].second;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_array.markers.push_back(marker);
  }

  feasible_goals_vis_pub_.publish(marker_array);
}


/*******************************************************************************
 * Publishes a goal (std::pair) at a given orientation
 */
void CRFIDFollow::publishGoalWithOrientation(
  const std::pair<double,double>& goal,
  const double& orientation,
  const ros::Publisher& goal_pub)
{
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, orientation);
  q.normalize();

  tf::Transform goal_tf;

  goal_tf.setOrigin(
    tf::Vector3(goal.first, goal.second, 0.0));
  goal_tf.setRotation(q);

  geometry_msgs::PoseStamped goal_msg;
  goal_msg.pose.position.x = goal_tf.getOrigin().getX();
  goal_msg.pose.position.y = goal_tf.getOrigin().getY();
  goal_msg.pose.position.z = goal_tf.getOrigin().getZ();
  goal_msg.pose.orientation.x = goal_tf.getRotation().getX();
  goal_msg.pose.orientation.y = goal_tf.getRotation().getY();
  goal_msg.pose.orientation.z = goal_tf.getRotation().getZ();
  goal_msg.pose.orientation.w = goal_tf.getRotation().getW();
  goal_msg.header.stamp = ros::Time::now();
  goal_msg.header.frame_id = "/map";

  goal_pub.publish(goal_msg);
}


/*******************************************************************************
 * Read the last entry from the goal orientations file --- level 1 function
 */
void CRFIDFollow::readLastLineFromGoalOrientationsFile(
  const char* filepath,
  double* timestamp,
  double* orientation)
{
  std::ifstream file(filepath);

  if (file)
  {
    std::string line = readLastLineFromGoalOrientationsFile(file);

    std::istringstream iss(line);

    std::string ts_str;
    std::getline(iss, ts_str, ',');
    std::string t_str;
    std::getline(iss, t_str, ',');


    *timestamp = stod(ts_str);
    *orientation = stod(t_str);
  }
  else
    ROS_ERROR("[CRFIDFollow] ERROR IN OPENING GOAL ORIENTATIONS FILE");
}


/*******************************************************************************
 * Read the last entry from the goal orientations file --- level 0 function
 */
std::string CRFIDFollow::readLastLineFromGoalOrientationsFile(std::ifstream& in)
{
  std::ifstream::pos_type pos = in.tellg();

  std::ifstream::pos_type lastPos;
  while (in >> std::ws && ignoreline(in, lastPos))
    pos = lastPos;

  in.clear();
  in.seekg(pos);

  std::string line;
  std::getline(in, line);
  return line;
}


/*******************************************************************************
 * Publishes a message to a topic that should initiate a tag recovery
 */
void CRFIDFollow::requestLostTagDetection()
{
  std_msgs::Empty mt_msg;
  request_lost_tag_detection_pub_.publish(mt_msg);
}


/*******************************************************************************
 * The laser scan callback
 */
void CRFIDFollow::scanCallback(
  const sensor_msgs::LaserScan::Ptr& scan_msg)
{
  // **** if first scan, cache the tf from base to the scanner

  if (!received_scan_)
  {
    // cache the static tf from base to laser
    if (!getBaseToLaserTf(scan_msg->header.frame_id))
    {
      ROS_WARN("[CRFIDFollow] Skipping scan");
      return;
    }

    received_scan_ = true;
  }

  // Store the latest scan
  latest_scan_ =
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
CRFIDFollow::scanMap(
  const geometry_msgs::PoseWithCovarianceStamped::Ptr& robot_pose)
{
  // Get the laser's pose: the map scan needs that one, not the pose of the
  // robot!
  const geometry_msgs::Pose current_laser_pose =
    getCurrentLaserPose(robot_pose->pose.pose);

  sensor_msgs::LaserScan::Ptr laser_scan_info =
    boost::make_shared<sensor_msgs::LaserScan>(*latest_scan_);

  sensor_msgs::LaserScan::Ptr map_scan;
  double min_a = laser_scan_info->angle_min;
  double inc_a = laser_scan_info->angle_increment;

  // Scan the map using the occupancy_grid_utils method

  // Convert laser position to grid coordinates
  float x = current_laser_pose.position.x / map_.info.resolution;
  float y = map_.info.height - 1 -
    current_laser_pose.position.y / map_.info.resolution;
  float a = extractYawFromPose(current_laser_pose);

  // How many rays?
  int num_rays = numRaysFromAngleRange(latest_scan_->angle_min,
    latest_scan_->angle_max, laser_scan_info->ranges.size(),
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
    ROS_ERROR("[CRFIDFollow] Please provide a valid value");
    ROS_ERROR("                     for param laser_z_orientation");
  }

  // Iterate over all angles.
  // Calculate range for every angle (calc_range returns range in pixels).
  // The angles start counting negatively.
  // For details see
  // https://github.com/kctess5/range_libc/blob/deploy/docs/RangeLibcUsageandInformation.pdf

  for (unsigned int i = 0; i < num_rays; i++)
    map_scan->ranges.push_back(map_.info.resolution *
      rm_.calc_range(x,y, -a + sgn * (min_a + i * inc_a)));

  map_scan->header.stamp = robot_pose->header.stamp;

  return map_scan;
}


/*******************************************************************************
 * Sends the given goal to the robot via a move base action client
 */
void CRFIDFollow::sendGoal(const double& x, const double& y, const double& yaw)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = 0.0;

  tf::quaternionTFToMsg(q, goal.target_pose.pose.orientation);
  move_base_goal_client_.sendGoal(goal);

  ROS_INFO("[CRFIDFollow] Goal sent!");
}


/*******************************************************************************
 * Sends the given goal to the robot via /move_base_simple/goal
 */
void CRFIDFollow::sendGoalSimple(const double& x, const double& y,
  const double& yaw)
{
  geometry_msgs::PoseStamped goal_msg;
  goal_msg.header.frame_id = "map";

  tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, yaw);

  goal_msg.pose.position.x = x;
  goal_msg.pose.position.y = y;
  goal_msg.pose.position.z = 0.0;

  tf::quaternionTFToMsg(q, goal_msg.pose.orientation);

  goal_pub_.publish(goal_msg);

  ROS_INFO("[CRFIDFollow] Goal sent!");
}



/*******************************************************************************
 * A sign function
 */
int CRFIDFollow::sgn(const double& x)
{
  return (x > 0) - (x < 0);
}


/*******************************************************************************
 * Given a PoseStamped message, this method unwraps its contents in order to
 * be easily fed to the client calling aoa.
 * Returns timestamp,x,y,theta of one PoseStamped msg
 */
  std::vector<double>
CRFIDFollow::unwrapAntennaPose(const geometry_msgs::PoseStamped& antenna_pose)
{
  // Make timestamp into double ----
  // Compute decimals of timestamp
  std::string prefix;

  if (static_cast<double>(antenna_pose.header.stamp.nsec) / 10 < 1)
    prefix = "00000000";
  else if (static_cast<double>(antenna_pose.header.stamp.nsec) / 100 < 1)
    prefix = "0000000";
  else if (static_cast<double>(antenna_pose.header.stamp.nsec) / 1000 < 1)
    prefix = "000000";
  else if (static_cast<double>(antenna_pose.header.stamp.nsec) / 10000 < 1)
    prefix = "00000";
  else if (static_cast<double>(antenna_pose.header.stamp.nsec) / 100000 < 1)
    prefix = "0000";
  else if (static_cast<double>(antenna_pose.header.stamp.nsec) / 1000000 < 1)
    prefix = "000";
  else if (static_cast<double>(antenna_pose.header.stamp.nsec) / 10000000 < 1)
    prefix = "00";
  else if (static_cast<double>(antenna_pose.header.stamp.nsec) / 100000000 < 1)
    prefix = "0";

  std::string nsec_str = prefix + std::to_string(antenna_pose.header.stamp.nsec);
  std::string sec_str =  std::to_string(antenna_pose.header.stamp.sec);
  std::string ts = sec_str + "." + nsec_str;
  double timestamp = std::stod(ts);

  double x = antenna_pose.pose.position.x;
  double y = antenna_pose.pose.position.y;
  double yaw = extractYawFromPose(antenna_pose.pose);

  std::vector<double> return_vector;
  return_vector.push_back(timestamp);
  return_vector.push_back(x);
  return_vector.push_back(y);
  return_vector.push_back(yaw);

  return return_vector;
}


/*******************************************************************************
 * Logs the velocity of the robot.
 */
void CRFIDFollow::velocityCallback(const geometry_msgs::Twist& msg)
{
  return;
  ros::WallTime ts = ros::WallTime::now();

  std::ofstream velocity_file(velocity_filename_.c_str(), std::ios::app);

  if (velocity_file.is_open())
  {
    std::string prefix;

    if (static_cast<double>(ts.nsec) / 10 < 1)
      prefix = "00000000";
    else if (static_cast<double>(ts.nsec) / 100 < 1)
      prefix = "0000000";
    else if (static_cast<double>(ts.nsec) / 1000 < 1)
      prefix = "000000";
    else if (static_cast<double>(ts.nsec) / 10000 < 1)
      prefix = "00000";
    else if (static_cast<double>(ts.nsec) / 100000 < 1)
      prefix = "0000";
    else if (static_cast<double>(ts.nsec) / 1000000 < 1)
      prefix = "000";
    else if (static_cast<double>(ts.nsec) / 10000000 < 1)
      prefix = "00";
    else if (static_cast<double>(ts.nsec) / 100000000 < 1)
      prefix = "0";

    std::string nsec_str = prefix + std::to_string(ts.nsec);

    velocity_file << ts.sec;
    velocity_file << ".";
    velocity_file << nsec_str;
    velocity_file << ", ";
    velocity_file << msg.linear.x;
    velocity_file << ", ";
    velocity_file << msg.linear.y;
    velocity_file << ", ";
    velocity_file << msg.linear.z;
    velocity_file << ", ";
    velocity_file << msg.angular.x;
    velocity_file << ", ";
    velocity_file << msg.angular.y;
    velocity_file << ", ";
    velocity_file << msg.angular.z;
    velocity_file << std::endl;

    velocity_file.close();
  }
}


/*******************************************************************************
 * @brief Wraps an angle in the [-π, π] interval
 * @param[in,out] angle [double&] The angle to be expressed in [-π,π]
 * @return void
 */
void CRFIDFollow::wrapAngle(double& angle)
{
  angle = fmod(angle + 5*M_PI, 2*M_PI) - M_PI;
}


/*******************************************************************************
 *
 */
void CRFIDFollow::writeReadersAntennasPolarities()
{
  std::ofstream fil(antennas_polarities_filename_.c_str());

  if (fil.is_open())
  {
    for (int r = 0; r < num_readers_; r++)
    {
      if (active_readers_[r])
      {
        for (int a = 0; a < 4; a++)
        {
          if (readers_active_antennas_[r][a])
          {
            fil << readers_macs_[r] << ", ";
            fil << std::to_string(a+1) << ", ";
            fil << sgn(reader_antenna_poses_[r][a][1])*1;
            fil << std::endl;
          }
        }
      }
    }
  }
  else
    ROS_ERROR("[CRFIDFollow] Could not log antennas polarities");
}
