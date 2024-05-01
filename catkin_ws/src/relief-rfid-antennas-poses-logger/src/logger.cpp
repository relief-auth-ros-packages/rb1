#include "logger.h"

/*******************************************************************************
 * Constructor. Initializes params / subscribers.
 */
Logger::Logger(void)
{
  loadParams();

  pose_sub_ = nodehandle_.subscribe(pose_topic_, 10,
    &Logger::poseCallback, this);
  pose_pipeline_sub_ = nodehandle_.subscribe(pose_pipeline_topic_, 10,
    &Logger::posePipelineCallback, this);
  velocity_sub_ = nodehandle_.subscribe(velocity_topic_, 1,
    &Logger::velocityCallback, this);

  initLogfiles();

  // Store into a file the polarity of each antenna of each reader, e.g.
  // if it faces left, its polarity is positive (+)
  // if it faces right, its polarity is negative (-)
  // (we use the robot's native reference frame for reference)
  writeReadersAntennasPolarities();

  ROS_INFO("[Logger] Node initialised");
}


/*******************************************************************************
 * Destructor
 */
Logger::~Logger(void)
{
  ROS_INFO("[Logger] Node destroyed");
}


/*******************************************************************************
 * Create the logfiles and register the headers.
 */
void Logger::initLogfiles(void)
{
  // Create the robot pose logfile ---------------------------------------------
  std::ofstream pose_robot_file(pose_robot_filename_.c_str());

  if (pose_robot_file.is_open())
    pose_robot_file.close();
  else
    ROS_ERROR("[Logger] Robot pose file not open");

  // Create the robot pose_pipeline logfile ------------------------------------
  std::ofstream pose_pipeline_robot_file(pose_pipeline_robot_filename_.c_str());

  if (pose_pipeline_robot_file.is_open())
    pose_pipeline_robot_file.close();
  else
    ROS_ERROR("[Logger] Robot pipeline pose file not open");

  // Create the velocity logfile -----------------------------------------------
  std::ofstream velocity_file(velocity_filename_.c_str());

  if (velocity_file.is_open())
    velocity_file.close();
  else
    ROS_ERROR("[Logger] Velocity file not open");

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
            ROS_ERROR("[Logger] Pose file not open");

          // pipeline localisation
          std::string filename_pipeline =
            pose_pipeline_filename_ + "_reader_" + readers_macs_[r] + "_antenna_" + std::to_string(a+1) + ".txt";

          std::ofstream fil_p(filename_pipeline.c_str());

          if (fil_p.is_open())
            fil_p.close();
          else
            ROS_ERROR("[Logger] Pose pipeline file not open");
        }
      }
    }
  }

  // Create antennas polarities file per reader and per antenna ----------------
  std::ofstream pfil(antennas_polarities_filename_.c_str());

  if (pfil.is_open())
    pfil.close();
  else
    ROS_ERROR("[Logger] Antennas' polarities file not open");

}


/*******************************************************************************
 * Param loader. Check logger.h
 */
void Logger::loadParams(void)
{
  nodehandle_.getParam(ros::this_node::getName() + "/pose_robot_filename",
    pose_robot_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/pose_pipeline_robot_filename",
    pose_pipeline_robot_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/pose_filename",
    pose_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/pose_pipeline_filename",
    pose_pipeline_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/velocity_filename",
    velocity_filename_);
  nodehandle_.getParam(ros::this_node::getName() +
    "/antennas_polarities_filename", antennas_polarities_filename_);

  nodehandle_.getParam(ros::this_node::getName() + "/pose_topic",
    pose_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/pose_pipeline_topic",
    pose_pipeline_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/velocity_topic",
    velocity_topic_);

  nodehandle_.getParam(ros::this_node::getName() + "/robot_type",
    robot_type_);

  loadReaderAndAntennaParams();
}

/*******************************************************************************
 * Readers' and antennas' param loader.
 */
void Logger::loadReaderAndAntennaParams()
{
  if (robot_type_.compare("rb1") == 0)
    num_readers_ = 2;
  else if (robot_type_.compare("turtlebot") == 0)
    num_readers_ = 1;
  else
    ROS_ERROR("FALSE ROBOT TYPE");

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
 * Logs the pose of the robot.
 */
void Logger::poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
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
          q_a.setRPY(0.0, 0.0, sgn(reader_antenna_poses_[r][a][1]) * M_PI/2);
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
 * Logs the pipelined pose of the robot.
 */
void Logger::posePipelineCallback(
  const geometry_msgs::PoseWithCovarianceStamped& msg)
{
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
  std::ofstream pose_pipeline_robot_file(pose_pipeline_robot_filename_.c_str(), std::ios::app);

  if (pose_pipeline_robot_file.is_open())
  {
    pose_pipeline_robot_file << msg.header.stamp.sec;
    pose_pipeline_robot_file << ".";
    pose_pipeline_robot_file << nsec_str;
    pose_pipeline_robot_file << ", ";
    pose_pipeline_robot_file << msg.pose.pose.position.x;
    pose_pipeline_robot_file << ", ";
    pose_pipeline_robot_file << msg.pose.pose.position.y;
    pose_pipeline_robot_file << ", ";
    pose_pipeline_robot_file << msg.pose.pose.position.z;
    pose_pipeline_robot_file << ", ";
    pose_pipeline_robot_file << r_yaw;
    pose_pipeline_robot_file << std::endl;

    pose_pipeline_robot_file.close();
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
          q_a.setRPY(0.0, 0.0, sgn(reader_antenna_poses_[r][a][1]) * M_PI/2);
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
            pose_pipeline_filename_ + "_reader_" + readers_macs_[r] + "_antenna_" + std::to_string(a+1) + ".txt";

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
 * Logs the velocity of the robot.
 */
void Logger::velocityCallback(const geometry_msgs::Twist& msg)
{
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
 * A sign function
 */
int Logger::sgn(const double& x)
{
  return (x > 0) - (x < 0);
}


/*******************************************************************************
 *
 */
void Logger::writeReadersAntennasPolarities()
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
    ROS_ERROR("[Logger] Could not log antennas polarities");
}
