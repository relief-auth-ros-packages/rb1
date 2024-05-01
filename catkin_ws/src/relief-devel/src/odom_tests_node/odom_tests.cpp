#include "odom_tests_node/odom_tests.h"

/*******************************************************************************
 * Constructor. Initializes params / subscribers.
 */
OdomTests::OdomTests(ros::NodeHandle nh) : nh_(nh)
{
  loadParams();

  odom_sub_ = nh_.subscribe(odom_topic_, 1, &OdomTests::odomCallback, this);

  initLogfiles();

  ROS_INFO("[OdomTests] Node initialised");

}



/*******************************************************************************
 * Destructor
 */
OdomTests::~OdomTests(void)
{
  ROS_INFO("[OdomTests] Node destroyed");
}



/*******************************************************************************
 * Logs an odometry message.
 */
void OdomTests::odomCallback(const nav_msgs::Odometry& msg)
{
  ROS_INFO("[Odom Tests] Received odom msg");
  std::ofstream odom_file(odom_filename_.c_str(), std::ios::app);

  if (odom_file.is_open())
  {
    tf::Quaternion q(
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    odom_file << msg.header.stamp.sec;
    odom_file << ", ";
    odom_file << msg.header.stamp.nsec;
    odom_file << ", ";
    odom_file << msg.pose.pose.position.x;
    odom_file << ", ";
    odom_file << msg.pose.pose.position.y;
    odom_file << ", ";
    odom_file << yaw;
    odom_file << ", ";
    odom_file << msg.twist.twist.linear.x;
    odom_file << ", ";
    odom_file << msg.twist.twist.linear.y;
    odom_file << ", ";
    odom_file << msg.twist.twist.angular.x;
    odom_file << ", ";
    odom_file << msg.twist.twist.angular.y;
    odom_file << ", ";
    odom_file << msg.twist.twist.angular.z;
    odom_file << std::endl;

    odom_file.close();
  }
}


/*******************************************************************************
 * Create the logfiles and register the headers.
 */
void OdomTests::initLogfiles(void)
{
  // Create the global plan file
  std::ofstream odom_file(odom_filename_.c_str());

  if (odom_file.is_open())
  {
    odom_file << "secs, nsecs, x, y, yaw, twist_linear_x, twist_linear_y, ";
    odom_file << "twist_angular_x, twist_angular_y, twist_angular_z";
    odom_file << std::endl;
    odom_file.close();
  }
  else
    ROS_ERROR("Odom file not open");
}



/*******************************************************************************
 */
void OdomTests::loadParams(void)
{
  nh_.getParam(ros::this_node::getName() + "/odom_filename", odom_filename_);
  nh_.getParam(ros::this_node::getName() + "/odom_topic", odom_topic_);
}
