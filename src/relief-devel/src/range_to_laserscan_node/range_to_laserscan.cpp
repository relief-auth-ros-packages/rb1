#include "range_to_laserscan_node/range_to_laserscan.h"

/*******************************************************************************
 * Constructor. Initializes publishers/subscribers.
 */
RangeToLaserscan::RangeToLaserscan(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private)
{
  // Read input and output topics
  if (!nh_private_.getParam ("range_topic_in", range_topic_in_))
    range_topic_in_ = "front_sonar/range_default";
  if (!nh_private_.getParam ("laserscan_topic_out", laserscan_topic_out_))
    laserscan_topic_out_ = "front_sonar/scan/range_default";

  // Subscribe to the sonar's topic
  range_sub_ = nh_private_.subscribe(range_topic_in_, 1,
    &RangeToLaserscan::rangeCallback, this);

  // Advertise laserscan topic
  laserscan_pub_ = nh_private_.advertise<sensor_msgs::LaserScan>(
    laserscan_topic_out_, 1);

  ROS_INFO("[RangeToLaserscan] Node initialised");
}



/*******************************************************************************
 * Destructor
 */
RangeToLaserscan::~RangeToLaserscan(void)
{
  ROS_INFO("[RangeToLaserscan] Node destroyed");
}


/*******************************************************************************
*/
void RangeToLaserscan::rangeCallback(const sensor_msgs::Range& msg)
{
  // Range measurement from sonar sensor
  float min_range = msg.range;

  // Construct the laserscan msg
  sensor_msgs::LaserScan scan_msg;

  // Fill in the blanks
  scan_msg.header = msg.header;
  scan_msg.range_min = msg.min_range;
  scan_msg.range_max = msg.max_range;
  scan_msg.ranges.push_back(min_range);
  scan_msg.angle_min = -msg.field_of_view/2;
  scan_msg.angle_max = +msg.field_of_view/2;

  // Publish the message
  laserscan_pub_.publish(scan_msg);
}
