#include "range_to_laserscan_node/range_to_laserscan.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "range_to_laserscan");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  RangeToLaserscan range_to_laserscan(nh, nh_private);
  ros::spin();

  return 0;
}
