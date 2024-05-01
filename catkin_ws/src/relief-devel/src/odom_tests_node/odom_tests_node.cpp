#include "odom_tests_node/odom_tests.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_tests");
  ros::NodeHandle nh;
  OdomTests odom_tests(nh);
  ros::spin();

  return 0;
}
