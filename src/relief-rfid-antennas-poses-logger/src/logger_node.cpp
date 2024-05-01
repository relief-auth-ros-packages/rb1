#include "logger.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "logger");
  Logger logger;
  ros::spin();

  return 0;
}
