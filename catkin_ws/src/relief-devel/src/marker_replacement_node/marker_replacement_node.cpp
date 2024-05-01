#include "marker_replacement_node/marker_replacement.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_replacement");
  MarkerReplacement marker_replacement;
  ros::spin();

  return 0;
}
