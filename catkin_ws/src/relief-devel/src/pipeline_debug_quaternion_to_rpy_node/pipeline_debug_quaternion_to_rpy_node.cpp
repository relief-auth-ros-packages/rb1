#include "pipeline_debug_quaternion_to_rpy_node/pipeline_debug_quaternion_to_rpy.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pipeline_debug_quaternion_to_rpy");
  PipelineQuaternionToRPY pipeline_quaternion_to_rpy;
  ros::spin();

  return 0;
}
