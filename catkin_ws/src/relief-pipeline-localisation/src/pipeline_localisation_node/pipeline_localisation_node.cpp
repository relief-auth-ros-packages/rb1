#include <pipeline_localisation_node/pipeline_localisation.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pipeline_localisation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PipelineLocalisation pipeline_localisation(nh, nh_private);
  ros::spin();
  return 0;
}
