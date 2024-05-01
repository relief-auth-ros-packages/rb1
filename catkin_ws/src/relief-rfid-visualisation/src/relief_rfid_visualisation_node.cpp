#include "relief_rfid_visualisation.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "relief_rfid_visualisation_node");
  RfidVisualisation rfid_visualisation;
  ros::spin();

  return 0;
}
