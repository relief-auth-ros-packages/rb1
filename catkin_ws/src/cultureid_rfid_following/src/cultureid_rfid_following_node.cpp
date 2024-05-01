#include "cultureid_rfid_following.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cultureid_rfid_following");
  CRFIDFollow crfid_follow;

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
