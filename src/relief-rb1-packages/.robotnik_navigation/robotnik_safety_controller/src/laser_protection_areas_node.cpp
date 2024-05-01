#include <ros/ros.h>

#include <robotnik_safety_controller/LaserProtectionAreas.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_protection_areas");
  ros::NodeHandle n;

  LaserProtectionAreas areas(n);
  areas.start();
}
