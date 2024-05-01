#include <robotnik_twist2ackermann/Twist2Ackermann.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist2ackermann");
  ros::NodeHandle n;

  Twist2Ackermann t2a(n);
  t2a.start();
}
