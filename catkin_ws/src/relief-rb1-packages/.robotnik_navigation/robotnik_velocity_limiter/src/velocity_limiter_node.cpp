#include <robotnik_velocity_limiter/VelocityLimiter.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_limiter");
  ros::NodeHandle n;

  robotnik_navigation::VelocityLimiter velocity_limiter(n);
  velocity_limiter.start();
}
