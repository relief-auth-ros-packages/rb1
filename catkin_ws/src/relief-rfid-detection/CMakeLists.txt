cmake_minimum_required(VERSION 2.8.3)
project(relief_rfid_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  SetMaxTransVelocity.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
