cmake_minimum_required(VERSION 2.8.3)
project(relief_rfid_visualisation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  tf2
  tf2_ros
  cv_bridge
  rviz_plugin_manager
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/relief_rfid_visualisation_cfg.cfg)

add_service_files(
  FILES
  ListInventory.srv
  VisualiseAll.srv
  VisualiseNone.srv
  VisualiseThisEPC.srv
  VisualiseThisType.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  message_runtime
  dynamic_reconfigure
  INCLUDE_DIRS include
)



###########
## Build ##
###########



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(relief_rfid_visualisation src/relief_rfid_visualisation.cpp)

#Note we don't link against pcl as we're using header-only parts of the library
target_link_libraries(relief_rfid_visualisation
  ${catkin_LIBRARIES}
  )

add_dependencies(relief_rfid_visualisation
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS})

add_executable(relief_rfid_visualisation_node
  src/relief_rfid_visualisation_node.cpp)

target_link_libraries(relief_rfid_visualisation_node
 relief_rfid_visualisation
  ${catkin_LIBRARIES}
)

add_dependencies(relief_rfid_visualisation_node
  ${catkin_EXPORTED_TARGETS}
)
