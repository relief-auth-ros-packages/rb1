#!/bin/bash

# Launch robot
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://RB1B0-180423AE:11311
export ROBOT_TYPE=rb1
export NUM_CAMERAS=3

roslaunch relief_devel rb1_bringup_live.launch &
sleep 5
#roslaunch relief_devel avanti_live.launch &
#sleep 2
#roslaunch relief_rfid_antennas_poses_logger avanti_log.launch &
#sleep 2
#roslaunch relief_rfid_visualisation avanti_visualisation.launch &
#sleep 1
#roslaunch relief_rfid_detection localise_rfid_tags.launch &
#sleep 1
roslaunch waypoint_mapping_plugin rviz.launch
