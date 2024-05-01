#!/bin/bash

# Launch robot
#screen -S TIFF_robot -d -m roslaunch relief_devel rb1_bringup_live.launch
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://relief-nuc-0:11311
export ROBOT_TYPE=turtlebot
export NUM_CAMERAS=1

roslaunch relief_devel turtlebot_bringup_live.launch &
sleep 5
roslaunch relief_devel avanti_live.launch &
sleep 2
roslaunch relief_rfid_antennas_poses_logger avanti_log.launch &
sleep 2
roslaunch relief_rfid_visualisation avanti_visualisation.launch
