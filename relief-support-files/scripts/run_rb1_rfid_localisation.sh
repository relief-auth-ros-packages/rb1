#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://RB1B0-180423AE:11311
export ROBOT_TYPE=rb1

# Launch localisation
echo "Launching rfid localisation"

cd ~/relief-support-files/scripts/
./clear_logs.sh

roslaunch relief_rfid_detection localise_rfid_tags.launch &
