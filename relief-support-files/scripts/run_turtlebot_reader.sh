#!/bin/bash
export ROBOT_TYPE=turtlebot

# Clear logs from previous experiments
bash ~/relief_support_files/clear_logs.sh &

echo "Launching reader application"
cd ~/catkin_ws/src/relief-rfid-detection/application
bin/speedwayr_x86 192.168.2.10 001625127C5A &

# Launch tag localisation method
roslaunch relief_rfid_detection localise_rfid_tags.launch
