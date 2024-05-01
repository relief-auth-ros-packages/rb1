#!/bin/bash

# Kill all ros and roslaunch instances
pkill ros &
pkill roslaunch &
pkill rosrun &
pkill rviz &

# Kill readers
#cd ~/catkin_ws/src/relief-rfid-detection/application_java_001625127C5D
#echo 1 > control.txt

#cd ~/catkin_ws/src/relief-rfid-detection/application_java_001625127C5F
#echo 1 > control.txt
