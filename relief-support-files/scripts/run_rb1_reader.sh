#!/bin/bash
export ROBOT_TYPE=rb1

# Clear logs from previous experiments
#bash ~/relief-support-files/scripts/clear_logs.sh &

cd ~/relief-support-files/scripts/
./clear_logs.sh

# Launch readers
echo "Launching readers applications"

## C++ execution (deprecated)
#cd ~/catkin_ws/src/relief-rfid-detection/application
#bin/speedwayr_x86 192.168.0.190 001625127C5D 4 &

#cd ~/catkin_ws/src/relief-rfid-detection/application
#bin/speedwayr_x86 192.168.0.192 001625127C5F 4 &

## Java
#cd ~/catkin_ws/src/relief-rfid-detection/application_java_001625127C5D
#echo 0 > control.txt
#java -jar 001625127C5D.jar &

cd ~/catkin_ws/src/relief-rfid-detection/application_java_001625127C5F
echo 0 > control.txt
java -jar 001625127C5F.jar &
