#!/bin/bash

# Clear logs from previous experiments
rm ~/catkin_ws/src/relief-rfid-detection/results/rfid_locations.txt &
touch ~/catkin_ws/src/relief-rfid-detection/results/rfid_locations.txt &
rm ~/catkin_ws/src/relief-rfid-detection/application_java_*/results_*.txt &
rm ~/catkin_ws/src/relief-rfid-antennas-poses-logger/results/* &
