# V1.6 (18/11/15)
Updated imu_calibrator
Updated map_nav_manager
Updated poi_manager
Updated rb1_base_common
Updated rb1_base_robot
Updated robotnik_msgs
Updated robotnik_sensors
Updated robotnik_base_hw_lib
Change robotnik_base_hw branch to kinetic-0.9.5
New param in rb1_params.env (activate docker)
Now if we don’t set an IP for the Hokuyo laser, he search the laser in USB port.
Fixed error with map_nav_manager (saving maps)
# V1.5 (18/11/09)
Added rplidar.launch
Change robotnik_base_hw branch to kinetic-0.9.5
Changed map route parameter in map_nav_manager config file because installing the software this way, the directory linked is ‘sources’ instead of each ros package. Now the path looks like: /home/rb1/catkin_ws/sources/...
# V1.4 (18/09/24)
Now there’s a folder with non_deliverable packages for the customer
Generating script to create .deb of the packages
Removing robotnik_navigation from sources and leaving just the libraries
All the libraries are saved on “libraries” folder
A installation file .sh is created automatically
# V1.3
Adding map_nav_manager
# V1.2
Adding docking charger
# V1.1
Adding imu calibrator package


