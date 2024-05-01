cmake_minimum_required(VERSION 3.1)
project(relief_amcl_mod)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED
  COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    message_filters
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
)

find_package(Boost REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/AMCL.cfg
)

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    nav_msgs
    rosbag
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_msgs
    tf2_ros
  INCLUDE_DIRS include
  LIBRARIES amcl_sensors amcl_map amcl_pf
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(amcl_pf
                    src/relief_amcl_mod/pf/pf.c
                    src/relief_amcl_mod/pf/pf_kdtree.c
                    src/relief_amcl_mod/pf/pf_pdf.c
                    src/relief_amcl_mod/pf/pf_vector.c
                    src/relief_amcl_mod/pf/eig3.c
                    src/relief_amcl_mod/pf/pf_draw.c)

add_library(amcl_map
                    src/relief_amcl_mod/map/map.c
                    src/relief_amcl_mod/map/map_cspace.cpp
                    src/relief_amcl_mod/map/map_range.c
                    src/relief_amcl_mod/map/map_store.c
                    src/relief_amcl_mod/map/map_draw.c)

add_library(amcl_sensors
                    src/relief_amcl_mod/sensors/amcl_sensor.cpp
                    src/relief_amcl_mod/sensors/amcl_odom.cpp
                    src/relief_amcl_mod/sensors/amcl_laser.cpp)
target_link_libraries(amcl_sensors amcl_map amcl_pf)


add_executable(relief_amcl_mod_node src/relief_amcl_mod_node.cpp)

add_dependencies(relief_amcl_mod_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(relief_amcl_mod_node
    amcl_sensors amcl_map amcl_pf
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

install( TARGETS
    relief_amcl_mod_node amcl_sensors amcl_map amcl_pf
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/relief_amcl_mod/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
