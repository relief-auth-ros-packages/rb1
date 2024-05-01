add_executable(range_to_laserscan_node
  range_to_laserscan.cpp
  range_to_laserscan_node.cpp)

target_link_libraries(range_to_laserscan_node
  ${catkin_LIBRARIES}
)

add_dependencies(range_to_laserscan_node
  ${catkin_EXPORTED_TARGETS}
)
