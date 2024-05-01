add_executable(marker_replacement_node
  marker_replacement.cpp
  marker_replacement_node.cpp)

target_link_libraries(marker_replacement_node
  ${catkin_LIBRARIES}
)

add_dependencies(marker_replacement_node
  ${catkin_EXPORTED_TARGETS}
)
