add_executable(pipeline_debug_quaternion_to_rpy_node
  pipeline_debug_quaternion_to_rpy.cpp
  pipeline_debug_quaternion_to_rpy_node.cpp
  )

target_link_libraries(pipeline_debug_quaternion_to_rpy_node
   ${catkin_LIBRARIES}
)

add_dependencies(pipeline_debug_quaternion_to_rpy_node
  ${catkin_EXPORTED_TARGETS}
)
