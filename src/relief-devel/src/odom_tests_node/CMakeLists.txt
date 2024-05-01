add_library(odom_tests odom_tests.cpp)

#Note we don't link against pcl as we're using header-only parts of the library
target_link_libraries(odom_tests
  ${catkin_LIBRARIES}
  )

add_dependencies(odom_tests
  ${catkin_EXPORTED_TARGETS})

add_executable(odom_tests_node
  odom_tests_node.cpp)

target_link_libraries(odom_tests_node
  odom_tests
  ${catkin_LIBRARIES}
)

add_dependencies(odom_tests_node
  ${catkin_EXPORTED_TARGETS}
)
