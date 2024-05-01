add_library(relief_pipeline_localisation pipeline_localisation.cpp)

# Range_libc required library
set(BASEPATH "${CMAKE_SOURCE_DIR}")
set(RANGELIB_DIR ${BASEPATH}/include/utils/range_libc)
add_library(relief_pipeline_localisation_lodepng_lib
  ${RANGELIB_DIR}/vendor/lodepng/lodepng.cpp)

#Note we don't link against pcl as we're using header-only parts of the library
target_link_libraries(relief_pipeline_localisation
  relief_pipeline_localisation_lodepng_lib
  ${catkin_LIBRARIES}
  ${csm_LIBRARIES}
  relief_pipeline_localisation_utils_lib)

add_dependencies(relief_pipeline_localisation
  ${csm_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_executable(relief_pipeline_localisation_node
  pipeline_localisation_node.cpp)

target_link_libraries(relief_pipeline_localisation_node
  relief_pipeline_localisation
  ${catkin_LIBRARIES}
)

add_dependencies(relief_pipeline_localisation_node
  ${catkin_EXPORTED_TARGETS}
)
