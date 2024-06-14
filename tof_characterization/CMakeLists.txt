cmake_minimum_required(VERSION 2.8.3)
project(tof_characterization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  generate_tof_map
  pcl_ros
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime generate_tof_map
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tof_characterization src/tof_characterization.cpp src/tof_characterization_node.cpp)
add_dependencies(tof_characterization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tof_characterization ${catkin_LIBRARIES})
