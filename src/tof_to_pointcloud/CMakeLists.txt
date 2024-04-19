cmake_minimum_required(VERSION 2.8.3)
project(tof_to_pointcloud)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
)

include_directories(
  include
  ${PCL_DIR}
  ${catkin_INCLUDE_DIRS}
)


add_executable(tof_to_pointcloud src/tof_to_pointcloud.cpp src/tof_to_pointcloud_node.cpp)
target_link_libraries(tof_to_pointcloud ${catkin_LIBRARIES})
