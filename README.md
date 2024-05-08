# TOF reader
## Overview
The package is designed to read data from a multi-zone Time-of-Flight (ToF) sensor via a serial port, convert the data into a PointCloud structure compatible with the Point Cloud Library (PCL), and publish it in a ROS message.
The 'tof_reader' package consists of two sub-packages:

1. read_raw_tof_pointcloud
The 'read_raw_tof_pointcloud' sub-package is responsible for reading data from a connected serial port and publishing the raw sensor data to a ROS topic named /tof_raw_pointcloud_data.

2. tof_to_pointcloud
The 'tof_to_pointcloud' sub-package handles the conversion of raw sensor data into a structured pointcloud format and publishes it to a ROS topic named /tof_pointcloud.

Create a ROS workspace (if you already have one, you can skip this step):
```
mkdir -p ~/projects/merlin_ws/src
cd ~/projects/merlin_ws
catkin init
```
Then:
```
cd ~/projects/merlin_ws/src <--[use your workspace name] 
git clone https://github.com/FabPrez/tof_reader.git
cd ~/projects/merlin_ws
catkin build
```
il file in 
