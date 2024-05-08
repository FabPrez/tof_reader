# TOF reader
## Overview
The package is designed to read data from a multi-zone Time-of-Flight (ToF) sensor via a serial port, convert the data into a PointCloud structure compatible with the Point Cloud Library (PCL), and publish it in a ROS message.

**Note:** This package requires that the computation from distance data to pointcloud is already done in the TOF micro.

The 'tof_reader' package consists of two sub-packages:

### 1. read_raw_tof_pointcloud
The 'read_raw_tof_pointcloud' sub-package is responsible for reading data from a connected serial port and publishing the raw sensor data to a ROS topic named /tof_raw_pointcloud_data.

### 2. tof_to_pointcloud
The 'tof_to_pointcloud' sub-package handles the conversion of raw sensor data into a structured pointcloud format and publishes it to a ROS topic named /tof_pointcloud.

## Getting started
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

## Real-time Visualization
You can visualize the TOF sensor data in real-time and launch the two sub-packages correctly using the following command:
```bash
roslaunch tof_to_pointcloud read_tofdata_publishPC.launch
```

## TOF Configuration

To customize the behavior of the `read_raw_tof_pointcloud` sub-package, follow these steps:

1. Navigate to the file `~/projects/merlin_ws/src/tof_reader/read_tof_pointcloud/src/read_raw_tof_pointcloud.py`.

2. Modify the parameters in the section titled "Constants and Configurations". Here, you can tailor the package to your specific TOF sensor by adjusting parameters such as the number of zones, the number of bytes sent by the sensor, etc.

Example:
```python
# Constants and Configurations
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
TIMEOUT = 1
BYTES_PER_POINT = 2
POINTS_PER_ZONE = 64
ZONES = 3
BYTES_PER_READ = BYTES_PER_POINT * POINTS_PER_ZONE * ZONES
PUBLISH_RATE = 10  # in Hz
```

