# TOF reader
## Overview
The package is designed to read data from a multi-zone Time-of-Flight (ToF) sensor via a serial port, convert the data into a PointCloud structure compatible with the Point Cloud Library (PCL), and publish it in a ROS message.

**Note:** This package requires that the computation from distance data to pointcloud is already done in the TOF micro.

The 'tof_reader' package consists of two sub-packages:

### 1. read_raw_tof_pointcloud
The 'read_raw_tof_pointcloud' sub-package is responsible for reading data from a connected serial port and publishing the raw sensor data to a ROS topic named /tof_raw_pointcloud_data.

### 2. tof_to_pointcloud
The 'tof_to_pointcloud' sub-package handles the conversion of raw sensor data into a structured pointcloud format and publishes it to a ROS topic named /tof_pointcloud.
### 3. tof_characterization 
Manages all acquisitions necessary for sensor characterization.
### 4. generate_tof_map
Acts as a server to create and save heatmap-style images from ToF sensor data.

---

## Prerequisites
The `tof_reader` package requires Python's `pyserial` library. Install it using pip:
```bash
pip install pyserial
```
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
## HowTo

### Publishing and visualizing tof pointcloud

To visualize the Time-of-Flight (TOF) sensor data in real-time, you can automatically launch the reference system with a static transform using the following command:

```bash
roslaunch tof_to_pointcloud publish_tof_pc.launch
```

### Important parameter when launching
You can choose to read publish pointcloud form tof in a continuos way (publishing to a specific topic):```<arg name="continuous_mode" default="true"/>```
or calling a service to acquire only when needed:```<arg name="continuous_mode" default="false"/>```

Example of service client configuration for discrete mode acquisition:
```cpp
std::string service_name = "/start_acquisition_tof" + std::to_string(i + 1);
ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>(service_name);
```

#### Multiple Sensors Configuration

If you have multiple sensors reading from the same port, you can configure your launch file in a modular way by specifying the number of TOF sensors and their respective reference systems. You can specify the number of tofs by playing with parameters, adding the reference frame as well. Then, you can launch the tof with:

```bash
roslaunch tof_to_pointcloud publish_tof_pc.launch
```
So far, the launch file is configured to mange at max 2 tof. If you need another one, please modify the file "publish_tof_pc.launch" accordingly.

This setup allows for efficient real-time data visualization and management of multiple TOF sensors.


To determine the name of the port through which the data is exchanged:

```bash
sudo dmesg | grep tty
```

And to grant permissions to read data from that port, you need to run the command:
```bash
sudo chmod 666 /dev/portName
```
For example:
```bash
sudo chmod 666 /dev/ttyACM0
```
---

## Tof characterization
Launch this to start the calculations necessary for a single measurement and for those requiring multiple measurements, which can be set when launching the node.

```bash
roslaunch tof_characterization tof_characteriazation.launch num_measurements:=100
```

## TOF Configuration

To customize the behavior of the `read_raw_tof_pointcloud` sub-package, follow these steps:

1. Navigate to the file `~/projects/merlin_ws/src/tof_reader/read_tof_pointcloud/src/read_raw_tof_pointcloud.py`.

2. Modify the parameters in the section titled "Constants and Configurations". Here, you can tailor the package to your specific TOF sensor by adjusting parameters such as the number of zones, the number of bytes sent by the sensor, etc.

Example:
```python
# Constants and Configurations
SERIAL_PORT = '/dev/ttyACM0'
TYPE_BIT_SENT_BY_TOF = 16 # TOF IS SENDING 16 BIT PER DATA: 2 BYTES
TOF_ZONE_NUMBER = 64
POINTS_PER_ZONE = 3 # X, Y, Z COORDINATES
BYTE_PER_POINT = 2 # TOF IS USING 2 BYTES PER POINT
BYTE_PER_READ = TOF_ZONE_NUMBER * POINTS_PER_ZONE * BYTE_PER_POINT
BAUD_RATE = 115200
TIMEOUT = 1
```

