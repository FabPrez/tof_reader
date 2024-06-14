#ifndef TOF_CHARACTERIZATION_HPP
#define TOF_CHARACTERIZATION_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32MultiArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class Tof_characterization
{
public:
	Tof_characterization(ros::NodeHandle &nh);
	void spinner(void);

private:
	// Tof prorperties
	const int num_points = 64;

	// Ros stuff
	ros::NodeHandle nh;
	ros::Subscriber sub_tof_acquisition;

	void store_pointcloud(const sensor_msgs::PointCloud2 &msg);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tof_pointcloud;

	// Characterization variables and methods
	int num_measurements; // Num measurements needed for characterization
	int actual_num_measurements;
	void characterization_routine_single_measurement(void);
	void characterization_routine_multiple_measurement(void);
	// single measurement characterization variables
	float single_average = 0;
	std::vector<float> point_deviation;
	float calculate_average_measurement(void);
	void calculate_std_deviation(void);
	// assolute measurement characterization variables
	float assolute_average = 0;
	void calculate_ass_average(void);
};

#endif // TOF_CHARACTERIZATION_HPP
