#ifndef TOF_CHARACTERIZATION_HPP
#define TOF_CHARACTERIZATION_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32MultiArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "generate_tof_map/GenerateHeatmap.h"

class Tof_characterization
{
public:
	Tof_characterization(ros::NodeHandle &nh);
	void spinner(void);
	void generate_heatmap_service(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &name_graph);
	void generate_heatmap_service(const std::vector<double> &data, const std::string &name_graph);
	// void generate_heatmap_service(const std::vector<double> &data1, const std::string &name_graph, const std::vector<double> *data2 = nullptr);

private:
	// Tof prorperties
	const int num_points = 64;

	// Ros stuff
	ros::NodeHandle nh;
	ros::Subscriber sub_tof_acquisition;
	ros::ServiceClient client_generate_tof_map;
	void wait_for_service(void);

	void store_pointcloud(const sensor_msgs::PointCloud2 &msg);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tof_pointcloud;

	// Characterization variables and methods
	int num_measurements; // Num measurements needed for characterization
	int actual_num_measurements;
	double distance;
	void characterization_routine_single_measurement(void);
	void characterization_routine_multiple_measurement(void);
	// single measurement characterization variables
	float single_average = 0;
	std::vector<double> point_deviation;
	void calculate_average_measurement(void);
	void calculate_average_meanerror(void);
	double average;
	void calculate_std_deviation(void);
	// assolute measurement characterization variables
	std::vector<double> assolute_average;
	std::vector<float> variance;
	std::vector<double> squared_deviation_sum;
	std::vector<double> abs_std;
	void calculate_ass_std(void);
	void calculate_average_std(void);
	double std;
	std::vector<double> mean_error;
	std::vector<double> ass_mean_error;
	void calculate_mean_error(void);
	void calculate_ass_mean_error(void);
	void calculate_invalid_pixels(void);
	float total_invalid = 0;
	float invalid;
};

#endif // TOF_CHARACTERIZATION_HPP
