#ifndef TOF_TO_POINTCLOUD_HPP
#define TOF_TO_POINTCLOUD_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32MultiArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class Tof_to_pointcloud
{
	public:
		Tof_to_pointcloud(ros::NodeHandle& nh);
		void spinner(void);
	private:
		void tof_pointcloud_to_pcd(const std_msgs::Int32MultiArray::ConstPtr& msg);

        pcl::PointCloud<pcl::PointXYZ>::Ptr tof_pointcloud;

		ros::NodeHandle nh;
		ros::NodeHandle private_nh;		
		
		ros::Subscriber sub_distance_tof;
		ros::Publisher pub_pointcloud;
		sensor_msgs::PointCloud2 pointcloud_msg;

		std::string input_topic;
    	std::string output_topic;
    	std::string frame_id;
};

#endif // TOF_TO_POINTCLOUD_HPP
