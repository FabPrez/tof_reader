#ifndef TOF_TO_POINTCLOUD_HPP
#define TOF_TO_POINTCLOUD_HPP

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32MultiArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/make_shared.hpp>

class Tof_to_pointcloud
{
public:
    Tof_to_pointcloud(ros::NodeHandle& nh);
	void spinner(void);

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;        
    ros::Subscriber sub_;
    ros::Publisher pub_pointcloud;
    ros::ServiceServer sv_pointcloud;

    pcl::PointCloud<pcl::PointXYZ>::Ptr tof_pointcloud;
    sensor_msgs::PointCloud2 pointcloud_msg;
    bool acquisitionAllowed;
    bool continuousMode;
    bool tof_simulated;
    
    std::string input_topic;
    std::string output_topic;
    std::string frame_id;
	std::string service_name;

    void raw_distance_to_pc(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void simulated_pc_to_pc(const sensor_msgs::PointCloud2::ConstPtr& msg);

    bool startAcquisition(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};

#endif // TOF_TO_POINTCLOUD_HPP

