#include "tof_to_pointcloud.hpp"

Tof_to_pointcloud::Tof_to_pointcloud(ros::NodeHandle& nh)
    : nh(nh), private_nh("~"), acquisitionAllowed(false)
{
    private_nh.getParam("input_topic", input_topic);
    private_nh.getParam("output_topic", output_topic);
    private_nh.getParam("frame_id", frame_id);
    private_nh.getParam("continuous_mode", continuousMode);

    sub_distance_tof = nh.subscribe(input_topic, 2, &Tof_to_pointcloud::tof_pointcloud_to_pcd, this);
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 2);

    if (!continuousMode)
    {
        // Service server to start acquisition
        sv_pointcloud = nh.advertiseService("/start_acquisition", &Tof_to_pointcloud::startAcquisition, this);
    }

    tof_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

void Tof_to_pointcloud::tof_pointcloud_to_pcd(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    if (!acquisitionAllowed && !continuousMode)
    {
        return;
    }

    // Popolo la pointcloud
    tof_pointcloud->width = 1;
    tof_pointcloud->height = msg->data.size() / 3;
    tof_pointcloud->is_dense = false;
    tof_pointcloud->points.resize(tof_pointcloud->width * tof_pointcloud->height);

    for (size_t idx = 0; idx < msg->data.size() / 3; idx++)
    {
        tof_pointcloud->points[idx].x = msg->data[3 * idx];
        tof_pointcloud->points[idx].y = msg->data[3 * idx + 1];
        tof_pointcloud->points[idx].z = msg->data[3 * idx + 2];
        tof_pointcloud->points[idx].x /= 1000; // converto da mm a m
        tof_pointcloud->points[idx].y /= 1000; // converto da mm a m
        tof_pointcloud->points[idx].z /= 1000; // converto da mm a m
        tof_pointcloud->points[idx].x *= -1;
    }

    pcl::toROSMsg(*tof_pointcloud, pointcloud_msg);
    pointcloud_msg.header.frame_id = frame_id;
    pointcloud_msg.header.stamp = ros::Time::now();

    // Reset the flag if in service mode
    if (!continuousMode)
    {
        acquisitionAllowed = false;
    }

    // Pubblica la pointcloud
    pub_pointcloud.publish(pointcloud_msg);
}

bool Tof_to_pointcloud::startAcquisition(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    acquisitionAllowed = true;
    res.success = true;
    res.message = "Acquisition started";
    return true;
}

void Tof_to_pointcloud::spinner(void)
{
    ros::spinOnce();
}
