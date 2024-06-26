#include "../include/tof_to_pointcloud.hpp"

Tof_to_pointcloud::Tof_to_pointcloud(ros::NodeHandle &nh)
{
    this->nh = nh;
    sub_distance_tof = nh.subscribe("/tof_raw_pointcloud_data", 2, &Tof_to_pointcloud::tof_pointcloud_to_pcd, this);
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/tof_pointcloud", 2);
}

void Tof_to_pointcloud::tof_pointcloud_to_pcd(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    // popolo la pointcloud
    tof_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    tof_pointcloud->width = 1;
    tof_pointcloud->height = msg->data.size() / 3;
    tof_pointcloud->is_dense = false;
    tof_pointcloud->points.resize(tof_pointcloud->width * tof_pointcloud->height);

    for (size_t idx = 0; idx < msg->data.size() / 3; idx++)
    {
        tof_pointcloud->points[idx].x = msg->data[3 * idx];
        tof_pointcloud->points[idx].y = msg->data[3 * idx + 1];
        tof_pointcloud->points[idx].z = msg->data[3 * idx + 2];
        tof_pointcloud->points[idx].x /= 1000; // converto da m a mm
        tof_pointcloud->points[idx].y /= 1000; // converto da m a mm
        tof_pointcloud->points[idx].z /= 1000; // converto da m a mm
        tof_pointcloud->points[idx].x *= -1;
    }

    // converto la pointcloud in messaggio ROS
    pcl::toROSMsg(*tof_pointcloud, pointcloud_msg);
    pointcloud_msg.header.frame_id = "tof_frame";
}

void Tof_to_pointcloud::spinner()
{
    pub_pointcloud.publish(pointcloud_msg);
    ros::spinOnce();
}
