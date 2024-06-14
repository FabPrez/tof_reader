#include "../include/tof_characterization.hpp"

Tof_characterization::Tof_characterization(ros::NodeHandle &nh)
{
    this->nh = nh;
    nh.getParam("/tof_characterization_node/num_measurements", num_measurements);
    ROS_INFO("Number of measurements: %d", num_measurements);
    
    sub_tof_acquisition = nh.subscribe("/tof_pointcloud", 1, &Tof_characterization::store_pointcloud, this);
    tof_pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    int actual_num_measurements = 0;   
}

void Tof_characterization::store_pointcloud(const sensor_msgs::PointCloud2& msg)
{
    pcl::fromROSMsg(msg, *tof_pointcloud);
    ROS_INFO("Pointcloud received");

    characterization_routine_single_measurement();
    characterization_routine_multiple_measurement();

    actual_num_measurements++;
}

void Tof_characterization::characterization_routine_single_measurement()
{   
    // it is executed only for the first measurement
    if (actual_num_measurements > 0) return;

    calculate_average_measurement();
    calculate_std_deviation();
    
}

void Tof_characterization::characterization_routine_multiple_measurement()
{
    calculate_ass_average();
    
}

float Tof_characterization::calculate_average_measurement(void)
{
    float sum = 0;
    for (int i = 0; i < num_points; i++)
    {
        sum += tof_pointcloud->points[i].z;
    }
    single_average = sum / num_points;
    return single_average;
}

void Tof_characterization::calculate_std_deviation(void)
{   
    // Std deviation of a sigle measurement
    for (size_t i = 0; i < num_points; ++i)
    {
        point_deviation[i] = std::sqrt(std::pow(tof_pointcloud->points[i].z - single_average, 2));
    }
}

void Tof_characterization::calculate_ass_average(void)
{
    assolute_average += calculate_average_measurement();
    if (actual_num_measurements ==  num_measurements)
        assolute_average = assolute_average/num_measurements;
}

void Tof_characterization::spinner()
{
    ros::spinOnce();
}
